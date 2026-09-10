/*
 * M5Paper MP3 Player: streams MP3 files from an SD card to any Bluetooth
 * audio sink, be it headphones, earbuds, a headset or a speaker.
 *
 * Portrait UI. Remembers the last-connected device in /config/last_device.txt
 * (name + MAC) and auto-reconnects on boot. Tap the BT bar to change device.
 *
 * First-time pairing:
 * 1. Put the audio device into pairing mode (see its own manual).
 * 2. Tap the BT bar, pick the device, wait for the "Connected" status.
 * 3. Playback starts automatically. From then on, power-up reconnects.
 */

#include <M5EPD.h>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <algorithm>
#include <esp_avrc_api.h> // Required for AVRCP passthrough commands
#include <esp_gap_bt_api.h>

#include "AudioFileSourceSD.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutput.h"
#include "BluetoothA2DPSource.h"

// ==================== Config ====================
#define SCREEN_W          540
#define SCREEN_H          960
#define RING_FRAMES       (64 * 1024)
#define PREBUFFER_FRAMES  (RING_FRAMES / 4)
#define MAX_DEVICES       20

// Colors. On this panel 0 renders as white (no ink) and 15 as black.
#define C_BG              0     // white paper
#define C_INK             15    // black ink
#define C_MUTED           9     // mid-gray for secondary text
#define C_DIV             4     // very light grey divider
#define C_CARD_BORDER     6     // light grey card outline
#define C_HL_FILL         13    // dark grey highlight fill
#define C_HL_INK          0     // white text on highlight
#define C_ACCENT          15

// Layout (all portrait coords, 540 wide x 960 tall)
#define HEAD_H            100
#define CARD_X            24
#define CARD_Y            116
#define CARD_W            (SCREEN_W - 2 * CARD_X)
#define CARD_H            210
#define TRACKS_LABEL_Y    352
#define TRACKS_Y          392
#define TRACK_ROW_H       60
#define TRACK_ROWS        6
#define CTRL_CY           820
#define BT_BAR_Y          900
#define BT_BAR_H          60

#define SAVED_DEVICE_PATH "/config/last_device.txt"
#define KNOWN_DEVICE_PATH "/config/devices.txt"
#define WIPE_MARKER_PATH  "/config/wiped2"
#define MAX_KNOWN         8

// Devices screen list geometry
#define DEV_LIST_TOP      120
#define DEV_LABEL_H       34
#define DEV_ROW_H         70
#define DEV_LIST_BOT      830

// Volume / position overlay, raised by the side wheel push
#define OVL_X             24
#define OVL_Y             352
#define OVL_W             (SCREEN_W - 2 * OVL_X)
#define OVL_H             320
#define OVL_BAR_X         (OVL_X + 24)
#define OVL_BAR_W         (OVL_W - 48)
#define OVL_BAR_H         44
#define OVL_VOL_Y         (OVL_Y + 76)
#define OVL_POS_Y         (OVL_Y + 216)
#define OVL_GRAB          40

// One bar plus its labels, the only strip a drag has to repaint.
// The panel wants both x and width on a 4 pixel boundary.
#define OVL_STRIP_X       28
#define OVL_STRIP_W       484
#define OVL_STRIP_H       80
#define OVL_STRIP_LIFT    36

// ==================== Globals ====================
M5EPD_Canvas canvas(&M5.EPD);
M5EPD_Canvas barCanvas(&M5.EPD); // one overlay bar, pushed on its own

/*
 * A sink that is paired but idle answers no inquiry, so discovery never finds
 * it a second time. connectKnown skips discovery: it points the library's
 * state machine at an address and lets its heart-beat timer retry until the
 * sink answers.
 */
class A2dpSource : public BluetoothA2DPSource {
public:
    void connectKnown(const esp_bd_addr_t addr) {
        esp_bt_gap_cancel_discovery();
        memcpy(peer_bd_addr, addr, ESP_BD_ADDR_LEN);
        is_autoreconnect_allowed = true;
        set_last_connection(peer_bd_addr);
        reconnect();
    }

    /* The heart beat keeps dialling on its own, which is wrong once the user
     * gives up on a sink or deletes it. */
    void stopRetrying() {
        is_target_status_active = false;
        set_auto_reconnect(false);
    }

    /*
     * start() is the only place the library ever begins an inquiry, so every
     * later scan needs its own kick. Without this the SCAN button did nothing
     * once a link had been made and dropped.
     */
    void startDiscovery() {
        stopRetrying();
        esp_bt_gap_cancel_discovery();
        s_a2d_state = APP_AV_STATE_DISCOVERING;
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    }

    /* Forget has to reach the address the library cached in NVS as well. */
    void forgetTarget() {
        is_autoreconnect_allowed = true;
        clean_last_connection();
        memset(peer_bd_addr, 0, ESP_BD_ADDR_LEN);
        is_autoreconnect_allowed = false;
        stopRetrying();
    }
};

A2dpSource a2dp;

int16_t *ringBuf = nullptr;
/*
 * Single producer (decoder, core 1), single consumer (Bluetooth, core 0).
 * Each side owns one index and publishes it only after its data, so the ring
 * needs no lock. The audio callback must never block: it runs in the radio's
 * time-critical path.
 */
volatile size_t ringR = 0, ringW = 0;

std::vector<String> tracks;
std::vector<int> displayTracks; // Stores indices of filtered tracks
int trackIdx = 0;

enum PlayState { P_STOPPED, P_PLAYING, P_PAUSED };
volatile PlayState playState = P_STOPPED;
String nowPlayingName = "";
String statusMsg = "";
volatile bool uiDirty = false;
volatile bool needFullRefresh = false;

// AVRCP Control Flags (Thread Safe)
volatile bool avrcTogglePlayPending = false;
volatile bool avrcNextPending = false;
volatile bool avrcPrevPending = false;
volatile bool avrcStopPending = false;
volatile int  avrcVolumeSteps = 0;

volatile bool btConnected = false;
volatile bool connEventPending = false;
volatile int  connEventState = -1;
int connectFails = 0;
#define MAX_CONNECT_FAILS 6
enum BtMode { BT_IDLE, BT_SCANNING, BT_CONNECTING };
volatile BtMode btMode = BT_IDLE;
struct Dev { String name; esp_bd_addr_t addr; };
std::vector<Dev> devices;      // found by the last scan
std::vector<Dev> knownDevices; // remembered on the SD card, most recent first

// Devices screen rows, rebuilt on render so touch and paint cannot drift apart
enum DevRowKind { ROW_LBL_SAVED, ROW_LBL_FOUND, ROW_SAVED, ROW_FOUND };
struct DevRow { DevRowKind kind; int idx; int y; };
std::vector<DevRow> devRows;
int devTotalItems = 0;
int devFirstShown = 0;
unsigned long clearArmedMs = 0; // CLEAR asks once before it wipes

// Devices screen bottom row. The band is taller than the buttons look, because
// a thumb at the very bottom edge kept missing them.
#define BTN_Y             860
#define BTN_H             60
#define BTN_BAND_TOP      846
#define BTN_BAND_BOT      940
#define BTN_W             150
#define BTN_BACK_X        30
#define BTN_SCAN_X        195
#define BTN_CLEAR_X       360
SemaphoreHandle_t devMx = nullptr;
String targetName = "";
esp_bd_addr_t targetMac;
bool hasTargetMac = false;
unsigned long scanStartMs = 0;

enum Screen { S_PLAYER, S_DEVICES, S_SEARCH };
Screen screen = S_PLAYER;
String searchQuery = "";

// Keyboard Mapping
struct KeyDef { const char* label; int x; int y; int w; int h; };
std::vector<KeyDef> keys;

bool overlayShown = false;
enum DragTarget { DRAG_NONE, DRAG_VOL, DRAG_SEEK };
DragTarget dragTarget = DRAG_NONE;
float seekFrac = 0.0f;

int devScroll = 0; // index of the first drawn item on the devices screen

// Byte rate of the running track, the only way to turn a file offset into time
volatile uint32_t framesDecoded = 0;
uint32_t trackStartPos = 0;

volatile bool autoPlayPending = false;
bool a2dpStarted = false;
bool hasAutoStarted = false;  // one-shot: auto-play only on first connect per boot
int  a2dpVolume = 110;        // 0..127
#define VOLUME_STEP 8
unsigned long lastFlushMs = 0;

#define LOG(tag, fmt, ...) \
    Serial.printf("[%8lums][%s] " fmt "\n", millis(), tag, ##__VA_ARGS__)

AudioGeneratorMP3 *mp3 = nullptr;
AudioFileSourceSD *src = nullptr;
AudioFileSourceID3 *id3 = nullptr;

// ==================== Forward decls ====================
void renderPlayer();
void renderDevices();
void renderSearch();
void scanFiles(const char *dir);
void updateSearch();
void handleTouchPlayer(int x, int y);
void handleTouchDevices(int x, int y);
void handleTouchSearch(int x, int y);
void startScan();
void playTrack(int i);
void playTrackAt(int i, float frac);
void hideOverlay();
void pauseOrResume();
void stopTrack();
void nextTrack();
void prevTrack();
void enterDevicesScreen();
void enterPlayerScreen();
void loadKnownDevices();
void saveKnownDevices();
void rememberDevice(const String &name, const esp_bd_addr_t addr);
void forgetDevice(int i);
void forgetAllDevices();
void connectTo(const String &name, const esp_bd_addr_t addr);
void beginA2dpIfNeeded(bool autoReconnect);
void setVolume(int v);
void handleConnEvent(int state);

// ==================== AudioOutput -> ring ====================
static inline size_t ringUsed() {
    return (ringW - ringR) & (RING_FRAMES - 1);
}

class RingOutput : public AudioOutput {
public:
    int sourceRate = 44100;
    int targetRate = 44100; // Bluetooth A2DP default
    uint32_t phase = 0; 
    uint32_t phaseInc = (1 << 16); 

    RingOutput() { SetGain(1.0f); }
    bool begin() override { return true; }
    
    // Captures the MP3 file's sample rate when it starts playing
    bool SetRate(int hz) override { 
        sourceRate = hz ? hz : 44100; 
        // Calculate Q16 fractional step for our zero-overhead resampler
        phaseInc = ((uint64_t)targetRate << 16) / sourceRate;
        LOG("ring", "MP3 sample rate: %d Hz (Resampling to %d Hz)", sourceRate, targetRate);
        return true; 
    }
    
    bool ConsumeSample(int16_t s[2]) override {
        uint32_t temp_phase = phase + phaseInc;
        int num_pushes = temp_phase >> 16; 
        
        if (num_pushes > 0) {
            // Check if buffer has enough space for this frame
            if (ringUsed() + num_pushes > RING_FRAMES - 2) return false; // Buffer full, decoder will wait

            size_t w = ringW;
            phase = temp_phase - (num_pushes << 16); // keep the remainder for the next sample
            for (int i = 0; i < num_pushes; i++) {
                ringBuf[w * 2]     = s[0];
                ringBuf[w * 2 + 1] = s[1];
                w = (w + 1) & (RING_FRAMES - 1);
            }
            ringW = w; // publish only once the frames are in place
        } else {
            // Drop sample to downsample (fast forward the phase)
            phase = temp_phase;
        }
        framesDecoded++;
        return true;
    }
    
    bool stop() override { return true; }
};

RingOutput *audioOut = nullptr;

// ==================== A2DP: pull audio ====================
volatile unsigned long a2dpFirstCallMs = 0;
volatile unsigned long a2dpFirstAudioMs = 0;
volatile uint32_t a2dpCallCount = 0;
volatile uint32_t a2dpFramesServed = 0;
volatile uint32_t a2dpSilenceCount = 0;

/*
 * Runs on the Bluetooth task in the radio's time-critical path. No printing
 * and no locks: a UART write from here deadlocks core 0 against the other
 * core's logging, which trips the interrupt watchdog.
 */
int32_t get_sound_data(Frame *data, int32_t len) {
    if (a2dpFirstCallMs == 0) a2dpFirstCallMs = millis();
    a2dpCallCount++;

    size_t avail = ringUsed();
    if (playState != P_PLAYING || avail == 0) {
        a2dpSilenceCount++;
        memset(data, 0, len * sizeof(Frame));
        return len;
    }
    if (a2dpFirstAudioMs == 0) a2dpFirstAudioMs = millis();

    size_t r = ringR;
    for (int i = 0; i < len; i++) {
        if (avail > 0) {
            data[i].channel1 = ringBuf[r * 2];
            data[i].channel2 = ringBuf[r * 2 + 1];
            r = (r + 1) & (RING_FRAMES - 1);
            avail--;
            a2dpFramesServed++;
        } else {
            data[i].channel1 = 0;
            data[i].channel2 = 0;
        }
    }
    ringR = r; // publish only once the frames are consumed
    return len;
}

// ==================== A2DP: callbacks ====================

/*
 * Media keys sent by the connected audio device (AVRCP passthrough).
 *
 * Sinks disagree on which key a skip carries: some send FORWARD/BACKWARD,
 * others only the FAST_FORWARD/REWIND seek keys, so both pairs skip a track.
 * PLAY and PAUSE both toggle, because a sink keeps its own idea of the
 * transport state and that drifts out of sync with ours. A key that is held
 * down repeats, and each repeat is dropped until the release arrives.
 */
void avrc_callback(uint8_t cmd, bool isReleased) {
    static uint8_t heldCmd = 0;
    static unsigned long heldMs = 0;
    static unsigned long lastActionMs = 0;
    unsigned long now = millis();

    if (isReleased) {
        if (cmd == heldCmd) heldCmd = 0;
        return;
    }

    /* A sink that never sends the release must not wedge its key down. */
    if (heldCmd && now - heldMs > 1000) heldCmd = 0;

    if (cmd == heldCmd) { heldMs = now; return; }
    heldCmd = cmd;
    heldMs = now;

    // Strict 400ms debounce prevents double-skips and bouncy toggles
    if (now - lastActionMs < 400) return;
    lastActionMs = now;

    switch (cmd) {
        case ESP_AVRC_PT_CMD_PLAY:
        case ESP_AVRC_PT_CMD_PAUSE:
            avrcTogglePlayPending = true;
            break;
        case ESP_AVRC_PT_CMD_STOP:
            avrcStopPending = true;
            break;
        case ESP_AVRC_PT_CMD_FORWARD:
        case ESP_AVRC_PT_CMD_FAST_FORWARD:
            avrcNextPending = true;
            break;
        case ESP_AVRC_PT_CMD_BACKWARD:
        case ESP_AVRC_PT_CMD_REWIND:
            avrcPrevPending = true;
            break;
        case ESP_AVRC_PT_CMD_VOL_UP:
            avrcVolumeSteps++;
            break;
        case ESP_AVRC_PT_CMD_VOL_DOWN:
            avrcVolumeSteps--;
            break;
    }
}

/*
 * Runs on the Bluetooth task, so it only hands the event over. A String
 * assignment reallocs under the renderer, and the SD card shares its SPI bus
 * with the panel: neither belongs on this stack.
 */
void on_conn_state(esp_a2d_connection_state_t state, void *ptr) {
    LOG("a2dp", "conn_state=%d", (int)state);
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)         btConnected = true;
    else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) btConnected = false;
    connEventState = (int)state;
    connEventPending = true;
}

void handleConnEvent(int state) {
    static int lastPainted = -1;

    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        btMode = BT_IDLE;
        connectFails = 0;
        statusMsg = "Connected";
        a2dp.set_volume(a2dpVolume);
        if (hasTargetMac && targetName.length()) {
            rememberDevice(targetName, targetMac);
        }
        // Auto-play ONLY on the very first connection this boot, and only if
        // we aren't already playing / paused. Otherwise a reconnect would
        // silently override a user's pause.
        if (!hasAutoStarted && !tracks.empty() && playState == P_STOPPED) {
            autoPlayPending = true;
            hasAutoStarted = true;
            LOG("a2dp", "autoplay queued (first connect)");
        }
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        // Keep playState as-is; user's pause/play choice survives a reconnect.
        if (!hasTargetMac) {
            statusMsg = "Disconnected";
        } else if (++connectFails >= MAX_CONNECT_FAILS) {
            a2dp.stopRetrying();
            btMode = BT_IDLE;
            statusMsg = "Cannot reach " + targetName;
            LOG("a2dp", "gave up after %d tries", connectFails);
        } else {
            statusMsg = "Reconnecting " + targetName + "...";
        }
    } else {
        return; // CONNECTING and DISCONNECTING show nothing of their own
    }

    /* The retry loop cycles every few seconds; repainting it flashes the panel. */
    if (state != lastPainted) {
        lastPainted = state;
        uiDirty = true;
    }
}

bool on_ssid(const char *ssid, esp_bd_addr_t addr, int rssi) {
    if (!ssid) return false;
    String s = ssid;
    if (s.length() == 0) return false;

    if (btMode == BT_SCANNING) {
        if (xSemaphoreTake(devMx, portMAX_DELAY) == pdTRUE) {
            bool dup = false;
            for (auto &d : devices) if (d.name == s) { dup = true; break; }
            if (!dup && (int)devices.size() < MAX_DEVICES) {
                Dev d; d.name = s; memcpy(d.addr, addr, ESP_BD_ADDR_LEN);
                devices.push_back(d);
                uiDirty = true;
            }
            xSemaphoreGive(devMx);
        }
        return false;
    }
    if (btMode == BT_CONNECTING) {
        // Prefer MAC match (robust), fall back to name match.
        if (hasTargetMac && memcmp(addr, targetMac, ESP_BD_ADDR_LEN) == 0) {
            return true;
        }
        if (!hasTargetMac && targetName.length() && s == targetName) {
            memcpy(targetMac, addr, ESP_BD_ADDR_LEN);
            hasTargetMac = true;
            return true;
        }
    }
    return false;
}

// ==================== Saved device (SD persistence) ====================
static bool parseMac(const String &str, esp_bd_addr_t out) {
    int b[6];
    if (sscanf(str.c_str(), "%x:%x:%x:%x:%x:%x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
    return true;
}

static String formatMac(const esp_bd_addr_t a) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             a[0], a[1], a[2], a[3], a[4], a[5]);
    return String(buf);
}

void saveKnownDevices() {
    if (!SD.exists("/config")) SD.mkdir("/config");
    SD.remove(KNOWN_DEVICE_PATH);
    File f = SD.open(KNOWN_DEVICE_PATH, FILE_WRITE);
    if (!f) { LOG("saved", "write failed"); return; }
    for (auto &d : knownDevices) {
        f.print(formatMac(d.addr));
        f.print('\t');
        f.println(d.name);
    }
    f.close();
    LOG("saved", "wrote %u known devices", (unsigned)knownDevices.size());
}

/* The one-device file predates the known list, so fold it in and drop it. */
static void migrateSavedDevice() {
    if (!SD.exists(SAVED_DEVICE_PATH)) return;
    File f = SD.open(SAVED_DEVICE_PATH, FILE_READ);
    if (!f) return;
    String name = f.readStringUntil('\n');
    String mac  = f.readStringUntil('\n');
    f.close();
    name.trim();
    mac.trim();
    Dev d;
    if (name.length() && parseMac(mac, d.addr)) {
        d.name = name;
        knownDevices.push_back(d);
        saveKnownDevices();
        LOG("saved", "migrated %s", name.c_str());
    }
    SD.remove(SAVED_DEVICE_PATH);
}

void loadKnownDevices() {
    knownDevices.clear();
    File f = SD.open(KNOWN_DEVICE_PATH, FILE_READ);
    if (f) {
        while (f.available() && (int)knownDevices.size() < MAX_KNOWN) {
            String line = f.readStringUntil('\n');
            line.trim();
            int tab = line.indexOf('\t');
            if (tab < 0) continue;
            Dev d;
            if (!parseMac(line.substring(0, tab), d.addr)) continue;
            d.name = line.substring(tab + 1);
            if (d.name.length()) knownDevices.push_back(d);
        }
        f.close();
    }
    if (knownDevices.empty()) migrateSavedDevice();
    LOG("saved", "%u known devices", (unsigned)knownDevices.size());
}

void rememberDevice(const String &name, const esp_bd_addr_t addr) {
    for (size_t i = 0; i < knownDevices.size(); i++) {
        if (memcmp(knownDevices[i].addr, addr, ESP_BD_ADDR_LEN) == 0) {
            knownDevices.erase(knownDevices.begin() + i);
            break;
        }
    }
    Dev d;
    d.name = name;
    memcpy(d.addr, addr, ESP_BD_ADDR_LEN);
    knownDevices.insert(knownDevices.begin(), d);
    while ((int)knownDevices.size() > MAX_KNOWN) knownDevices.pop_back();
    saveKnownDevices();
}

/*
 * The pairing key lives in the radio's own flash store, not on the SD card.
 * Leaving it behind lets a half-negotiated link come back, and the codec then
 * starts with a zeroed configuration.
 */
static void removeBond(const esp_bd_addr_t addr) {
    if (!a2dpStarted) return;
    esp_bt_gap_remove_bond_device((uint8_t *)addr);
    LOG("bond", "removed %s", formatMac(addr).c_str());
}

static void removeAllBonds() {
    if (!a2dpStarted) return;
    int num = esp_bt_gap_get_bond_device_num();
    if (num <= 0) return;
    esp_bd_addr_t *list = (esp_bd_addr_t *)malloc(sizeof(esp_bd_addr_t) * num);
    if (!list) return;
    if (esp_bt_gap_get_bond_device_list(&num, list) == ESP_OK) {
        for (int i = 0; i < num; i++) esp_bt_gap_remove_bond_device(list[i]);
        LOG("bond", "removed %d bonded devices", num);
    }
    free(list);
}

void forgetAllDevices() {
    LOG("saved", "forget all (%u devices)", (unsigned)knownDevices.size());
    knownDevices.clear();
    SD.remove(KNOWN_DEVICE_PATH);
    SD.remove(SAVED_DEVICE_PATH);
    if (btConnected) a2dp.disconnect();
    if (a2dpStarted) a2dp.forgetTarget();
    removeAllBonds();
    hasTargetMac = false;
    targetName = "";
    btConnected = false;
    btMode = BT_IDLE;
    connectFails = 0;
    statusMsg = "Tap BT to pair device";
}

void forgetDevice(int i) {
    if (i < 0 || i >= (int)knownDevices.size()) return;
    LOG("saved", "forget %s", knownDevices[i].name.c_str());

    bool wasTarget = hasTargetMac &&
        memcmp(knownDevices[i].addr, targetMac, ESP_BD_ADDR_LEN) == 0;
    knownDevices.erase(knownDevices.begin() + i);
    saveKnownDevices();
    if (!wasTarget) return;

    if (btConnected) a2dp.disconnect();
    a2dp.forgetTarget();
    removeBond(targetMac);
    hasTargetMac = false;
    targetName = "";
    btConnected = false;
    btMode = BT_IDLE;
    connectFails = 0;
    statusMsg = "Tap BT to pair device";
    LOG("saved", "target cleared, auto-reconnect off");
}

void connectTo(const String &name, const esp_bd_addr_t addr) {
    targetName = name;
    memcpy(targetMac, addr, ESP_BD_ADDR_LEN);
    hasTargetMac = true;
    btMode = BT_CONNECTING;
    connectFails = 0;
    statusMsg = "Connecting " + targetName + "...";
    if (!a2dpStarted) {
        beginA2dpIfNeeded(true);
    } else {
        // The source holds one link, so the old one goes before the new one.
        if (btConnected) a2dp.disconnect();
        a2dp.connectKnown(targetMac);
    }
    enterPlayerScreen();
}

// ==================== Icons (primitives) ====================
static void iconPlay(int cx, int cy, int size, uint8_t col) {
    int s = size;
    canvas.fillTriangle(
        cx - s / 3,    cy - s / 2,
        cx - s / 3,    cy + s / 2,
        cx + s * 2/3,  cy,
        col);
}

static void iconPause(int cx, int cy, int size, uint8_t col) {
    int barW = size / 4;
    int barH = size;
    int gap  = size / 5;
    canvas.fillRoundRect(cx - gap - barW, cy - barH / 2, barW, barH, 4, col);
    canvas.fillRoundRect(cx + gap,        cy - barH / 2, barW, barH, 4, col);
}

static void iconPrev(int cx, int cy, int size, uint8_t col) {
    int s = size;
    int barW = s / 5;
    canvas.fillRoundRect(cx - s / 2, cy - s / 2, barW, s, 3, col);
    canvas.fillTriangle(
        cx + s / 2,           cy - s / 2,
        cx + s / 2,           cy + s / 2,
        cx - s / 2 + barW + 2, cy,
        col);
}

static void iconNext(int cx, int cy, int size, uint8_t col) {
    int s = size;
    int barW = s / 5;
    canvas.fillTriangle(
        cx - s / 2,           cy - s / 2,
        cx - s / 2,           cy + s / 2,
        cx + s / 2 - barW - 2, cy,
        col);
    canvas.fillRoundRect(cx + s / 2 - barW, cy - s / 2, barW, s, 3, col);
}

static void iconNote(int cx, int cy, int size, uint8_t col) {
    // Simple 8th note: filled head + stem
    int headR = size / 3;
    canvas.fillCircle(cx, cy + size / 3, headR, col);
    canvas.fillRect(cx + headR - 2, cy - size / 2, 3, size - headR / 2, col);
    // tiny flag
    canvas.fillTriangle(
        cx + headR + 1, cy - size / 2,
        cx + headR + 1, cy - size / 4,
        cx + size / 2,  cy - size / 4,
        col);
}

static void iconBt(int cx, int cy, int size, uint8_t col) {
    // True Geometric Bluetooth Logo
    int h = size;
    int w = size * 0.6;
    int x0 = cx - w/2;
    int x1 = cx;
    int x2 = cx + w/2;
    int y0 = cy - h/2;
    int y1 = cy - h/4;
    int y2 = cy;
    int y3 = cy + h/4;
    int y4 = cy + h/2;

    for(int dx = 0; dx < 2; dx++) {
        int sx1 = x1 + dx;
        canvas.drawLine(sx1, y0, sx1, y4, col);
        canvas.drawLine(sx1, y0, x2+dx, y1, col);
        canvas.drawLine(x2+dx, y1, sx1, y2, col);
        canvas.drawLine(sx1, y2, x2+dx, y3, col);
        canvas.drawLine(x2+dx, y3, sx1, y4, col);
        canvas.drawLine(x0+dx, y3, sx1, y2, col);
        canvas.drawLine(x0+dx, y1, sx1, y2, col);
    }
}

static void iconSearch(int cx, int cy, int size, uint8_t col) {
    int r = size / 3;
    // Draw hollow circle twice for thickness
    canvas.drawCircle(cx - r/2, cy - r/2, r, col);
    canvas.drawCircle(cx - r/2, cy - r/2, r-1, col);
    // Handle
    canvas.drawLine(cx, cy, cx + r, cy + r, col);
    canvas.drawLine(cx+1, cy, cx + r + 1, cy + r, col);
    canvas.drawLine(cx, cy+1, cx + r, cy + r + 1, col);
}

void initKeyboard() {
    if (!keys.empty()) return;
    int kw = 50, kh = 70, gap = 4;
    
    // Shifted down closer to the bottom edge for easier typing
    int startY = 640; 
    
    // Row 1
    const char* r1[] = {"Q","W","E","R","T","Y","U","I","O","P"};
    for(int i=0; i<10; i++) keys.push_back({r1[i], 2 + i*(kw+gap), startY, kw, kh});
    
    // Row 2
    const char* r2[] = {"A","S","D","F","G","H","J","K","L"};
    for(int i=0; i<9; i++) keys.push_back({r2[i], 27 + i*(kw+gap), startY + kh + gap, kw, kh});
    
    // Row 3
    const char* r3[] = {"Z","X","C","V","B","N","M"};
    for(int i=0; i<7; i++) keys.push_back({r3[i], 52 + i*(kw+gap), startY + 2*(kh+gap), kw, kh});
    keys.push_back({"DEL", 52 + 7*(kw+gap), startY + 2*(kh+gap), kw*2, kh});
    
    // Row 4
    int y4 = startY + 3*(kh+gap);
    keys.push_back({"CLR", 2, y4, 120, kh});
    keys.push_back({" ", 122+gap, y4, 280, kh});
    keys.push_back({"DONE", 402+gap*2, y4, 130, kh});
}

// ==================== Setup ====================
void setup() {
    M5.begin(true, false, true, true, false);
    Serial.begin(115200);
    delay(100);
    Serial.println("\n\n[M5 MP3] boot");
    static const char *RESET_NAME[] = {
        "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT",
        "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO"};
    int rr = (int)esp_reset_reason();
    LOG("boot", "reset=%s(%d) heap=%u psram=%u batt=%umV",
        rr < 11 ? RESET_NAME[rr] : "?", rr,
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
        (unsigned)M5.getBatteryVoltage());
    LOG("boot", "M5.begin done");

    M5.EPD.SetRotation(90);
    M5.TP.SetRotation(90);
    M5.EPD.Clear(true);
    canvas.createCanvas(SCREEN_W, SCREEN_H);
    canvas.fillCanvas(C_BG);
    canvas.setTextSize(2);
    barCanvas.createCanvas(OVL_STRIP_W, OVL_STRIP_H);
    LOG("boot", "portrait canvas %dx%d, psram=%d",
        SCREEN_W, SCREEN_H, (int)psramFound());

    devMx  = xSemaphoreCreateMutex();


    /* 256 KB of ring only fits in PSRAM, and it buys 1.5s of decode-ahead. */
    size_t ringBytes = RING_FRAMES * 2 * sizeof(int16_t);
    ringBuf = (int16_t *)ps_malloc(ringBytes);
    if (!ringBuf) {
        LOG("boot", "ring alloc FAILED");
        while (true) delay(1000);
    }
    memset(ringBuf, 0, ringBytes);

    unsigned long t0 = millis();
    bool sdOk = SD.begin(4, SPI, 20000000);
    if (!sdOk) {
        statusMsg = "SD card error";
        LOG("boot", "SD.begin FAILED");
    } else {
        LOG("boot", "SD.begin ok (%lums)", millis() - t0);
        t0 = millis();
        scanFiles("/");
        std::sort(tracks.begin(), tracks.end());
        updateSearch(); // Populate the initial unfiltered track list
        LOG("boot", "%u tracks (%lums)",
            (unsigned)tracks.size(), millis() - t0);
    }

    audioOut = new RingOutput();

    a2dp.set_ssid_callback(on_ssid);
    a2dp.set_on_connection_state_changed(on_conn_state);
    
    // Bind our custom remote control interceptor to the library
    a2dp.set_avrc_passthru_command_callback(avrc_callback);

    /* One-off cleanup on request; the marker keeps it from repeating. */
    bool wipeNow = sdOk && !SD.exists(WIPE_MARKER_PATH);
    if (wipeNow) {
        if (!SD.exists("/config")) SD.mkdir("/config");
        SD.remove(KNOWN_DEVICE_PATH);
        SD.remove(SAVED_DEVICE_PATH);
        File m = SD.open(WIPE_MARKER_PATH, FILE_WRITE);
        if (m) m.close();
        LOG("saved", "wiped every remembered device");
    }

    if (sdOk) loadKnownDevices();
    bool haveSaved = !knownDevices.empty();
    if (haveSaved) {
        targetName = knownDevices[0].name;
        memcpy(targetMac, knownDevices[0].addr, ESP_BD_ADDR_LEN);
        hasTargetMac = true;
        statusMsg = "Reconnecting " + targetName + "...";
        btMode = BT_CONNECTING;
    } else {
        statusMsg = tracks.empty() ? "No MP3s on SD"
                                   : String("Tap BT to pair device");
    }

    /* Always up, so a scan starts at once and bonds can always be cleared. */
    beginA2dpIfNeeded(haveSaved);
    if (wipeNow) removeAllBonds();

    needFullRefresh = true;
    renderPlayer();
    LOG("boot", "setup complete");
}

void beginA2dpIfNeeded(bool autoReconnect) {
    if (a2dpStarted) return;
    if (autoReconnect && hasTargetMac) {
        a2dp.set_auto_reconnect(targetMac);
    } else {
        a2dp.set_auto_reconnect(false);
    }
    a2dp.start("__none__", get_sound_data);
    a2dp.set_volume(a2dpVolume);
    a2dpStarted = true;
    LOG("a2dp", "started (autoReconnect=%d)", (int)autoReconnect);
}

void setVolume(int v) {
    if (v < 0) v = 0;
    if (v > 127) v = 127;
    if (v == a2dpVolume) return;
    a2dpVolume = v;
    if (a2dpStarted) a2dp.set_volume(a2dpVolume);
    LOG("a2dp", "volume=%d", a2dpVolume);
    if (overlayShown && screen == S_PLAYER) pushBarStrip(true);
    else                                    uiDirty = true;
}

// ==================== Search Engine ====================
void updateSearch() {
    displayTracks.clear();
    String query = searchQuery;
    query.toLowerCase();
    
    for (int i = 0; i < (int)tracks.size(); i++) {
        if (query.length() == 0) {
            displayTracks.push_back(i);
        } else {
            // Extract the filename without the path hierarchy for clean searching
            String name = asciiOnly(tracks[i].substring(tracks[i].lastIndexOf('/') + 1));
            name.toLowerCase();
            if (name.indexOf(query) >= 0) {
                displayTracks.push_back(i);
            }
        }
    }
}

// ==================== Main loop ====================
void loop() {
    M5.TP.update();
    M5.update();  // side button / scroll wheel

    // Act on the media keys queued by the AVRCP callback
    if (avrcTogglePlayPending) {
        avrcTogglePlayPending = false;
        LOG("avrc", "remote requested PLAY/PAUSE toggle");
        pauseOrResume();
    }
    if (avrcStopPending) {
        avrcStopPending = false;
        LOG("avrc", "remote requested STOP");
        stopTrack();
        statusMsg = "Stopped";
        uiDirty = true;
    }
    if (avrcNextPending) {
        avrcNextPending = false;
        LOG("avrc", "remote requested NEXT Track");
        nextTrack();
    }
    if (avrcPrevPending) {
        avrcPrevPending = false;
        LOG("avrc", "remote requested PREV Track");
        prevTrack();
    }
    if (avrcVolumeSteps != 0) {
        int steps = avrcVolumeSteps;
        avrcVolumeSteps = 0;
        LOG("avrc", "remote requested VOLUME %+d", steps);
        setVolume(a2dpVolume + steps * VOLUME_STEP);
    }

    /*
     * Side wheel. Its job follows the screen: it scrolls the device list, it
     * rides the volume while the overlay is up, and it skips tracks otherwise.
     */
    if (M5.BtnP.wasPressed()) {
        if (screen == S_PLAYER) setOverlay(!overlayShown);
    }
    if (M5.BtnL.wasPressed()) {
        if (screen == S_DEVICES)     { devScroll--; uiDirty = true; }
        else if (overlayShown)       setVolume(a2dpVolume + VOLUME_STEP);
        else                         prevTrack();
    }
    if (M5.BtnR.wasPressed()) {
        if (screen == S_DEVICES)     { devScroll++; uiDirty = true; }
        else if (overlayShown)       setVolume(a2dpVolume - VOLUME_STEP);
        else                         nextTrack();
    }

    if (connEventPending) {
        connEventPending = false;
        handleConnEvent(connEventState);
    }

    static unsigned long lastHealth = 0;
    if (millis() - lastHealth > 10000) {
        lastHealth = millis();
        LOG("health", "heap=%u min=%u psram=%u batt=%umV bt=%d play=%d ring=%u",
            (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
            (unsigned)ESP.getFreePsram(), (unsigned)M5.getBatteryVoltage(),
            (int)btConnected, (int)playState, (unsigned)ringUsed());
    }

    // Tick so the "Scanning... Ns" label and the position bar keep up.
    static unsigned long lastTick = 0;
    if (millis() - lastTick > 1000) {
        if (screen == S_DEVICES) uiDirty = true;
        if (screen == S_PLAYER && overlayShown && dragTarget == DRAG_NONE &&
            playState == P_PLAYING) {
            pushBarStrip(false);
        }
        lastTick = millis();
    }

    if (uiDirty) {
        if (screen == S_PLAYER) renderPlayer();
        else if (screen == S_DEVICES) renderDevices();
        else if (screen == S_SEARCH) renderSearch();
        uiDirty = false;
    }

    if (autoPlayPending && btConnected && !displayTracks.empty()) {
        autoPlayPending = false;
        // Start from top of search results if filtered
        int startTrack = displayTracks.empty() ? trackIdx : displayTracks[0];
        LOG("main", "autoPlay → playTrack(%d)", startTrack);
        playTrack(startTrack);
    }

    if (playState == P_PLAYING && mp3) {
        // Aggressively fill the audio buffer to build a safety net against stuttering
        // This decodes up to ~0.65 seconds of audio in a single pass if the buffer is low
        int max_decode = 25; 
        while (mp3->isRunning() && ringUsed() < RING_FRAMES * 3 / 4 && max_decode-- > 0) {
            mp3->loop();
        }
        
        if (!mp3->isRunning()) {
            LOG("play", "decoder done, next");
            stopTrack();
            nextTrack();
        }

        static unsigned long lastLog = 0;
        static uint32_t lastFrames = 0;
        if (millis() - lastLog > 2000) {
            uint32_t df = a2dpFramesServed - lastFrames;
            LOG("play", "ring=%5u | +frames/2s=%u",
                (unsigned)ringUsed(), df);
            lastFrames = a2dpFramesServed;
            lastLog = millis();
        }
    }

    // State-based (Edge-Triggered) Touch Detection
    bool isTouched = !M5.TP.isFingerUp();
    static bool wasTouched = false;
    static unsigned long lastEdgeMs = 0;

    // 20ms hardware noise filter (reduced drastically for much faster tap/typing response)
    if (isTouched != wasTouched && millis() - lastEdgeMs > 20) {
        wasTouched = isTouched;
        lastEdgeMs = millis();
        
        // Trigger action the exact moment the finger goes DOWN
        if (isTouched) { 
            tp_finger_t f = M5.TP.readFinger(0);
            if (f.x > 0 || f.y > 0) {
                LOG("tap", "DOWN x=%d y=%d screen=%d", f.x, f.y, (int)screen);
                if (screen == S_PLAYER) handleTouchPlayer(f.x, f.y);
                else if (screen == S_DEVICES) handleTouchDevices(f.x, f.y);
                else if (screen == S_SEARCH) handleTouchSearch(f.x, f.y);
            }
        } else if (dragTarget != DRAG_NONE) {
            // The seek only lands on release, so one drag costs one restart
            bool seeking = (dragTarget == DRAG_SEEK && playState != P_STOPPED);
            dragTarget = DRAG_NONE;
            if (seeking) {
                LOG("seek", "to %d%%", (int)(seekFrac * 100));
                playTrackAt(trackIdx, seekFrac);
                uiDirty = true;
            }
        }
    } else if (isTouched && dragTarget != DRAG_NONE) {
        // A strip push already paces this, so only skip work the eye cannot see
        static int lastFill = -1;
        tp_finger_t f = M5.TP.readFinger(0);
        int fill = (int)(barFrac(f.x) * OVL_BAR_W);
        if (fill != lastFill) {
            lastFill = fill;
            if (dragTarget == DRAG_VOL) setVolume((int)(barFrac(f.x) * 127.0f));
            else { seekFrac = barFrac(f.x); pushBarStrip(false); }
        }
    }

    delay(playState == P_PLAYING ? 2 : 10);
}

// ==================== Touch handling ====================
void handleTouchPlayer(int x, int y) {
    if (overlayShown && y >= OVL_Y && y <= OVL_Y + OVL_H) {
        if (onBar(y, OVL_VOL_Y)) {
            dragTarget = DRAG_VOL;
            setVolume((int)(barFrac(x) * 127.0f));
            pushBarStrip(true);
        } else if (onBar(y, OVL_POS_Y)) {
            dragTarget = DRAG_SEEK;
            seekFrac = barFrac(x);
            pushBarStrip(false);
        }
        return;
    }

    // Header search button
    if (y > 20 && y < 80 && x > SCREEN_W - 140 && x < SCREEN_W - 60) {
        screen = S_SEARCH;
        needFullRefresh = true;
        uiDirty = true;
        return;
    }

    // Track list rows
    int listBot = TRACKS_Y + TRACK_ROWS * TRACK_ROW_H;
    if (!overlayShown && y >= TRACKS_Y && y < listBot) {
        int row = (y - TRACKS_Y) / TRACK_ROW_H;
        
        // Find current track position inside filtered list
        int displayIdx = 0;
        for (int i = 0; i < (int)displayTracks.size(); i++) {
            if (displayTracks[i] == trackIdx) { displayIdx = i; break; }
        }

        int start = displayIdx - (TRACK_ROWS / 2);
        if (start > (int)displayTracks.size() - TRACK_ROWS) start = displayTracks.size() - TRACK_ROWS;
        if (start < 0) start = 0;
        
        int selectionIndex = start + row;
        if (selectionIndex >= 0 && selectionIndex < (int)displayTracks.size()) {
            playTrack(displayTracks[selectionIndex]);
            return;
        }
    }

    // Controls: three circular buttons
    int prevCx = 110, playCx = 270, nextCx = 430;
    int dy = y - CTRL_CY;
    if (abs(dy) < 70) {
        int dx = x - prevCx;
        if (dx * dx + dy * dy < 60 * 60) { prevTrack(); return; }
        dx = x - playCx;
        if (dx * dx + dy * dy < 75 * 75) { pauseOrResume(); return; }
        dx = x - nextCx;
        if (dx * dx + dy * dy < 60 * 60) { nextTrack(); return; }
    }

    // BT bar at bottom
    if (y >= BT_BAR_Y && y <= BT_BAR_Y + BT_BAR_H) {
        enterDevicesScreen();
        return;
    }
}

void handleTouchSearch(int x, int y) {
    for (auto& k : keys) {
        if (x >= k.x && x <= k.x + k.w && y >= k.y && y <= k.y + k.h) {
            String l = k.label;
            if (l == "DEL") {
                if (searchQuery.length() > 0) searchQuery.remove(searchQuery.length() - 1);
            } else if (l == "CLR") {
                searchQuery = "";
            } else if (l == "DONE") {
                updateSearch();
                screen = S_PLAYER;
                needFullRefresh = true;
                uiDirty = true;
                return; 
            } else {
                searchQuery += l;
            }
            uiDirty = true;
            return;
        }
    }
}

void handleTouchDevices(int x, int y) {
    if (y >= BTN_BAND_TOP && y <= BTN_BAND_BOT) {
        bool armed = clearArmedMs && millis() - clearArmedMs < 5000;
        if (x < BTN_SCAN_X) {
            clearArmedMs = 0;
            if (btMode == BT_SCANNING) {
                btMode = BT_IDLE;
                esp_bt_gap_cancel_discovery();
            }
            enterPlayerScreen();
            return;
        }
        if (x < BTN_CLEAR_X) {
            clearArmedMs = 0;
            startScan();
        } else if (armed) {
            clearArmedMs = 0;
            forgetAllDevices();
        } else {
            clearArmedMs = millis(); // one tap arms, the next one wipes
        }
        needFullRefresh = true;
        renderDevices();
        return;
    }

    for (auto &r : devRows) {
        if (r.kind == ROW_LBL_SAVED || r.kind == ROW_LBL_FOUND) continue;
        if (y < r.y || y >= r.y + DEV_ROW_H) continue;

        if (r.kind == ROW_SAVED) {
            if (x >= SCREEN_W - 110) {
                forgetDevice(r.idx);
                needFullRefresh = true;
                renderDevices();
            } else {
                connectTo(knownDevices[r.idx].name, knownDevices[r.idx].addr);
            }
            return;
        }

        String pickedName;
        esp_bd_addr_t pickedAddr;
        bool got = false;
        if (xSemaphoreTake(devMx, portMAX_DELAY) == pdTRUE) {
            if (r.idx < (int)devices.size()) {
                pickedName = devices[r.idx].name;
                memcpy(pickedAddr, devices[r.idx].addr, ESP_BD_ADDR_LEN);
                got = true;
            }
            xSemaphoreGive(devMx);
        }
        if (got) connectTo(pickedName, pickedAddr);
        return;
    }
}

// ==================== Overlay ====================

/* A full refresh follows every toggle: DU4 leaves the panel under it ghosted. */
static void setOverlay(bool on) {
    if (overlayShown == on) return;
    overlayShown = on;
    dragTarget = DRAG_NONE;
    needFullRefresh = true;
    uiDirty = true;
}

void hideOverlay() { setOverlay(false); }

static float barFrac(int x) {
    float f = (float)(x - OVL_BAR_X) / (float)OVL_BAR_W;
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

static bool onBar(int y, int barY) {
    return y >= barY - OVL_GRAB && y <= barY + OVL_BAR_H + OVL_GRAB;
}

// ==================== Screen transitions ====================

/* Opening the list must never drop the link: a stray tap once killed playback. */
void enterDevicesScreen() {
    screen = S_DEVICES;
    devScroll = 0;
    needFullRefresh = true;
    renderDevices();
}

void enterPlayerScreen() {
    screen = S_PLAYER;
    overlayShown = false;
    dragTarget = DRAG_NONE;
    needFullRefresh = true;
    renderPlayer();
}

void startScan() {
    if (xSemaphoreTake(devMx, portMAX_DELAY) == pdTRUE) {
        devices.clear();
        xSemaphoreGive(devMx);
    }
    devScroll = 0;
    btMode = BT_SCANNING;
    scanStartMs = millis();
    if (!a2dpStarted) {
        beginA2dpIfNeeded(false);
    } else {
        // Cancel any in-flight auto-reconnect to old MAC, drop active connection.
        if (btConnected) { a2dp.disconnect(); btConnected = false; }
        a2dp.startDiscovery();
    }
    connectFails = 0;
    statusMsg = "Scanning for devices...";
    LOG("scan", "discovery started");
}

// ==================== Playback ====================

/*
 * Fraction of the file already read. MP3 carries no position, so the byte
 * offset stands in for it: exact for constant bit rate, close enough for the
 * rest.
 */
float trackProgress() {
    if (!src) return 0.0f;
    uint32_t size = src->getSize();
    if (size == 0) return 0.0f;
    float f = (float)src->getPos() / (float)size;
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

/* Seconds of audio per byte, measured from what the decoder produced so far. */
static float trackSecondsPerByte() {
    if (!src || !audioOut || framesDecoded < 4096) return 0.0f;
    uint32_t pos = src->getPos();
    if (pos <= trackStartPos) return 0.0f;
    float seconds = (float)framesDecoded / (float)audioOut->sourceRate;
    return seconds / (float)(pos - trackStartPos);
}

static String formatTime(float seconds) {
    if (seconds <= 0.0f || seconds > 86400.0f) return String("--:--");
    int t = (int)seconds;
    char buf[12];
    snprintf(buf, sizeof(buf), "%d:%02d", t / 60, t % 60);
    return String(buf);
}

void playTrack(int i) {
    playTrackAt(i, 0.0f);
}

void playTrackAt(int i, float frac) {
    if (i < 0 || i >= (int)tracks.size()) return;
    if (!btConnected) {
        statusMsg = "Connect a device first";
        uiDirty = true;
        return;
    }

    unsigned long tOpen = millis();
    LOG("play", ">>> playTrack(%d)", i);

    stopTrack();

    trackIdx = i;
    needFullRefresh = true; // Forces E-ink clear to prevent ghosting/text overlap
    
    String path = tracks[i];
    nowPlayingName = path.substring(path.lastIndexOf('/') + 1);
    if (nowPlayingName.endsWith(".mp3") || nowPlayingName.endsWith(".MP3")) {
        nowPlayingName.remove(nowPlayingName.length() - 4);
    }

    ringR = ringW = 0; // safe: the decoder is stopped and playState is not PLAYING

    a2dpFirstCallMs = 0;
    a2dpFirstAudioMs = 0;
    a2dpCallCount = 0;
    a2dpFramesServed = 0;
    a2dpSilenceCount = 0;

    src = new AudioFileSourceSD(path.c_str());
    mp3 = new AudioGeneratorMP3();

    /*
     * The ID3 filter only makes sense at the head of the file. Past a seek the
     * tag is already behind us, and libmad resyncs on the next frame header,
     * so the raw file feeds the decoder directly.
     */
    AudioFileSource *feed;
    if (frac > 0.0f) {
        src->seek((int32_t)(frac * (float)src->getSize()), SEEK_SET);
        feed = src;
    } else {
        id3 = new AudioFileSourceID3(src);
        feed = id3;
    }
    framesDecoded = 0;
    trackStartPos = src->getPos();

    if (!mp3->begin(feed, audioOut)) {
        LOG("play", "mp3->begin failed");
        statusMsg = "Decode error";
        stopTrack();
        uiDirty = true;
        return;
    }

    unsigned long t = millis();
    int guard = 0;
    while (ringUsed() < PREBUFFER_FRAMES && mp3->isRunning()) {
        mp3->loop();
        if (++guard > 4000) break;
    }
    LOG("play", "pre-buffer ring=%u iters=%d (%lums)",
        (unsigned)ringUsed(), guard, millis() - t);

    if (!mp3->isRunning() || ringUsed() < 1000) {
        LOG("play", "decoder died in pre-buffer");
        statusMsg = "Decoder failed";
        stopTrack();
        uiDirty = true;
        return;
    }

    statusMsg = "Playing";
    playState = P_PLAYING;
    needFullRefresh = true; // Wipes out the ghosted text
    uiDirty = true;
    LOG("play", "<< total %lums", millis() - tOpen);
}

void stopTrack() {
    playState = P_STOPPED;
    framesDecoded = 0;
    if (mp3) { mp3->stop(); delete mp3; mp3 = nullptr; }
    if (id3) { delete id3; id3 = nullptr; }
    if (src) { delete src; src = nullptr; }
}

void pauseOrResume() {
    if (playState == P_PLAYING) {
        playState = P_PAUSED;
        statusMsg = "Paused";
    } else if (playState == P_PAUSED && mp3) {
        playState = P_PLAYING;
        statusMsg = "Playing";
    } else if (!displayTracks.empty()) {
        // If queue is stopped, default to playing the first item in the filtered view
        playTrack(displayTracks[0]);
        return;
    }
    uiDirty = true;
}

void nextTrack() {
    int size = displayTracks.size();
    if (size == 0) return;
    
    // Cycle explicitly through the actively filtered list
    int currentDisplayIdx = -1;
    for (int i = 0; i < size; i++) {
        if (displayTracks[i] == trackIdx) { currentDisplayIdx = i; break; }
    }
    if (currentDisplayIdx == -1) currentDisplayIdx = 0;
    
    playTrack(displayTracks[(currentDisplayIdx + 1) % size]);
}

void prevTrack() {
    int size = displayTracks.size();
    if (size == 0) return;
    
    // Cycle explicitly through the actively filtered list
    int currentDisplayIdx = -1;
    for (int i = 0; i < size; i++) {
        if (displayTracks[i] == trackIdx) { currentDisplayIdx = i; break; }
    }
    if (currentDisplayIdx == -1) currentDisplayIdx = 0;
    
    playTrack(displayTracks[(currentDisplayIdx - 1 + size) % size]);
}

// ==================== SD scan ====================
void scanFiles(const char *dir) {
    File root = SD.open(dir);
    if (!root || !root.isDirectory()) { if (root) root.close(); return; }
    File f = root.openNextFile();
    while (f) {
        String name = f.name();
        if (f.isDirectory()) {
            if (!name.startsWith(".") && name != "config") {
                String sub = dir;
                if (!sub.endsWith("/")) sub += "/";
                sub += name;
                scanFiles(sub.c_str());
            }
        } else if (!name.startsWith(".") && (name.endsWith(".mp3") || name.endsWith(".MP3"))) {
            String full = dir;
            if (!full.endsWith("/")) full += "/";
            full += name;
            tracks.push_back(full);
        }
        f = root.openNextFile();
    }
    root.close();
}

// ==================== Rendering ====================
static void pushCanvasNow() {
    m5epd_update_mode_t mode = UPDATE_MODE_DU4;
    if (needFullRefresh) {
        mode = UPDATE_MODE_GC16;
        needFullRefresh = false;
    }
    canvas.pushCanvas(0, 0, mode);
    lastFlushMs = millis();
}

static void drawHeader(const char *title) {
    canvas.setTextColor(C_INK);
    canvas.setTextSize(4); 
    
    // Pseudo-bold effect
    canvas.drawString(title, 30, 32);
    canvas.drawString(title, 31, 32);
    canvas.drawString(title, 30, 33);
    canvas.drawString(title, 31, 33);
    
    if (screen == S_PLAYER) {
        // Search button magnifying glass right next to logo
        iconSearch(SCREEN_W - 110, 48, 28, C_INK);
    }
    
    // subtle logo glyph on the right
    iconNote(SCREEN_W - 50, 50, 34, C_INK);
    canvas.drawLine(24, HEAD_H, SCREEN_W - 24, HEAD_H, C_DIV);
}

/*
 * Greedy word wrap against the real font metrics. A word wider than the line
 * is cut on a UTF-8 boundary, and leftover text ends the last line with an
 * ellipsis.
 */
/*
 * The built-in font holds ASCII only. A multi-byte character sends its glyph
 * lookup past the end of the table, which hangs the draw and resets the board,
 * so every string built from a filename or a device name is filtered first.
 */
static String asciiOnly(const String &t) {
    String out;
    out.reserve(t.length());
    bool marked = false;
    for (unsigned int i = 0; i < t.length(); i++) {
        uint8_t c = (uint8_t)t[i];
        if (c >= 0x20 && c < 0x7F) {
            out += (char)c;
            marked = false;
        } else if (!marked) {
            out += '?';
            marked = true;
        }
    }
    return out;
}

/* The built-in font is a fixed 6 pixels wide per size step. */
static inline int textW(const String &t, int size) {
    return t.length() * 6 * size;
}

/*
 * Greedy word wrap by character count. The canvas cannot measure text: its
 * textWidth() returns 0 whenever the font renderer is active, which made a
 * measuring wrapper useless and unsafe.
 */
static std::vector<String> wrapText(const String &text, int maxChars, int maxLines) {
    std::vector<String> lines;
    int n = text.length();
    int start = 0;

    while (start < n && (int)lines.size() < maxLines) {
        while (start < n && text[start] == ' ') start++;
        if (start >= n) break;

        if (n - start <= maxChars) {
            lines.push_back(text.substring(start));
            start = n;
            break;
        }

        int end = start + maxChars;
        int space = text.lastIndexOf(' ', end);
        if (space > start) {
            end = space;
        } else {
            while (end > start + 1 && (text[end] & 0xC0) == 0x80) end--;
        }
        lines.push_back(text.substring(start, end));
        start = end;
    }

    if (start < n && !lines.empty()) {
        String last = lines.back();
        while ((int)last.length() > maxChars - 3) {
            int k = last.length() - 1;
            while (k > 0 && (last[k] & 0xC0) == 0x80) k--;
            last = last.substring(0, k);
        }
        lines.back() = last + "...";
    }
    return lines;
}

static void drawNowPlayingCard() {
    // Card outline
    canvas.fillRoundRect(CARD_X, CARD_Y, CARD_W, CARD_H, 16, C_BG);
    canvas.drawRoundRect(CARD_X, CARD_Y, CARD_W, CARD_H, 16, C_CARD_BORDER);

    // Small label
    canvas.setTextColor(C_MUTED);
    canvas.setTextSize(2);
    canvas.drawString("NOW PLAYING", CARD_X + 22, CARD_Y + 18);

    // Title, wrapped over as many lines as the card has room for
    canvas.setTextColor(C_INK);
    canvas.setTextSize(3);
    String t = asciiOnly(nowPlayingName.length() ? nowPlayingName : String("No track"));
    int lineH = 30;
    int titleTop = CARD_Y + 54;
    int titleBot = CARD_Y + CARD_H - 54;
    int ty = titleTop;
    for (auto &line : wrapText(t, (CARD_W - 44) / 18, (titleBot - titleTop) / lineH)) {
        canvas.drawString(line, CARD_X + 22, ty);
        ty += lineH;
    }

    // State line with small icon
    canvas.setTextSize(2);
    canvas.setTextColor(C_MUTED);
    int sy = CARD_Y + CARD_H - 44;
    if (playState == P_PLAYING) {
        iconPlay(CARD_X + 34, sy + 10, 18, C_INK);
        canvas.setTextColor(C_INK);
        canvas.drawString("Playing", CARD_X + 58, sy + 2);
    } else if (playState == P_PAUSED) {
        iconPause(CARD_X + 34, sy + 10, 18, C_INK);
        canvas.setTextColor(C_INK);
        canvas.drawString("Paused", CARD_X + 58, sy + 2);
    } else {
        canvas.drawString(asciiOnly(statusMsg), CARD_X + 22, sy + 2);
    }
}

static void drawTrackList() {
    canvas.setTextColor(C_MUTED);
    canvas.setTextSize(2);
    canvas.drawString("TRACKS", 30, TRACKS_LABEL_Y);
    canvas.drawLine(30, TRACKS_LABEL_Y + 26, 130, TRACKS_LABEL_Y + 26, C_INK);

    if (displayTracks.empty()) {
        canvas.setTextColor(C_MUTED);
        canvas.setTextSize(2);
        canvas.drawString(searchQuery.length() > 0 ? "No matches found" : "No MP3s on SD card", 30, TRACKS_Y + 20);
        return;
    }

    // Find where the currently playing track is in our filtered view
    int displayIdx = 0;
    for (int i = 0; i < (int)displayTracks.size(); i++) {
        if (displayTracks[i] == trackIdx) { displayIdx = i; break; }
    }

    // Center the active track in the view instead of pinning it to the bottom
    int start = displayIdx - (TRACK_ROWS / 2);
    if (start > (int)displayTracks.size() - TRACK_ROWS) start = displayTracks.size() - TRACK_ROWS;
    if (start < 0) start = 0;

    for (int row = 0; row < TRACK_ROWS && (start + row) < (int)displayTracks.size(); row++) {
        int i = displayTracks[start + row];
        String name = tracks[i].substring(tracks[i].lastIndexOf('/') + 1);
        if (name.endsWith(".mp3") || name.endsWith(".MP3")) {
            name.remove(name.length() - 4);
        }
        int y = TRACKS_Y + row * TRACK_ROW_H;
        bool isCur = (i == trackIdx);

        if (isCur) {
            canvas.fillRoundRect(22, y + 2, SCREEN_W - 44, TRACK_ROW_H - 8, 14, C_HL_FILL);
            iconNote(52, y + TRACK_ROW_H / 2 - 2, 26, C_HL_INK);
            canvas.setTextColor(C_HL_INK);
        } else {
            iconNote(52, y + TRACK_ROW_H / 2 - 2, 22, C_MUTED);
            canvas.setTextColor(C_INK);
        }
        canvas.setTextSize(3);
        
        // UTF-8 Safe Truncation
        int maxChars = (SCREEN_W - 120) / 18;
        if ((int)name.length() > maxChars) {
            int cut = maxChars - 3;
            while (cut > 0 && (name[cut] & 0xC0) == 0x80) {
                cut--;
            }
            name = name.substring(0, cut) + "...";
        }
        
        canvas.drawString(name, 86, y + TRACK_ROW_H / 2 - 18);
    }
}

static void drawControls() {
    int prevCx = 110, playCx = 270, nextCx = 430;
    int cy = CTRL_CY;

    // Prev
    canvas.drawCircle(prevCx, cy, 50, C_INK);
    canvas.drawCircle(prevCx, cy, 49, C_INK);
    iconPrev(prevCx, cy, 40, C_INK);

    // Play/Pause: filled circle for emphasis
    canvas.fillCircle(playCx, cy, 62, C_INK);
    if (playState == P_PLAYING) iconPause(playCx, cy, 46, C_BG);
    else                        iconPlay (playCx, cy, 46, C_BG);

    // Next
    canvas.drawCircle(nextCx, cy, 50, C_INK);
    canvas.drawCircle(nextCx, cy, 49, C_INK);
    iconNext(nextCx, cy, 40, C_INK);
}

static void drawBtBar() {
    canvas.drawLine(24, BT_BAR_Y, SCREEN_W - 24, BT_BAR_Y, C_DIV);
    int cy = BT_BAR_Y + BT_BAR_H / 2;
    iconBt(50, cy, 34, C_INK);

    canvas.setTextSize(2);
    String left;
    if (btConnected)                  left = targetName.length() ? targetName : String("Connected");
    else if (btMode == BT_CONNECTING) left = "Connecting " + targetName + "...";
    else if (btMode == BT_SCANNING)   left = "Scanning...";
    else                              left = "Not connected, tap to pair";
    canvas.setTextColor(C_INK);

    // Volume readout, the only feedback for the device's own volume keys
    String vol = "VOL " + String(a2dpVolume * 100 / 127) + "%";
    int volW = vol.length() * 12;
    int volX = SCREEN_W - 30 - volW;
    if ((int)left.length() * 12 > volX - 90) left = left.substring(0, (volX - 90) / 12);
    canvas.drawString(asciiOnly(left), 80, cy - 10);
    canvas.setTextColor(C_MUTED);
    canvas.drawString(vol, volX, cy - 10);
}

static void drawBarInto(M5EPD_Canvas &cv, int bx, int by, float frac,
                        const char *label, const String &value) {
    cv.setTextSize(2);
    cv.setTextColor(C_MUTED);
    cv.drawString(label, bx, by - 30);
    cv.setTextColor(C_INK);
    cv.drawString(value, bx + OVL_BAR_W - textW(value, 2), by - 30);

    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    int fill = (int)(frac * OVL_BAR_W);
    cv.drawRoundRect(bx, by, OVL_BAR_W, OVL_BAR_H, 10, C_INK);
    if (fill > 4) cv.fillRoundRect(bx, by, fill, OVL_BAR_H, 10, C_INK);
    cv.fillCircle(bx + fill, by + OVL_BAR_H / 2, 19, C_INK);
    cv.fillCircle(bx + fill, by + OVL_BAR_H / 2, 11, C_BG);
}

static float volumeFrac() { return (float)a2dpVolume / 127.0f; }
static String volumeText() { return String(a2dpVolume * 100 / 127) + "%"; }
static float positionFrac() {
    return (dragTarget == DRAG_SEEK) ? seekFrac : trackProgress();
}

static String positionText(float frac) {
    float spb = trackSecondsPerByte();
    if (spb <= 0.0f || !src) return String("--:--");
    float total = spb * (float)src->getSize();
    return formatTime(frac * total) + " / " + formatTime(total);
}

/*
 * Repaint one bar and push only its strip. A whole-canvas push moves 253 KB
 * over SPI before the panel even starts, which is what made a drag lag behind
 * the finger.
 */
static void pushBarStrip(bool volumeBar) {
    if (!overlayShown || screen != S_PLAYER) return;
    int barY = volumeBar ? OVL_VOL_Y : OVL_POS_Y;
    int sy = (barY - OVL_STRIP_LIFT) & ~3;
    float frac = volumeBar ? volumeFrac() : positionFrac();

    barCanvas.fillCanvas(C_BG);
    drawBarInto(barCanvas, OVL_BAR_X - OVL_STRIP_X, barY - sy, frac,
                volumeBar ? "VOLUME" : "POSITION",
                volumeBar ? volumeText() : positionText(frac));
    barCanvas.pushCanvas(OVL_STRIP_X, sy, UPDATE_MODE_DU4);
    lastFlushMs = millis();
}

static void drawOverlay() {
    canvas.fillRoundRect(OVL_X, OVL_Y, OVL_W, OVL_H, 18, C_BG);
    canvas.drawRoundRect(OVL_X, OVL_Y, OVL_W, OVL_H, 18, C_INK);
    canvas.drawRoundRect(OVL_X + 1, OVL_Y + 1, OVL_W - 2, OVL_H - 2, 18, C_INK);

    drawBarInto(canvas, OVL_BAR_X, OVL_VOL_Y, volumeFrac(), "VOLUME", volumeText());
    float frac = positionFrac();
    drawBarInto(canvas, OVL_BAR_X, OVL_POS_Y, frac, "POSITION", positionText(frac));

    canvas.setTextSize(2);
    canvas.setTextColor(C_MUTED);
    canvas.drawString("Drag a bar. Wheel sets volume. Push closes.",
                      OVL_BAR_X, OVL_Y + OVL_H - 34);
}

void renderPlayer() {
    canvas.fillCanvas(C_BG);
    drawHeader("M5Player");
    drawNowPlayingCard();
    if (overlayShown) drawOverlay();
    else              drawTrackList();
    drawControls();
    drawBtBar();
    pushCanvasNow();
}

void renderSearch() {
    canvas.fillCanvas(C_BG);
    drawHeader("Search");

    // Dynamic Text Box
    canvas.drawRoundRect(20, 150, SCREEN_W - 40, 70, 8, C_INK);
    canvas.setTextSize(3);
    canvas.setTextColor(C_INK);
    
    // Typewriter cursor and truncation for long strings
    String disp = searchQuery + "_";
    if (disp.length() > 25) disp = "..." + disp.substring(disp.length() - 25);
    canvas.drawString(disp, 35, 172);

    initKeyboard();
    for(auto& k : keys) {
        String label = String(k.label);
        if (label == "DEL" || label == "CLR" || label == "DONE") {
            canvas.fillRoundRect(k.x, k.y, k.w, k.h, 6, C_INK);
            canvas.setTextColor(C_BG);
        } else {
            canvas.drawRoundRect(k.x, k.y, k.w, k.h, 6, C_INK);
            canvas.setTextColor(C_INK);
        }
        
        // Center text on keys
        int offX = k.w / 2 - (label.length() * 18) / 2;
        if (label == " ") offX = k.w / 2 - 40;
        
        canvas.drawString(label == " " ? "SPACE" : k.label, k.x + offX, k.y + 22);
    }
    
    // Since typing uses DU4 for speed, we don't clear the screen fully while typing
    pushCanvasNow();
}

static int devRowHeight(DevRowKind k) {
    return (k == ROW_LBL_SAVED || k == ROW_LBL_FOUND) ? DEV_LABEL_H : DEV_ROW_H;
}

/*
 * Lay out the window of the device list that fits on screen. The saved and
 * found sections scroll as one, so the side wheel reaches every entry.
 */
static void buildDevRows() {
    std::vector<DevRow> all;
    if (!knownDevices.empty()) {
        all.push_back({ROW_LBL_SAVED, 0, 0});
        for (int i = 0; i < (int)knownDevices.size(); i++)
            all.push_back({ROW_SAVED, i, 0});
    }
    all.push_back({ROW_LBL_FOUND, 0, 0});
    if (xSemaphoreTake(devMx, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < (int)devices.size(); i++)
            all.push_back({ROW_FOUND, i, 0});
        xSemaphoreGive(devMx);
    }

    devTotalItems = all.size();
    int maxScroll = devTotalItems;
    for (int h = 0; maxScroll > 0; maxScroll--) {
        h += devRowHeight(all[maxScroll - 1].kind);
        if (DEV_LIST_TOP + h > DEV_LIST_BOT) break;
    }
    if (devScroll > maxScroll) devScroll = maxScroll;
    if (devScroll < 0) devScroll = 0;
    devFirstShown = devScroll;

    devRows.clear();
    int y = DEV_LIST_TOP;
    for (int i = devScroll; i < devTotalItems; i++) {
        int h = devRowHeight(all[i].kind);
        if (y + h > DEV_LIST_BOT) break;
        all[i].y = y;
        devRows.push_back(all[i]);
        y += h;
    }
}

static void drawDevRow(const DevRow &r) {
    if (r.kind == ROW_LBL_SAVED || r.kind == ROW_LBL_FOUND) {
        canvas.setTextSize(2);
        canvas.setTextColor(C_MUTED);
        const char *t = (r.kind == ROW_LBL_SAVED) ? "SAVED"
                      : (btMode == BT_SCANNING ? "SCANNING..." : "FOUND");
        canvas.drawString(t, 30, r.y + 8);
        return;
    }

    bool saved = (r.kind == ROW_SAVED);
    String nm;
    if (saved) {
        nm = knownDevices[r.idx].name;
    } else if (xSemaphoreTake(devMx, portMAX_DELAY) == pdTRUE) {
        if (r.idx < (int)devices.size()) nm = devices[r.idx].name;
        xSemaphoreGive(devMx);
    }

    nm = asciiOnly(nm);
    canvas.drawRoundRect(24, r.y, SCREEN_W - 48, DEV_ROW_H - 10, 12, C_CARD_BORDER);
    iconBt(56, r.y + 30, 26, C_INK);
    canvas.setTextColor(C_INK);
    canvas.setTextSize(3);
    int maxChars = (SCREEN_W - (saved ? 210 : 140)) / 18;
    if ((int)nm.length() > maxChars) nm = nm.substring(0, maxChars - 1) + "...";
    canvas.drawString(nm, 100, r.y + 16);

    if (saved) {
        canvas.drawRoundRect(SCREEN_W - 104, r.y + 12, 72, 36, 8, C_INK);
        canvas.setTextSize(2);
        canvas.drawString("FORGET", SCREEN_W - 98, r.y + 22);
    }
}

void renderDevices() {
    canvas.fillCanvas(C_BG);
    canvas.setTextColor(C_INK);
    canvas.setTextSize(3);
    canvas.drawString("Devices", 30, 38);
    canvas.drawLine(24, 90, SCREEN_W - 24, 90, C_DIV);

    buildDevRows();

    /* DU4 leaves the old rows ghosted, so any layout change needs a GC16 wipe. */
    static int lastScroll = -1, lastKnown = -1, lastFound = -1, lastMode = -1;
    int nowFound = (int)devices.size();
    if (devScroll != lastScroll || (int)knownDevices.size() != lastKnown ||
        nowFound != lastFound || (int)btMode != lastMode) {
        needFullRefresh = true;
        lastScroll = devScroll;
        lastKnown = knownDevices.size();
        lastFound = nowFound;
        lastMode = btMode;
    }

    bool foundShown = false;
    for (auto &r : devRows) {
        if (r.kind == ROW_FOUND) foundShown = true;
        drawDevRow(r);
    }

    if (!foundShown && nowFound == 0 && !devRows.empty()) {
        const DevRow &last = devRows.back();
        int y = last.y + devRowHeight(last.kind) + 8;
        canvas.setTextColor(C_MUTED);
        canvas.setTextSize(2);
        if (btMode == BT_SCANNING) {
            canvas.drawString("Put the headphones or speaker", 30, y);
            canvas.drawString("into pairing mode.", 30, y + 30);
            unsigned long secs = (millis() - scanStartMs) / 1000;
            canvas.drawString("Elapsed: " + String(secs) + "s", 30, y + 70);
        } else {
            canvas.drawString("Tap a saved device to reconnect.", 30, y);
            canvas.drawString("Tap SCAN to find a new one.", 30, y + 30);
            canvas.drawString("SCAN ends the current link.", 30, y + 70);
        }
    }

    if (devTotalItems > (int)devRows.size()) {
        canvas.setTextColor(C_MUTED);
        canvas.setTextSize(2);
        String pos = String(devFirstShown + 1) + "-" +
                     String(devFirstShown + (int)devRows.size()) + " of " +
                     String(devTotalItems) + "  (side wheel scrolls)";
        canvas.drawString(pos, 30, DEV_LIST_BOT + 4);
    }

    bool armed = clearArmedMs && millis() - clearArmedMs < 5000;
    const char *labels[3] = {"BACK", "SCAN", armed ? "SURE?" : "CLEAR"};
    const int xs[3] = {BTN_BACK_X, BTN_SCAN_X, BTN_CLEAR_X};
    canvas.setTextSize(3);
    for (int i = 0; i < 3; i++) {
        canvas.fillRoundRect(xs[i], BTN_Y, BTN_W, BTN_H, 16, C_INK);
        canvas.setTextColor(C_BG);
        canvas.drawString(labels[i],
                          xs[i] + (BTN_W - textW(labels[i], 3)) / 2,
                          BTN_Y + 16);
    }

    pushCanvasNow();
}
