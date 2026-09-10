# M5Player

Plays MP3 files from the SD card to any Bluetooth headphones or speaker.
Runs on the M5Paper.

## Supported

- Any Bluetooth audio sink (headphones, earbuds, headset, speaker) over A2DP.
- MP3 files on the SD card, found in every folder.
- Remote media keys from the headset: play, pause, stop, next, previous, volume.
- Remembers up to 8 devices in `/config/devices.txt` and reconnects on power-up.

## Use

1. Copy MP3 files to the SD card.
2. Power on. It reconnects to the last device by itself.
3. To pair: tap the BT bar at the bottom, tap `SCAN`, tap your device.
   Put the device in pairing mode first.

### Controls

| Action | How |
| --- | --- |
| Play, pause | Round button on screen |
| Next, previous track | Side arrows on screen |
| Pick a track | Tap it in the list |
| Search | Magnifier in the header |
| Volume and position | Push the side wheel, then drag a bar |
| Skip tracks | Side wheel up and down |
| Scroll device list | Side wheel up and down |

The volume and position panel stays open until you push the wheel again.
Drag the position bar and let go to seek.

### Devices screen

- Tap a saved device to reconnect. No scan needed.
- `FORGET` on a row deletes that one device.
- `CLEAR` deletes all of them. It asks once first.
- `SCAN` looks for new devices. It ends the current link.

Forget and clear also delete the Bluetooth pairing key, so pair again from both sides.

## Build with the Arduino IDE

**1. Add the M5Stack boards.** File, then Preferences. Paste this into
"Additional boards manager URLs":

```
https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json
```

**2. Install the board package.** Tools, then Board, then Boards Manager.
Search `M5Stack` and install **version 2.1.4**. Do not take 3.x, see Versions below.

**3. Install the libraries.** Tools, then Manage Libraries. Install:

- `M5EPD`
- `ESP8266Audio`
- `ESP32-A2DP` (version 1.8.8)

**4. Pick the board.** Tools, then Board, then M5Stack, then **M5Paper**.

**5. Pick the port.** Tools, then Port. On Linux it is usually `/dev/ttyACM0`.
The M5Paper must be switched on, or no port appears.

**6. Upload.** Open `M5Player.ino` and press the arrow button.

To watch the log, open Tools, then Serial Monitor, and set **115200 baud**.
It prints the reset reason, free memory and battery voltage.

## Build with the command line (optional)

Same result, no GUI.

```
arduino-cli compile --fqbn m5stack:esp32:m5stack_paper M5Player.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn m5stack:esp32:m5stack_paper M5Player.ino
```

## Versions

Build with the **`m5stack:esp32` core, version 2.1.4** (ESP-IDF 4.4.6).

- The `m5stack_paper` board exists only in the M5Stack core. `esp32:esp32:m5stack_paper`
  is not a valid FQBN.
- The headset media keys need ESP-IDF 4.0 or newer. `ESP32-A2DP` hides
  `esp_avrc_tg_init()` behind that check, so on an older core the sketch builds
  and the headset buttons quietly do nothing.

The 3.x cores move to ESP-IDF 5, a different API. This sketch is untested there.

## Licence

MIT. See `LICENSE`.
