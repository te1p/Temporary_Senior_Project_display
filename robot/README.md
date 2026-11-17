# Robot code

Everything in this folder runs **on the robots**. The laptop-side tools are in [`../laptop/`](../laptop/).

| Folder | Runs on | What it is |
|---|---|---|
| [`robot_wifi/`](robot_wifi/) | Raspberry Pi Pico W in the Freenove 4WD Mecanum car (FNK0089) | **The robot firmware.** Wi-Fi, driving, head servo, LED eyes, wireless updates |

## robot_wifi: what the firmware does

- **Wi-Fi:** joins the router named in `secrets.h`. With no `secrets.h`, or if the router isn't found within 15 s, it makes its own network `RobotDuel-<id>`. Each robot is reachable as `robot<id>.local`.
- **Driving:** mecanum mixing for forward/back, sideways, diagonals and spinning. **The motors stop by themselves if no drive command arrives for 0.3 s.**
- **Head:** servo on GPIO 13, 30–150° (90 = straight ahead). **Eyes:** two 8×8 LED matrices (I2C 0x71) that blink at random every 2–6 s and glance around in between: the pupils move one dot at a time to a new spot every 1.5–5 s, returning to center about half the time.
- **Control:** text commands over UDP port 4210 (listed at the top of `robot_wifi.ino`), plus a web page at `http://robot<id>.local/` with a drive pad, head slider and live status (battery, wheels, Wi-Fi signal).
- **Wireless updates (OTA):** after the first USB load, new firmware goes over Wi-Fi with a password.

## Setting up a robot

1. One time on the laptop: install the Pico board package for Arduino.
   ```bash
   arduino-cli core install rp2040:rp2040 --additional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
2. Copy `robot_wifi/secrets.example.h` to `robot_wifi/secrets.h` and fill it in: the router's name and password, plus a password for each robot's own network and one for wireless updates. Git ignores this file, so share it with teammates privately. Everyone uploading firmware needs the same `OTA_PASSWORD`.
3. First load, with the Pico plugged in by USB (use the robot number, 1–8):
   ```bash
   laptop/upload_usb.sh 3
   ```
4. From then on, over Wi-Fi:
   ```bash
   laptop/upload_wifi.sh 3
   ```

Tune behaviour in `robot_wifi/config.h`: speed cap, minimum motor power, wheel direction flags, servo offset and eye brightness. The robot number is set by the upload scripts, so `config.h` doesn't need editing per robot.

## Hardware notes

- Motor pins come from Freenove's code: front-left 18/19, rear-left 21/20, rear-right 7/6, front-right 9/8. Battery sense is GPIO 26.
- Power: the board wants 7.0–8.4 V. We use 5 alkaline AAs (about 7.9 V). Hard acceleration can drop the voltage to about 5 V and restart the Pico, so keep the speed at or below about 55%, or use 6 NiMH AAs.
