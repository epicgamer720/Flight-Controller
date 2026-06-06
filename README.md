# Rocket Flight Controller

A custom, open-source flight computer for model and high-power rockets — STM32F722 (Cortex-M7) with onboard IMU, barometer, GPS, dual pyro channels, quad servo outputs, and microSD logging. Designed from scratch in KiCad.



## Why I built it

Wanted to get my feet wet with more mixed PCB design. I designed this as a base for firmware and testing I plan I designing another one after I build and test this one on actual rockets. I plan to use this for my rocketry team for active control and data logging 

## Use cases

- **Dual-deploy recovery** — fire a drogue at apogee and main at a set altitude using the two pyro channels.
- **Active stabilization** — drive the four servo outputs for thrust-vector control or canard/fin steering during boost.
- **Flight data logging** — log IMU, barometer, and GPS to microSD for post-flight analysis (apogee, max velocity, flight path).
- **GPS tracking** — downrange tracking and finding the rocket after landing.
- **Telemetry** — optional LoRa (SX1262, 915 MHz) expansion for live altitude/position downlink.
- **General avionics dev board** — the IMU + baro + GPS + servo + logging combo also works for drones, RC, robotics, or balloon payloads.



<img width="849" height="813" alt="image" src="https://github.com/user-attachments/assets/19678770-1fe9-4f89-9dc1-16184f4c644c" />
<img width="682" height="753" alt="image" src="https://github.com/user-attachments/assets/2e024282-7264-4718-ab76-eee511c1b3b9" />
<img width="607" height="740" alt="image" src="https://github.com/user-attachments/assets/d1cdeaf3-4803-4a66-af96-be26f582517d" />

## Specifications

| Subsystem | Part / detail |
|---|---|
| MCU | STM32F722RET6 — Cortex-M7, up to 216 MHz |
| IMU | ICM-45686 — 6-axis accel + gyro |
| Barometer | BMP580 — altitude / apogee detection |
| Storage | microSD — flight data logging |
| Recovery | 2× pyro channels — drogue + main |
| Control | 4× servo outputs — TVC / fin / canard |
| GPS | JST connector + PRTR5V0U2X ESD protection |
| Telemetry | LoRa SX1262 (915 MHz) expansion header |
| Power | 2S LiPo, BQ25883 charger, USB-C |
| Status | Onboard status LED |



## This project uses:

- [KiCad](https://www.kicad.org/)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [easyeda2kicad.py](https://github.com/uPesy/easyeda2kicad.py)
- [Hack Club Blueprint](https://blueprint.hackclub.com/)
