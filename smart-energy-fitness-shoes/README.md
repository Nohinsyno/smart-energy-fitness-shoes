# Smart Energy-Generating Fitness Shoes ⚡

A wearable IoT system that harvests electrical energy from footsteps using piezoelectric sensors and tracks fitness activity over Bluetooth Low Energy (BLE).

## Features
- Piezoelectric energy harvesting from walking pressure
- Step detection from real foot pressure (not motion estimation)
- Live BLE dashboard: steps, calories burned, distance walked, energy generated (mWh)
- Web dashboard connects through Web Bluetooth (Chrome / Edge)
- Demo mode to preview the dashboard without hardware

## Repository structure
| Path | Description |
|------|-------------|
| `index.html` | Web dashboard (HTML, CSS, JavaScript) |
| `firmware/SmartSole_ESP32/SmartSole_ESP32.ino` | ESP32 Arduino firmware |

## Hardware
- ESP32 microcontroller
- 2x piezoelectric disc sensors (heel and toe)
- Signal conditioning: bridge rectifier, voltage divider, 3.3 V Zener clamp
- TP4056 Li-Po charger module and a 3.7 V Li-Po battery

## Wiring
| Sensor | ESP32 pin |
|--------|-----------|
| Heel piezo (rectified) | GPIO34 |
| Toe piezo (rectified) | GPIO35 |

## How to run
1. Open `firmware/SmartSole_ESP32/SmartSole_ESP32.ino` in Arduino IDE (ESP32 board package installed) and flash it.
2. Open `index.html` in Chrome or Edge (or the GitHub Pages link).
3. Go to **Connect**, click **Scan for ESP32**, and select **SmartSole**.

## BLE data format
The ESP32 notifies every 200 ms with a CSV string:

`steps,calories,distanceKm,energyMWh,heelADC,toeADC`

## Energy estimate
`E = (V² / R) × pulse duration`, calculated from the peak ADC voltage across the load resistor on each detected step.

## Tech stack
`C++` `Arduino` `ESP32` `BLE` `HTML5` `CSS3` `JavaScript`
