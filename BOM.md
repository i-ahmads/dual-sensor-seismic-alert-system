# Main components

This is the prototype design's component list. The etched PCBs shown in the photographs are unpopulated; a breadboarded ESP32 was used for the demonstration.

| Stage | Component | Model or value | Quantity |
|---|---|---|---:|
| Power, per planned board | LiPo cell | 3.7 V, 1000 mAh | 1 |
| Power, per planned board | Charger module | TP4056 | 1 |
| Power, per planned board | Buck module | LM2596 | 1 |
| Sensing | Accelerometer breakout | MPU-6050 | 1 |
| Sensing | Piezo vibration module | RBD-2229 | 1 |
| Signal conditioning | Dual op amp | LM358N | 1 |
| Signal conditioning | Resistors | 100 kΩ × 4; 27 kΩ; 10 kΩ | 6 |
| Signal conditioning | Capacitors | 100 nF × 4 | 4 |
| Processing | Sensing MCU | ESP32-S3-DEVKITC-1U-N8R8, planned | 1 |
| Gateway | Alarm MCU | ESP32 DevKitC, planned | 1 |
| Alarm | Discrete LEDs | Red, green, blue | 3 |
| Alarm | LED resistors | 220 Ω | 3 |
| Alarm | Piezo buzzer and series resistor | CSS-73B16K-SMT; 100 Ω | 1 each |

The LM2596 input headroom, F1 fuse footprint, and measured piezo bias require correction or bench verification before battery-powered hardware use. See the [README](README.md#known-limitations).
