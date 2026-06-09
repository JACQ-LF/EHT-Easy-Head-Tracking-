# EHT (Easy Head Tracking)
 
A DIY head tracking system using ESP32 and BMI160 for gaming and simulations, compatible with OpenTrack.
 
## 🎯 Features
 
- **3DOF Tracking**: Yaw, Pitch, Roll
- **Bluetooth LE**: Wireless connection, no WiFi setup required
- **Mahony Filter**: Quaternion-based fusion — no gimbal lock, minimal drift
- **Gyro Calibration**: Automatic offset calibration at startup
- **High Frequency Yaw**: Dedicated core for gyro integration
- **Low Latency**: Less than 20ms end-to-end
- **OpenTrack Compatible**: Plug & play with games and simulators
## 🔧 Components
 
### Current Version (v1.0)
- **ESP32-C3 SuperMini** (or any ESP32 with BLE) - ~$5
- **BMI160** (accelerometer + gyroscope) - ~$4
- **4 jumper wires** female-female - ~$2
- **Case/mount** (3D printing recommended) - ~$2
### Future Version (v2.0) - In Development
- **ESP32-S3 N4** - ~$3.8 (JLCPCB)
- **ICM-20948** (9-axis, accelerometer + gyroscope + magnetometer) - ~$4.5
- **EHT Custom PCB** (currently being designed)
- **Li-Ion Battery** (500-1000mAh)
- **Charging Circuit** (integrated PCB)
- **ON/OFF Switch**
- **Custom Case** (3D printed) - <$1
## 📐 Wiring Diagram
 
### Version v1.0
```
ESP32-C3       BMI160
--------       ------
3.3V    ---    VCC
GND     ---    GND
GPIO8   ---    SDA
GPIO9   ---    SCL
```
 
## 🚀 Installation
 
### 1. Prepare Environment
```bash
# Install Arduino IDE
# Add ESP32 support:
# File > Preferences > Additional Board Manager URLs
# https://dl.espressif.com/dl/package_esp32_index.json
```
 
### 2. Install Library
- Open Arduino IDE → Tools → Manage Libraries
- Search for `ESP32 BLE Gamepad` by **lemmingDev** and install it
### 3. Flash ESP32
- Select your ESP32 board
- Select serial port
- Compile and upload
- Keep the device **still** during the 2-second gyro calibration at startup
### 4. Pair via Bluetooth
- Open Windows Bluetooth settings
- Pair `ESP32 Head Tracker` like any BLE device
## 🎮 OpenTrack Configuration
 
### 1. Install OpenTrack
Download from: https://github.com/opentrack/opentrack/releases
 
### 2. Configuration
1. **Input**: `Joystick input`
   - Select `ESP32 Head Tracker` from the device list
2. **Output**: Choose according to your game
   - `freetrack 2.0 Enhanced` (most games)
   - `TrackIR` (for compatible games)
3. **Axis mapping**:
   - Joystick Axis 1 → Pitch
   - Joystick Axis 2 → Roll
   - Joystick Axis 3 → Yaw
### 3. Use
- Power on your EHT
- Stay still for 2 seconds while the gyro calibrates
- Open OpenTrack and click **Start**
- The octopus should move with your head
> ⚠️ **Star Citizen / DCS users**: Use OpenTrack's `freetrack` or `TrackIR` output — do **not** use the joystick axes directly in-game, as the device would be picked up as a controller and interfere with flight inputs.
 
## 📊 Performance
 
- **Frequency**: 100Hz output, high-frequency yaw integration on dedicated core
- **Total Latency**: < 20ms
- **Accuracy**: ±1° (pitch/roll), ±2° (yaw)
- **Drift**: None on pitch/roll (accelerometer correction), slow drift on yaw (no magnetometer)
## 🔧 Advanced Settings
 
### Sensitivity Adjustment
```cpp
// In OpenTrack mapping curves — preferred over code changes
// Or modify the map() ranges in the sketch:
int16_t out_pitch = (int16_t)map(pitch, -45.0f, 45.0f, 0, 32767); // narrower = more sensitive
```
 
### Mahony Filter Tuning
```cpp
#define ALPHA_ACC 0.02f  // 0.01–0.05 recommended
// Higher = accelerometer corrects faster (less drift, more noise on movement)
// Lower  = gyro dominates (smoother, slower correction)
```
 
### Yaw Drift
```cpp
#define YAW_DECAY 1.0f   // Set < 1.0 (e.g. 0.9995) to add auto-return-to-center
```
 
## 🛠️ Troubleshooting
 
### Device not appearing in Bluetooth
- Make sure ESP32 finished calibration (check Serial monitor)
- Remove old pairing in Windows and re-pair
### Joystick not showing in OpenTrack
- Check Windows → Game Controllers (`joy.cpl`) — the device must appear there first
- Reconnect via Bluetooth if needed
### Angle Drift
- **Pitch/Roll**: Should be drift-free thanks to accelerometer correction
- **Yaw**: Normal without magnetometer — recalibrate with `Ctrl+Home` in OpenTrack
- Reduce `ALPHA_ACC` if pitch/roll shake during fast movements
### Erratic Data
- Check I2C wiring (SDA/SCL)
- Verify stable 3.3V power supply
- Check Serial monitor for gyro offset values at startup — large offsets (>5°/s) indicate a wiring or power issue
## 🤝 Contributing
 
Contributions are welcome! Feel free to:
- Report bugs
- Suggest improvements
- Share hardware modifications
- Add support for more games
## 📄 License
 
See the `LICENSE` file for details.
 
## 🎮 Tested Games
 
- ✅ **DCS World** (freetrack)
- ✅ **Star Citizen** (freetrack)
## 🔮 Roadmap
 
### Version 1.0 (Current) ✅
- [x] 3DOF tracking (Yaw, Pitch, Roll)
- [x] Bluetooth LE transmission (no WiFi required)
- [x] Mahony quaternion filter (no gimbal lock)
- [x] Gyro calibration at startup
- [x] High-frequency yaw on dedicated core
- [x] OpenTrack compatibility
### Version 2.0 (Planned)
- [ ] **EHT Custom PCB** - Compact and professional design
- [ ] **Magnetometer support** - Eliminates yaw drift completely
- [ ] **Integrated battery** - 8-12h autonomy
- [ ] **USB-C charging circuit** - Fast charging
- [ ] **Status LEDs or Small OLED Screen** - Battery, BLE, tracking status
---
 
**⚠️ Important**: This project is for personal use. Please respect the terms of use of games and simulators.
