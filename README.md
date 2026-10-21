# Tactile Vision Device
> Real-time visual-to-tactile substitution device for the visually impaired.  
> 카메라의 깊이 정보를 24개의 촉각 핀으로 변환해 공간을 손끝에 전달합니다.

---

## Hardware Preview

<a href="hardware/full/7_22 set_final.stl">
  <img src="https://img.shields.io/badge/View%203D%20Model-STL-blue?style=for-the-badge&logo=autodesk" alt="View 3D Model"/>
</a>

> Click the badge above, or click the STL file directly on GitHub to launch the interactive 3D viewer.

**[→ Open Full Assembly STL (interactive 3D viewer)](hardware/full/7_22%20set_final.stl)**

| Part | STL |
|---|---|
| Full Assembly (24-zone tactile display) | [7_22 set_final.stl](hardware/full/7_22%20set_final.stl) |
| Servo + Rack & Pinion unit | [SG90 and Rack and Pinion.stl](hardware/assembly/SG90%20and%20Rack%20and%20Pinion.stl) |
| Rack | [rack.stl](hardware/parts/rack.stl) |
| Pinion | [pinion.stl](hardware/parts/pinion.stl) |
| Servo bracket | [servo_bracket.stl](hardware/parts/servo_bracket.stl) |
| 3-tier stack, assembled (preview only) | [stack_assembled.stl](hardware/stack/stack_assembled.stl) |
| Tier 2 — Pico, PCA9685 ×2, step-down ×2, battery ×2 | [tier2.stl](hardware/stack/tier2.stl) |
| Tier 3 — Jetson, power bank, C920 | [tier3.stl](hardware/stack/tier3.stl) |
| Tier 3 side filler bars ×2 | [tier3_side_fillers.stl](hardware/stack/tier3_side_fillers.stl) |
| Print plate — tiers 2 + 3 + fillers on one H2D plate | [print_plate_nested.stl](hardware/stack/print_plate_nested.stl) |

---

## Project Structure

```
tactile-vision-device/
├── software/
│   ├── main.py              # Jetson main pipeline
│   ├── requirements.txt
│   └── pico/
│       └── main.cpp         # Raspberry Pi Pico firmware
└── hardware/
    ├── parts/               # rack.stl / pinion.stl / servo_bracket.stl
    ├── assembly/            # SG90 + rack & pinion unit
    ├── full/                # complete 24-zone tactile display (tier 1)
    └── stack/               # tiers 2–3 housing + generator (stack_tiers.py)
```

---

## System Overview

```
Camera
  │
  ▼
Jetson Orin Developer Kit
  ├─ Depth Anything V2 (TensorRT)
  ├─ 6×4 grid depth mapping (24 zones)
  └─ UART  "GRID:v0,...,v23\n"
          │
          ▼
Raspberry Pi Pico
  ├─ I2C → PCA9685 #1 (0x40) → SG90 ×12  [rows 1–2]
  └─ I2C → PCA9685 #2 (0x41) → SG90 ×12  [rows 3–4]
                  │
                  ▼
        Rack & Pinion — Z-axis pin extrusion
```

## Grid Layout

```
[ 0][ 1][ 2][ 3][ 4][ 5]   row 1 ┐ PCA9685 0x40
[ 6][ 7][ 8][ 9][10][11]   row 2 ┘
[12][13][14][15][16][17]   row 3 ┐ PCA9685 0x41
[18][19][20][21][22][23]   row 4 ┘
```

Higher depth value = closer object = pin extends further.

---

## Hardware

| Component | Spec |
|---|---|
| Main board | NVIDIA Jetson Orin Developer Kit |
| MCU | Raspberry Pi Pico (RP2040) |
| PWM driver | PCA9685 × 2 (I2C: `0x40`, `0x41`) |
| Actuator | SG90 servo × 24 |
| Mechanism | Rack & pinion, Z-axis extrusion (3D printed) |
| Camera | Logitech C920 (USB, MJPEG 720p) |
| Servo power | Battery × 2 → step-down × 2, one line per PCA9685 |
| Jetson power | Power bank |
| Housing | 3-tier stack, 3D printed (Bambu Lab H2D) |

### Housing (3-tier stack)

```
tier 1  tactile display (hardware/full)   servos on top, wires drop through 24 slots
tier 2  200 × 236 × 65 mm                  Pico, PCA9685 ×2, step-down ×2, battery ×2
tier 3  252 × 250 × 86 mm                  Jetson, power bank, C920 (front window)
total height ≈ 203 mm
```

- Each tier sits inside the rim of the tier below; lower tiers are wider.
- Tier 2 has a center divider that splits the PCA9685 `0x40` and `0x41` power lines left/right. Lower cross dividers separate the battery, step-down and PCA bays. The Pico bay at one end spans both sides.
- Tiers 2–3 have 12 matching Ø4 holes for cable ties or M3 bolts. The tier 2 floor has 19 cable slots.
- Parts are fixed with cable ties or tape, so there are no mounting bosses.
- **One-plate print:** tier 2 sits rotated 90° inside tier 3, 3 mm above its floor, with the two filler bars beside it. Total footprint is 252 × 305 mm. Enable supports so the 3 mm gap under tier 2 is filled. Use **normal(auto)**, not tree(auto): tree support stalls slicing on the large flat floor. Bambu Studio estimate (H2D 0.4, 0.20 mm Standard, PLA Basic): about 16 h 44 m and 783 g.
- To regenerate after changing dimensions: `pip install trimesh manifold3d networkx`, then `python hardware/stack/stack_tiers.py`. All dimensions are in `PARAMETERS` at the top of the script.

---

## Getting Started

### Jetson

```bash
pip install -r software/requirements.txt
# Place TRT engine → checkpoints/depth_anything_v2_vits.engine

python software/main.py              # run
python software/main.py --debug      # with visualization
python software/main.py --no-hw      # software only
```

| Option | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyACM0` | Pico serial port |
| `--baud` | `115200` | Baud rate |
| `--debug` | `False` | Show depth grid overlay |
| `--no-hw` | `False` | Run without hardware |

### Pico Firmware

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add board URL in **Preferences**:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. Install **Raspberry Pi Pico/RP2040** via Board Manager
4. Install **Adafruit PWM Servo Driver Library**
5. Upload `software/pico/main.cpp`

**I2C wiring:** `GP4` → SDA, `GP5` → SCL

**Before first run** — lower `SAFE_MAX` in `pico/main.cpp` to avoid mechanical damage:
```cpp
static constexpr uint16_t SAFE_MAX = 350;  // increase gradually after testing
```

---

## References

- [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- [Adafruit PWM Servo Driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
- [arduino-pico](https://github.com/earlephilhower/arduino-pico)
