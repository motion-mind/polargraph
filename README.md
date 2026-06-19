# Polargraph Firmware — feature/multi-pen branch
> **Branch:** `feature/multi-pen` — four independent pen servos, colour separation
> **Base:** `main` (single-pen, all fixes applied)

# Polargraph Firmware v4.0 — Fysetc E4

A modern hanging pen-plotter controller for the
[Fysetc E4](https://github.com/FYSETC/FYSETC-E4) (ESP32 + 4× TMC2209).

---

## Updating the firmware — no toolchain required after first flash

### Every update after the first

1. Go to **GitHub → Actions → latest build → Artifacts**
   and download `polargraph-ota-N.zip`
2. Extract to get `polargraph-firmware.bin` and `polargraph-filesystem.bin`
3. Open `http://<polargraph-ip>/update` in any browser
4. Upload **`polargraph-firmware.bin`** → click Update → wait for reboot
5. Upload **`polargraph-filesystem.bin`** → click Update → wait for reboot

Or push a version tag (`git tag v1.2.3 && git push --tags`) and GitHub
creates a Release with all binaries attached.

---

## First-time flash

### Option 1 — Browser flasher (recommended, nothing to install)

**`https://YOUR_USERNAME.github.io/polargraph/`**

Works in **Chrome** or **Edge** (version 89+). Firefox and Safari don't
support Web Serial yet.

1. Plug the Fysetc E4 into USB-C
2. Open the link above
3. Click **Connect & Flash**
4. Select the serial port from the browser dialog (CP2102 or CH340)
5. Wait ~30 seconds — done

The flasher page is hosted on GitHub Pages and is automatically updated
on every push to `main`. The blobs it flashes are built by the same CI
job, so the page always serves the current firmware.

### Option 2 — Python script (fallback, needs Python 3 only)

Download `polargraph-first-flash.zip` from Releases or Actions, extract,
run the script for your OS:

| OS | Command |
|----|---------|
| Windows | Double-click `flash.bat` |
| macOS / Linux | `./flash.sh` |

Python 3 is the only prerequisite — the script installs `esptool`
automatically via pip.

### Option 3 — PlatformIO (developers only)

```bash
pip install platformio
pio run -t upload && pio run -t uploadfs
```

### After flashing (all options)

1. Board creates WiFi AP: **`Polargraph-Setup`** (password: `polargraph`)
2. Connect to it, open **`http://192.168.4.1/wifi`**
3. Enter your home WiFi credentials → Save
4. Board reboots onto your network
5. Find its IP on your router → open **`http://<ip>/`**
6. All future updates via **`http://<ip>/update`** — no USB again

### Enable GitHub Pages (one-time repo setup)

After pushing to GitHub:
1. Go to repo **Settings → Pages**
2. Source: **GitHub Actions**
3. Save

The next push to `main` will deploy the flasher automatically.

### Troubleshooting first flash

| Symptom | Fix |
|---------|-----|
| "Web Serial not supported" | Use Chrome or Edge, not Firefox/Safari |
| "Failed to connect" | Hold **BOOT** button while clicking Flash |
| Port not listed (Windows) | Install [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html) driver |
| Linux: permission denied | `sudo usermod -aG dialout $USER` then log out |
| Flashing fails mid-way | Try a different USB cable (some are charge-only) |

---

## Architecture

### Job execution — two paths

```
Browser image processor
        │
        ├── .plg binary  ──────────► BinJobExecutor (Core 0)
        │   Pre-planned segments         fread → pushBinSeg → ISR queue
        │   Full global lookahead        ~4µs per segment
        │
        └── .gcode text  ──────────► JobExecutor (Core 0)
            Per-device planning          parse → kinematics → plan → queue
            Interop with ext. tools      ~120µs per segment
```

Both paths feed the same `MotionPlanner` ring buffer and ISR.
WiFi / web server runs on Core 1, never blocked by motion execution.

### Binary format (.plg)

```
[PlgHeader — 32 bytes]
  magic[4]      'PLG1'
  version       2
  segCount      u32 — total segments
  machineWidth  f32 mm
  machineHeight f32 mm
  stepsPerMm    f32
  estDurationMs u32

[PlgSeg × segCount — 20 bytes each]
  deltaL        i32 — left cord step delta
  deltaR        i32 — right cord step delta
  entryInterval u16 — µs/step at start
  cruiseInterval u16 — µs/step at cruise
  exitInterval  u16 — µs/step at end
  accelSteps    u16
  decelSteps    u16
  flags         u8  — PLG_PEN_DOWN(1) PLG_PEN_UP(2) PLG_LAST(4) PLG_RAPID(8)
  pad           u8
```

Velocity is pre-planned in the browser with a two-pass algorithm
(forward + reverse lookahead across the entire job) before upload.
The ESP32 ISR does zero floating-point math during execution.

### Velocity planning (browser-side two-pass)

```
Forward pass:  entry[i] = min(maxFeed, sqrt(exit[i-1]² + 2·a·len[i]))
Reverse pass:  exit[i]  = min(maxFeed, sqrt(entry[i+1]² + 2·a·len[i]))
Profile:       trapezoid(entry[i], cruise, exit[i]) per segment
```

This gives every segment the globally optimal speed — faster than
anything achievable with a bounded lookahead window on-device.

---

## Hardware wiring

```
Fysetc E4 socket     Polargraph
─────────────────────────────────────────
X motor (E0)    →    Left cord motor
Y motor (E1)    →    Right cord motor
FAN1 (GPIO 2)   →    Servo signal
5V / GND        →    Servo power
USB-C           →    First flash only
```

---

## Web interface

| URL              | Purpose                                    |
|------------------|--------------------------------------------|
| `http://<ip>/`   | Dashboard: position, job control, SG bars  |
| `http://<ip>/image` | Image processor: upload → convert → send |
| `http://<ip>/update` | OTA firmware / filesystem flash          |
| `http://<ip>/wifi`  | WiFi credential setup                    |
| `http://<ip>/status` | JSON status (position, job progress)    |
| `http://<ip>/upload` | POST a .plg or .gcode file              |
| `http://<ip>/job/start` | Start uploaded job                   |
| `http://<ip>/job/pause` | Pause                                |
| `http://<ip>/job/resume` | Resume                              |
| `http://<ip>/job/stop`   | Stop + flush motion queue           |

---

## Machine setup (serial, 250000 baud)

```gcode
M104 S700    ; machine width mm (anchor to anchor)
M105 S900    ; canvas height mm
M92  X80     ; steps/mm  (200step × 16µstep ÷ 40mm/rev GT2)
M203 X150    ; max feedrate mm/s
M201 X400    ; acceleration mm/s²
M914 X80     ; StallGuard threshold (0–255, lower=more sensitive)
M500         ; save to flash
G28          ; sensorless home
```

---

## Sensorless homing (StallGuard2)

The TMC2209's StallGuard2 detects motor stall without physical switches.
During homing, both cord motors retract until stall is detected (cord
fully wound), then pay out to the home position.

Tune with `M914`:
```gcode
M914 X80     ; SG threshold — reduce if stall not detected
M914 S30     ; homing speed mm/s — reduce if false triggers
M914 B5      ; back-off distance mm after stall
M500
```

Watch live SG values on the dashboard's tension bars while jogging
to find the right threshold for your cord tension and motor current.

---

## Image processing workflow

1. Open `http://<ip>/image`
2. Drop an image (JPG/PNG/etc.)
3. Adjust brightness/contrast/gamma
4. Choose algorithm:
   - **Floyd-Steinberg** — classic error-diffusion dither, best for photos
   - **Ordered (Bayer)** — regular dot pattern, good for graphics
   - **Scanline Hatching** — ruled lines, very pen-plotter aesthetic
   - **Contour/Isoline** — topographic look, traces brightness contours
   - **Weighted Stipple** — random dots weighted by image darkness
5. Click **▶ Process** — builds G-code preview AND binary .plg simultaneously
6. Go to **③ Send** tab → enter machine IP → **⬆ Upload & Start**
   - Uploads the `.plg` binary (preferred) automatically
   - Falls back to `.gcode` if binary generation failed
7. Monitor progress on dashboard or leave it — machine runs autonomously

---

## File sizes (typical A4 dithered portrait)

| Format   | Segments | File size | Parse time |
|----------|----------|-----------|------------|
| `.gcode` | ~8,000   | ~1.2 MB   | ~1 s       |
| `.plg`   | ~8,000   | ~160 KB   | ~25 ms     |

---

## G-code reference

```
G0  Xnnn Ynnn          Rapid move
G1  Xnnn Ynnn Fnnn     Linear move (F mm/min)
G4  Pnnn               Dwell (ms)
G28                    Sensorless home
G90 / G91              Absolute / Relative
M3  [Snnn]             Pen down [angle]
M5                     Pen up
M92  Xnnn              Steps/mm
M104 Snnn              Machine width (mm)
M105 Snnn              Machine height (mm)
M114                   Report position
M201 Xnnn              Acceleration (mm/s²)
M203 Xnnn              Max feedrate (mm/s)
M280 P0|1 Snnn         Servo angle (P0=up, P1=down)
M500                   Save settings
M502                   Reset to defaults
M503                   Print settings
M906 Xnnn              Motor current (mA RMS)
M914 [Xnnn] [Snnn] [Bnnn]  StallGuard threshold / speed / backoff
M100                   Help
```

---

## Building locally

```bash
pip install platformio
pio run                      # build only
pio run -t upload            # build + flash firmware (USB)
pio run -t uploadfs          # build + flash filesystem (USB)
pio device monitor           # serial monitor at 250000 baud
```

## Releasing a new version

```bash
git tag v1.2.3
git push origin v1.2.3
# GitHub Actions builds and creates a Release with .bin files attached
# Flash via http://<ip>/update — no local tools needed
```

---

## Multi-pen wiring (feature/multi-pen branch)

```
E4 Header    GPIO   Pen    Servo signal
─────────────────────────────────────────
HEAT_E0       2     Pen 0  (top-left  in gondola)
HEAT_BED      4     Pen 1  (top-right)
FAN_E0       13     Pen 2  (bot-right)
Z endstop    14     Pen 3  (bot-left)
```

Power all four servos from the board's 5V pins — do NOT use the
heater/fan headers for power (those are 12/24V switched MOSFETs).

### Gondola arrangement (top view)

```
┌────────────┐
│  P0    P1  │
│            │
│  P3    P2  │
└────────────┘
```

### Calibrating tool offsets

With all pens loaded, measure the XY distance from gondola centre
to each pen tip:

```gcode
M218 T0 X-10 Y-10   ; pen 0 is 10mm left and 10mm up from centre
M218 T1 X10  Y-10   ; pen 1 is 10mm right, 10mm up
M218 T2 X10  Y10    ; pen 2 is 10mm right, 10mm down
M218 T3 X-10 Y10    ; pen 3 is 10mm left,  10mm down
M500                 ; save
```

### Colour workflow

1. Open `http://<ip>/image`
2. Upload image → Process on **① Image** tab
3. Switch to **② Colours** tab → choose CMYK or RGB mode → **Separate**
4. Review per-layer previews
5. **③ Export** → download `.plg` binary
6. **④ Send** → Upload & Start

The machine will print each layer in sequence, pausing between layers
only long enough for the servo to switch pens.
