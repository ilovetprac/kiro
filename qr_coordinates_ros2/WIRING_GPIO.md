# Wiring: Raspberry Pi GPIO → Microstep Drivers (no Arduino)

Two STEP/DIR microstep drivers (TB6600-style), driven directly from the Pi GPIO.

## DIP switches (set BOTH drivers identically)
- **Microstep 1/8 = 1600 pulse/rev:** SW1 **OFF**, SW2 **ON**, SW3 **OFF**
- **Current 0.5 A (PK 0.7 A):** SW4 **ON**, SW5 **ON**, SW6 **ON**
- With a 1 mm screw lead → **1600 steps/mm** (0.000625 mm/step; far finer than 1 mm).

## Signal wiring (common-cathode: drive the `+` pins, tie `-` pins to Pi GND)

| Driver pin | Connect to (Pi) | BCM | Physical pin |
|-----------|-----------------|-----|--------------|
| **X** PUL+ | X STEP          | GPIO17 | 11 |
| **X** DIR+ | X DIR           | GPIO27 | 13 |
| **Y** PUL+ | Y STEP          | GPIO22 | 15 |
| **Y** DIR+ | Y DIR           | GPIO23 | 16 |
| PUL- , DIR- (both drivers) | Pi **GND** | — | 9 (or any GND) |
| ENA+ / ENA- | leave unconnected (enabled by default) | — | — |

## Power + motor wiring (NEVER connect 24 V to the Pi)

| Driver pin | Connect to |
|-----------|------------|
| VCC | **24 V supply +** |
| GND | 24 V supply − (motor ground) |
| A+ , A− | Motor coil A |
| B+ , B− | Motor coil B |

## ⚠️ 3.3 V vs 5 V note
The inputs are labelled "+5V". The Pi GPIO is 3.3 V. Many TB6600-style opto
inputs trigger fine at 3.3 V — try it first. If steps are missed/erratic, add a
**3.3 V → 5 V level shifter** on the PUL+ and DIR+ lines (powered from the Pi 5 V
pin). Do not connect the Pi's 5 V/3.3 V and the 24 V grounds together except
through the common-cathode signal ground as shown.

## Software setup
```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable --now pigpiod        # hardware-timed pulses
```

## Run
```bash
ros2 launch qr_coordinates_ros2 agv_align.launch.py
# if an axis moves the wrong way:
ros2 launch qr_coordinates_ros2 agv_align.launch.py invert_x:=true invert_y:=true
```

## First-power checklist
1. DIP switches set on BOTH drivers (above). 24 V to VCC/GND, motors on A/B.
2. Signal wires per the table; **centre both slides** (power-on = 0,0).
3. Start pigpiod, then launch. Show ArUco ID 1 near centre (0.3–1.5 m).
4. Test one axis at a time, slides free, hand near the 24 V switch. If an axis
   drives away from centre, set invert_x/invert_y true for that axis.

No end-stops: the Pi tracks position only relative to power-on. Re-centre the
slides before every power-on (add limit switches later for auto-homing).
