# Wiring guide

Built for this hardware:

- **ACEBOTT ESP32 Max V1.0** board, used on its own (no car shield)
- **SparkFun Motor Driver - Dual TB6612FNG (1A)**. This version ships without header pins, so solder pins or wires to the holes you use.
- **2 gearmotors** (24 mm, 6 V, 260 rpm), one per side
- **MPU-6050** gyro/accelerometer
- **2S Li-ion battery** (two cells in series, 7.4 V nominal, 8.4 V fully charged) wired directly to the motor driver

The driver's pins are labeled `VM VCC GND STBY AIN1 AIN2 PWMA BIN1 BIN2 PWMB A01 A02 B01 B02`. If your board has anything else on it (for example a Qwiic cable connector), stop and tell us before wiring.

Every pin number below matches `src/config.h`. If you wire something differently, change it there too.

## The big picture

```
             start switch
 Battery + ----/ ----------+---------------------> VM         (motor driver)
 (2S pack)                 |
                           +---------------------> DC input   (ESP32 board, rated 6 to 18 V)

 Battery - ------------------+--------------------> GND  (motor driver)
                              +-------------------> GND  (ESP32 board)
                              +-------------------> GND  (MPU-6050)

 ESP32 3.3 V -------------------+------------------> VCC  (motor driver)
                                 +-----------------> VCC  (MPU-6050)
```

Three rules:

1. **Everything shares one ground.** Battery -, ESP32, motor driver and MPU-6050 grounds are all connected. Most "nothing works" problems are a missing ground.
2. **Battery voltage goes only to `VM` and the ESP32 board's DC input.** Never put it on a 3.3 V pin, on the MPU-6050 `VCC`, or on the driver's `VCC`.
3. **Wire with the battery disconnected.**

## A. ESP32 to motor driver

| ESP32 pin | Motor driver pin | What it does |
|-----------|------------------|--------------|
| GPIO25 | PWMA | left motor speed |
| GPIO26 | AIN1 | left motor direction |
| GPIO27 | AIN2 | left motor direction |
| GPIO32 | PWMB | right motor speed |
| GPIO33 | BIN1 | right motor direction |
| GPIO13 | BIN2 | right motor direction |
| GPIO4 | STBY | turns the driver on (the board holds this low until the code raises it, so the motors cannot move before the code starts) |
| 3.3 V (see note) | VCC | powers the driver's logic |
| GND | GND | ground |

**`VCC` must be 3.3 V, not 5 V.** The TB6612FNG only counts a control input as "high" above 0.7 times `VCC` (Toshiba datasheet). At 5 V that is 3.5 V, which is higher than an ESP32 pin can output (3.3 V), so the driver would ignore or misread the commands. If your board offers a 3.3 V or 5 V choice on its power pins, pick 3.3 V, and measure it with a multimeter before connecting `VCC`.

The motor driver has several `GND` pins. They are all the same connection; use any.

ESP32 boards print pin names differently: GPIO25 may be marked `25`, `D25` or `IO25`. They all mean the same pin. If your board's pins come in groups of three (signal, voltage, ground), the signal wire goes on the signal pin of that group.

## B. ESP32 to MPU-6050

| ESP32 pin | MPU-6050 pin |
|-----------|--------------|
| 3.3 V | VCC |
| GND | GND |
| GPIO21 | SDA |
| GPIO22 | SCL |

The MPU-6050 must run from 3.3 V too: its I2C lines are pulled up to its own `VCC`, and the ESP32 pins are not 5 V tolerant.

Leave `AD0`, `INT`, `XDA` and `XCL` unconnected. (`AD0` unconnected means the address is 0x68, which is what the code expects.)

**Mounting matters as much as the wires.** Fix the board flat to the chassis, as close to the middle as you can, not on anything that moves, with the printed X arrow pointing toward the front. If it ends up any other way, nothing is wrong, but you must tell the code: the bench checklist at the top of `src/main.cpp` shows how.

## C. Motors to motor driver

| Motor | Driver terminals |
|-------|------------------|
| Left motor | `A01` and `A02` |
| Right motor | `B01` and `B02` |

Either wire can go on either terminal. If a motor later spins the wrong way, do not rewire it: flip its `invert` in `src/config.h` (the right motor is already set to `true`, since the two motors face opposite directions).

## D. Power

**Motor power.** Battery + goes through the **start switch** to `VM` on the motor driver. Battery - goes to `GND`.

**ESP32 power.** The same switched battery + also goes to the ESP32 board's DC power input, which ACEBOTT rates for 6 to 18 V. A 2S pack is inside that range, so no extra regulator is needed. Check how that input is exposed on your board (barrel jack or terminal) and its polarity with a multimeter before connecting.

**Do not feed the ESP32 board from USB and the battery at the same time.** ACEBOTT's documentation does not say whether that is safe. For bench testing (which needs USB for the serial monitor), power the ESP32 from USB only and connect the battery to the motor driver alone (`VM` and `GND`, grounds shared). For real runs, unplug USB and power the ESP32 from the battery.

- **Watch the low end.** A 2S pack drops toward 6 V as it empties, and voltage sags further when the motors pull hard. Below 6 V the ESP32 board can reset in the middle of a run. Start every run on fully charged cells, and when testing, climb with the battery at a low-ish charge once to see whether it resets.
- **Motor voltage.** The motors are rated 6 V and will see up to 8.4 V at full command, which is a deliberate small overvolt. The speeds in the code are below full (a command of 200 out of 255 is about 6.6 V on a fully charged pack), so most of the run is near the rating.
- Put a **100 to 470 microfarad capacitor** across `VM` and `GND` right at the driver (the stripe on the capacitor marks the negative leg, which goes to `GND`). Motors cause voltage dips that can reset the ESP32; this is cheap insurance.

**The contest allows one start switch for everything.** It must be in the battery + line ahead of both the motor driver and the ESP32, and reachable by the judge without reaching under the robot.

## E. Before you connect the battery

With the battery disconnected, use a multimeter on the beep (continuity) setting:

1. Battery + wire to battery - wire: must **not** beep. If it beeps, there is a short.
2. `VM` to `GND` on the driver: must **not** beep.
3. ESP32 3.3 V pin to `GND`: must **not** beep.
4. ESP32 DC input + to `GND`: must **not** beep.
5. `GND` on the ESP32, the driver and the MPU-6050: must all beep to each other.
6. Look at each wire once more against the tables above.

## F. First power-up, in order

1. Flash the **`esp32dev_bench`** environment over USB with **no battery** connected. Open the serial monitor at 115200. You should see `MPU-6050 ok (WHO_AM_I 0x68)`. A `WHO_AM_I` of 0x70, 0x72 or 0x98 is also fine (common clone chips).
2. Lift the robot so the tracks are off the table. Leave USB connected for the serial monitor, and connect the battery to the **motor driver only** (`VM` and `GND`), not to the ESP32's DC input. Flip the start switch.
3. Send `l`: only the left track should turn, **forward**. Send `x` to stop. Send `r`: only the right track, **forward**. A track that turns backward gets its `invert` flipped in `config.h`.
4. Continue with the checklist at the top of `src/main.cpp` (sensor directions, then the speed measurement).

## G. If something is wrong

| What you see | Most likely cause |
|--------------|-------------------|
| `MPU-6050 not responding`, bus scan finds nothing | `SDA`/`SCL` swapped, no `GND`, or `VCC` not connected |
| Bus scan finds `0x69` | `AD0` is tied high; set `MPU_I2C_ADDR` to `0x69` in `config.h` |
| Everything prints fine but the motors never move | `STBY` not connected to GPIO4, `VM` has no power, or no shared `GND` |
| One motor works, the other does not | the pins for that channel in section A, or that motor's two wires |
| Robot spins in place when told to go straight | one motor needs its `invert` flipped |
| Motors ignore commands, or move erratically | driver `VCC` is on 5 V: it must be 3.3 V |
| ESP32 resets when the motors start or the pack is low | voltage dip below the board's 6 V minimum: charge the cells, add the capacitor, use thicker/shorter battery wires |
| Driver board gets very hot or the robot stops mid-climb | the motors are drawing more than the board allows (**1.2 A continuous, 3.2 A peak per channel**). Stall current is being measured on the bench; compare it to those limits |
