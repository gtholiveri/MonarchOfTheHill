# Wiring guide

For: ESP32, **SparkFun Motor Driver - Dual TB6612FNG**, **2 motors** (one per side), **MPU-6050** gyro/accelerometer.

How to confirm you have this board: the chip on it is printed `TB6612FNG`, and the pins are labeled `VM VCC GND STBY AIN1 AIN2 PWMA BIN1 BIN2 PWMB A01 A02 B01 B02`. If your board looks different (for example a Qwiic cable connector), stop and tell us before wiring.

Every pin number below matches `src/config.h`. If you wire something differently, change it there too.

## The big picture

```
             start switch
 Battery + ----/ ----------+---------------------> VM   (motor driver)
                           |
                           +---> ESP32 power (section D)

 Battery - ------------------+--------------------> GND  (motor driver)
                              +-------------------> GND  (ESP32)
                              +-------------------> GND  (MPU-6050)

 ESP32 3V3 --------------------+-------------------> VCC  (motor driver)
                                +------------------> VCC  (MPU-6050)
```

Three rules:

1. **Everything shares one ground.** Battery -, ESP32, motor driver and MPU-6050 grounds are all connected. Most "nothing works" problems are a missing ground.
2. **Never put battery voltage on the ESP32 3V3 pin or the MPU-6050 VCC.** Those get 3.3 V from the ESP32 only. Battery voltage goes only to `VM` (and to the ESP32 through a regulator, section D).
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
| 3V3 | VCC | powers the driver's logic |
| GND | GND | ground |

The motor driver has several `GND` pins. They are all the same connection; use any.

ESP32 boards print pin names differently: GPIO25 may be marked `25`, `D25` or `IO25`. They all mean the same pin.

## B. ESP32 to MPU-6050

| ESP32 pin | MPU-6050 pin |
|-----------|--------------|
| 3V3 | VCC |
| GND | GND |
| GPIO21 | SDA |
| GPIO22 | SCL |

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

- `VM` accepts roughly 2.5 V to 13.5 V. A pack of two 18650 cells in series is about 7.4 V (8.4 V fully charged), which is fine for the board.
- At full command, the motors see the full battery voltage. If your motors are rated for less than your battery voltage, tell us and we will add a speed cap.
- Put a **100 to 470 microfarad capacitor** across `VM` and `GND` right at the driver (the stripe on the capacitor marks the negative leg, which goes to `GND`). Motors cause voltage dips that can reset the ESP32; this is cheap insurance.

**ESP32 power.** Pick one:

1. *ESP32 stays on the ACEBOTT shield.* The shield already powers it from the battery. Do not use the shield's `M1` to `M4` motor terminals; the new driver replaces them. Take `VM` and `GND` from the shield's **switched** battery voltage (after its power switch). Use a multimeter to find that point: it reads about 7 to 8 V with the switch on and 0 V with it off. Because it sits behind that switch, the shield's switch is your single start switch. Also check that GPIO 25, 26, 27, 32, 33, 13 and 4 are reachable on the shield's headers and not already used for something else; if one is not, pick a free pin and change `config.h`.
2. *No shield.* Power the ESP32's `VIN` or `5V` pin from a **5 V buck converter** fed by the battery. Set the converter to 5.0 V with a multimeter **before** connecting it to the ESP32.

**The contest allows one start switch for everything.** It must be in the battery + line ahead of both the motor driver and the ESP32, and reachable by the judge without reaching under the robot.

## E. Before you connect the battery

With the battery disconnected, use a multimeter on the beep (continuity) setting:

1. Battery + wire to battery - wire: must **not** beep. If it beeps, there is a short.
2. `VM` to `GND` on the driver: must **not** beep.
3. ESP32 `3V3` to `GND`: must **not** beep.
4. `GND` on the ESP32, the driver and the MPU-6050: must all beep to each other.
5. Look at each wire once more against the tables above.

## F. First power-up, in order

1. Flash the **`esp32dev_bench`** environment over USB with **no battery** connected. Open the serial monitor at 115200. You should see `MPU-6050 ok (WHO_AM_I 0x68)`. A `WHO_AM_I` of 0x70, 0x72 or 0x98 is also fine (common clone chips).
2. Lift the robot so the tracks are off the table. Connect the battery and flip the start switch.
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
| ESP32 resets when the motors start | voltage dip: add the capacitor, use thicker/shorter battery wires, or power the ESP32 from its own regulator |
| Driver board gets very hot or the robot stops mid-climb | the motors draw more than the board allows: **1.2 A continuous, 3.2 A peak per channel**. Check the motor's *stall* current on its datasheet; on a steep ramp the motors get close to stalling |
