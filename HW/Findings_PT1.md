Findings

PCB_TOP
- variant problem in BOM (R502)
- BTN LED resistor missing (here or on PCB-BOT)
- Identifier in solder mask wrong (top - bot)
- Export File name missing "TOP"
- IC503, IC504, IC505 EP is LED pin, not GND
- don't use low temp solder (LEDs >= 100°C)
- add vias to LEDs to transport heat (full connection)
- add heat sink from bottom
- LCD_Keypad_PWM not connected -> bridge with BG PWM
- add cover to BTN LEDs (too bright), maybe drill holes for mounting something

PCB_BOT
- silk of LEDs swapped (standby and charge)
- IC302 silk pin1 not visible
- IC400 assembly pin1 missing
- SW302 wrong type (resistor)
- assembly plan not cleaned up
- stencil cutouts for bat connectors (keepout holes)
- Identifier in solder mask wrong (top - bot)
- J200 +/- missing
- J200 add some clearance to J400
- 3V0 Ref not properly working at low bat voltages -> use VBUS anyway
- DFP Audio en pull-down
- Keepout for programmer backplate
- Move SWD to clear gear
- Idle current consumption
  - 5mA at 64 MHz
  - 4.0mA at 4MHz (higher than expected)
  - 1.3mA at 1MHz (higher than expected)
- add 0R to en of IC203 to analyze current
- Export File name mitting BOT
- Silk of Battery much larger symbol
- LCD_Keypad_PWM not connected
- maybe add debug LED to indicate interrups
- DS300 pin 1 marking missing

MECHANICS
- Fix bearings
