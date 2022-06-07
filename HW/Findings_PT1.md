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
- replace R502, R505 with poti (to be tested)
- rename Set LED to Time_Date LED 
- schematic net name timer / quicksetting not consistent with LEDs
- Timer 1 LED is connected to signal Timer 2 and vice versa (for now fixed in sw)
- LCD VDD too low, above 3V3 working -> use VBAT, mind voltage levels
- LED not yellow enugh, replace with warmer color, e.g. XTEAWT-00-0000-00000LAEATR-ND or 475-GWQSSPA1.EM-LBLH-XX58-1-350-R18TR-ND
- adjust max current for new LEDs

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
- Battery connectors too far apart, check distance
- flip programmer to bottom side
- DFPlayer is flipped in two axis
- speaker is way too weak (lloks like ear piece), replace with stronger speaker
- also provide line out and bat out for external amplifier or add PAM 8403 plus elko
- add / edit I2C adress of RTC in schematic (0xA4)
- rename Set LED to Time_Date LED 
- comment of EEPROM im schematic is not helfful -> change to AT34C04

MECHANICS
- Fix bearings
- add speaker Mount

COMMENTS
- use BAT down to 3.4V (lcd gets less and less visible)
