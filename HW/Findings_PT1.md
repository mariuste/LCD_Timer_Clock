Findings

PCB_TOP
- variant problem in BOM (R502) -> [Issue #1](https://github.com/mariuste/LCD_Timer_Clock/issues/1)
- BTN LED resistor missing (here or on PCB-BOT); yellow 2k and green 16k -> [Issue #2](https://github.com/mariuste/LCD_Timer_Clock/issues/2)
- Identifier in solder mask wrong (top - bot) -> [Issue #3](https://github.com/mariuste/LCD_Timer_Clock/issues/3)
- Export File name missing "TOP" -> [Issue #4](https://github.com/mariuste/LCD_Timer_Clock/issues/4)
- IC503, IC504, IC505 EP is LED pin, not GND -> [Issue #5](https://github.com/mariuste/LCD_Timer_Clock/issues/5)
- add vias to LEDs to transport heat (full connection) -> [Issue #6](https://github.com/mariuste/LCD_Timer_Clock/issues/6)
- add heat sink from bottom -> [Issue #7](https://github.com/mariuste/LCD_Timer_Clock/issues/7)
- LCD_Keypad_PWM not connected -> bridge with BG PWM -> [Issue #8](https://github.com/mariuste/LCD_Timer_Clock/issues/8)
- add cover to BTN LEDs (too bright), maybe drill holes for mounting something -> [Issue #9](https://github.com/mariuste/LCD_Timer_Clock/issues/9)
- rename Set LED to Time_Date LED and other signals ("T" missing) -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- schematic net name timer / quicksetting not consistent with LEDs -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- Timer 1 LED is connected to signal Timer 2 and vice versa (for now fixed in sw) -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- LCD VDD too low, above 3V3 working -> use VBAT, mind voltage levels -> [Issue #11](https://github.com/mariuste/LCD_Timer_Clock/issues/11)
- LED not yellow enugh, replace with warmer color, e.g. XTEAWT-00-0000-00000LAEATR-ND or 475-GWQSSPA1.EM-LBLH-XX58-1-350-R18TR-ND
- adjust max current for new LEDs
- rename signal "nSW_QUICKSETTING_T" to "nSW_TIMER2_T"
- rename signal "nSW_TIMER_T" to "nSW_TIMER1_T"
- rename signal "nSW_SET_" to "nSW_TIME_DATE_T"

- don't use low temp solder (LEDs >= 100°C)

#7

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
- Add pinheader fpr swd for standard connection
- Idle current consumption
  - 5mA at 64 MHz
  - 4.0mA at 4MHz (higher than expected)
  - 1.3mA at 1MHz (higher than expected)
- add 0R to en of IC203 to analyze current
- Export File name missing BOT
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
- add high side switch between VBUS and D401 to be able to power LED from VBAT even if a 0.9A USB power supply is attached. E.g. control High side with  "nUSB_PD_OK" this way monitoring this signal is not necessary any more.
- rename signal "nSW_QUICKSETTING" to "nSW_TIMER2"
- rename signal "nSW_TIMER" to "nSW_TIMER1"
- rename signal "nSW_SET" to "nSW_TIME_DATE"
- EEPROM text schematic: "Address: 0xA0"

MECHANICS
- Fix bearings
- add speaker Mount

COMMENTS
- use BAT down to 3.4V (lcd gets less and less visible)
- when nUSB_PD_OK = high, consider power budget to be 900 mA; charging max is 400 mA, therefore LEDs should be in sum <= 900mA - 400 mA (charging) - 300mA (DFPlayer) - 20mA (remaining circuit) = 180 mA (add some tolerance). Consider sum of main LEDs as well as background lighting LEDs -> see PCB_Bot "add high side switch between VBUS and D401" to eliminate this requirement. LEDs can be driven at full power at any time this this change. I assume the ouput of IC201 is limited to 400 mA at any time.
