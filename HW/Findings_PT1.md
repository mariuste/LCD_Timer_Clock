Findings

PCB_TOP
- variant problem in BOM (R502) -> [Issue #1](https://github.com/mariuste/LCD_Timer_Clock/issues/1)
- BTN LED resistor missing (here or on PCB-BOT); yellow 2k and green 16k -> [Issue #2](https://github.com/mariuste/LCD_Timer_Clock/issues/2)
- Identifier in solder mask wrong (top - bot) -> [Issue #3](https://github.com/mariuste/LCD_Timer_Clock/issues/3)
- Export File name missing "TOP" -> [Issue #4](https://github.com/mariuste/LCD_Timer_Clock/issues/4)
- IC503, IC504, IC505 EP is LED pin, not GND -> [Issue #5](https://github.com/mariuste/LCD_Timer_Clock/issues/5)
- add vias to LEDs to transport heat (full connection) -> [Issue #7](https://github.com/mariuste/LCD_Timer_Clock/issues/7)
- add heat sink from bottom -> [Issue #7](https://github.com/mariuste/LCD_Timer_Clock/issues/7)
- LCD_Keypad_PWM not connected -> bridge with BG PWM -> [Issue #8](https://github.com/mariuste/LCD_Timer_Clock/issues/8)
- add cover to BTN LEDs (too bright), maybe drill holes for mounting something -> [Issue #9](https://github.com/mariuste/LCD_Timer_Clock/issues/9)
- rename Set LED to Time_Date LED and other signals ("T" missing) -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- schematic net name timer / quicksetting not consistent with LEDs -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- Timer 1 LED is connected to signal Timer 2 and vice versa (for now fixed in sw) -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- LCD VDD too low, above 3V3 working -> use VBAT, mind voltage levels -> [Issue #11](https://github.com/mariuste/LCD_Timer_Clock/issues/11)
- LED not yellow enugh, replace with warmer color, e.g. XTEAWT-00-0000-00000LAEATR-ND or 475-GWQSSPA1.EM-LBLH-XX58-1-350-R18TR-ND -> [Issue #7](https://github.com/mariuste/LCD_Timer_Clock/issues/7)
- adjust max current for new LEDs -> [Issue #7](https://github.com/mariuste/LCD_Timer_Clock/issues/7)
- rename signal "nSW_QUICKSETTING_T" to "nSW_TIMER2_T" -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- rename signal "nSW_TIMER_T" to "nSW_TIMER1_T" -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)
- rename signal "nSW_SET_" to "nSW_TIME_DATE_T" -> [Issue #10](https://github.com/mariuste/LCD_Timer_Clock/issues/10)

- don't use low temp solder (LEDs >= 100°C)

PCB_BOT
- silk of LEDs swapped (standby and charge) -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- IC302 silk pin1 not visible -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- IC400 assembly pin1 missing -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- SW302 wrong type (resistor) -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- assembly plan not cleaned up -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- stencil cutouts for bat connectors (keepout holes) -> [Issue #13](https://github.com/mariuste/LCD_Timer_Clock/issues/13)
- Identifier in solder mask wrong (top - bot) -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- J200 +/- missing -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- J200 add some clearance to J400 -> [Issue #14](https://github.com/mariuste/LCD_Timer_Clock/issues/14)
- 3V0 Ref not properly working at low bat voltages -> use VBUS anyway -> [Issue #15](https://github.com/mariuste/LCD_Timer_Clock/issues/15)
- DFP Audio en pull-down -> [Issue #16](https://github.com/mariuste/LCD_Timer_Clock/issues/16)
- Keepout for programmer backplate -> [Issue #13](https://github.com/mariuste/LCD_Timer_Clock/issues/13)
- Move SWD to clear gear -> [Issue #14](https://github.com/mariuste/LCD_Timer_Clock/issues/14)
- Add pinheader for swd for standard connection -> [Issue #17](https://github.com/mariuste/LCD_Timer_Clock/issues/17)
- add 0R to en of IC203 to analyze current -> [Issue #17](https://github.com/mariuste/LCD_Timer_Clock/issues/17)
- Export File name missing BOT -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- Silk of Battery much larger symbol -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- LCD_Keypad_PWM not connected -> [Issue #18](https://github.com/mariuste/LCD_Timer_Clock/issues/18)
- DS300 pin 1 marking missing -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- Battery connectors too far apart, check distance -> [Issue #13](https://github.com/mariuste/LCD_Timer_Clock/issues/13)
- flip programmer to bottom side -> [Issue #14](https://github.com/mariuste/LCD_Timer_Clock/issues/14)
- DFPlayer is flipped in two axis -> [Issue #13](https://github.com/mariuste/LCD_Timer_Clock/issues/13)
- speaker is way too weak (lloks like ear piece), replace with stronger speaker -> [Issue #17](https://github.com/mariuste/LCD_Timer_Clock/issues/17)
- add / edit I2C adress of RTC in schematic (0xA4) -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- rename Set LED to Time_Date LED  -> [Issue #19](https://github.com/mariuste/LCD_Timer_Clock/issues/19)
- comment of EEPROM im schematic is not helpfull -> change to AT34C04 -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)
- add high side switch between VBUS and D401 to be able to power LED from VBAT even if a 0.9A USB power supply is attached. E.g. control High side with  "nUSB_PD_OK" this way monitoring this signal is not necessary any more. -> [Issue #17](https://github.com/mariuste/LCD_Timer_Clock/issues/17)
- rename signal "nSW_QUICKSETTING" to "nSW_TIMER2" -> [Issue #19](https://github.com/mariuste/LCD_Timer_Clock/issues/19)
- rename signal "nSW_TIMER" to "nSW_TIMER1" -> [Issue #19](https://github.com/mariuste/LCD_Timer_Clock/issues/19)
- rename signal "nSW_SET" to "nSW_TIME_DATE" -> [Issue #19](https://github.com/mariuste/LCD_Timer_Clock/issues/19)
- EEPROM text schematic: "Address: 0xA0" -> [Issue #12](https://github.com/mariuste/LCD_Timer_Clock/issues/12)

- Idle current consumption
  - 5mA at 64 MHz
  - 4.0mA at 4MHz (higher than expected)
  - 1.3mA at 1MHz (higher than expected)

MECHANICS
- Fix bearings
- add speaker Mount

COMMENTS
- use BAT down to 3.4V (lcd gets less and less visible)
- when nUSB_PD_OK = high, consider power budget to be 900 mA; charging max is 400 mA, therefore LEDs should be in sum <= 900mA - 400 mA (charging) - 300mA (DFPlayer) - 20mA (remaining circuit) = 180 mA (add some tolerance). Consider sum of main LEDs as well as background lighting LEDs -> see PCB_Bot "add high side switch between VBUS and D401" to eliminate this requirement. LEDs can be driven at full power at any time this this change. I assume the ouput of IC201 is limited to 400 mA at any time.
