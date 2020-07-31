# Bending Lights
## The idea
Ford Mondeo mk4 (manufactured from 2007 to 2014) higher-level trims have headlights with incorporated Static Bending Lamps. These are additional 55W H1 lamps located in headlight housing closer to the radiator grille.  
These light up when Low Beam is on and the steering wheel is turned more than 30 degrees in order to improve visibility for the driver when making turns on unlit roads.  

The original setup includes Headlamp Control Module (HCM, which actually controls Static Bending Lamps) based on infomation from CAN bus regarding speed, steering wheel position, Reverse gear state, Low Beam state.  
To work properly, the HCM requires quite a bit of additional wiring as well as two axle height sensors for automatic Low Beam HID light leveling.  
I could not obtain HCM, sensors, and the wiring looms but I got hold of the headlights with Static Bending Lamps. I decided that this feature should not be wasted, so I've created my own Static Bending Lamps controller.

## Information
CAN-bus messages are relevant for Ford Mondeo mk4 facelift (from 10.2010 to 2014).  

Requires additional wiring installed to the bending lamps in headlight units as well as to the fabricated device. Please engineer the required schematic and wire gauges yourself.  

## Libraries used
- [Seeed Studio CAN Bus Shield - MCP2515 & MCP2551](https://github.com/Seeed-Studio/CAN_BUS_Shield)

## CAN bus messages description
MS-CAN is used.
### Steering wheel angle and wheels direction
MS-CAN ID `0x480`  
- `WHEELS_DIRECTION` (which side the wheels are actually turned to) = **byte 06 bit 0** (global bit position 48 in CAN message), 1=right, 0=left  

- `SW_ANGLE` (steering wheel angle) = starts at **byte 06 bit 1, length 15 bits** (global start bit position 49 in CAN message), multiply value by `0,04395` to get degrees. Value always positive.

### Low Beam and Reverse gear
MS-CAN ID `0x433`  
- `LIGHTS` (low beam on) = **byte 03 bit 0** (global bit position 24 in CAN message), 1=enabled, 0=disabled  

- `REVERSE` (reverse gear engaged) = **byte 03 bit 6** (global bit position 30 in CAN message), 1=engaged, 0=disengaged

### Vehicle speed
MS-CAN ID `0x08b`  
- `VSS` (vehicle speed) = starts at **byte 01 bit 0, length 16 bits** (global start bit position 8 in CAN message), divide value by 100 to get km/h.

## Hardware
- 1 x Arduino Nano
- 1 x MCP2515 CAN bus shield with 8MHz crystal
- 2 x [RobotDyn Transistor DC relay 24V/30A](https://robotdyn.com/transistor-mosfet-dc-switch-relay-5v-logic-dc-24v-30a.html) (not sure it could handle 30A current though, but we need up to 5A max anyway)
- 1 x [HW411 3A DC-DC converter](https://arduino.ua/prod650-DC-DC-ponijaushhii-konverter-c-4-5-60V-do-3-35V) to power Arduino itself
- [Generic case](https://arduino.ua/prod3207-korpys-plastikovii-dlya-elektroniki-d110a-komplekt-36x92x110mm)

## Testing setup
- CANHacker v2.00.01
- second Arduino Nano with same MCP2515 CAN bus shield (used as USB-Serial to CAN bus interface, to play back recorded CAN bus traffic)
- sketch for the second Arduino: [CanHacker (lawicel) CAN adapter on Arduino + MCP2515](https://github.com/autowp/arduino-canhacker)

## Disclaimer
I won't be held responsible for any damage to your car and/or yourself if you decide to use this information.  
Anything in this repository is provided for information only.  
