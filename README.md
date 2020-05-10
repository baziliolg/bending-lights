# Bending Lights

## Information
TODO

## Libraries used
- [Seeed Studio CAN Bus Shield - MCP2515 & MCP2551](https://github.com/Seeed-Studio/CAN_BUS_Shield)

## CAN bus messages description
### Steering wheel angle and rotation
CAN ID `0x076`  
- `WHEELS_DIRECTION` (which side the wheels are actually turned to) = **byte 00 bit 1** (global bit position 1 in CAN message), 1=right, 0=left  
- `SW_ROTATION` (the direction where the steering wheel is being turned to right now) = **byte 02 bit 0** (global bit position 16 in CAN message), 1=right, 0=left  
- `SW_ANGLE` (steering wheel angle) = **byte 06 bit 2, length 14 bits** (global bit position 50 in CAN message), multiply value by `0,04395` to get degrees. Value always positive.

### Vehicle speed
TODO

### Current gear
TODO


## Hardware 
- Arduino Nano
- MCP2515 CAN bus shield with 8MHz crystal
- TODO

## Testing setup
TODO
- CANHacker v2.00.01
- second Arduino Nano with MCP2515 CAN bus shield (used as USB-Serial to CAN bus interface, to play back recorded CAN bus traffic)
- sketch for the second Arduino: [CanHacker (lawicel) CAN adapter on Arduino + MCP2515](https://github.com/autowp/arduino-canhacker)
