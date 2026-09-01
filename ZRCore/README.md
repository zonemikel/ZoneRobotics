# ZRCore — motion + power board

The big board. Handles motor and servo output, power management (IP5306 charging, dual 18650 battery bay) and the I²C connection used to expand the platform.

## What lives here

Datasheets, pinouts, board renders, mechanical drawings, media, and any expansion notes.

## Key hardware

- **Control:** PCA9685 PWM (16 channels, 12-bit) for servos, LEDs and actuators.
- **Motion:** DRV8833 dual H-bridge for two DC motors.
- **Expansion:** MCP23008 8-bit GPIO over I²C, plus multiple I²C connectors.
- **Power:** IP5306 power path, dual 18650 battery bay for 5+ hours of runtime.

## Product page

<https://home.zonerobotics.com/products/zrcore>

## Pairs with ZRSense

ZRCore + [ZRSense](../ZRSense/) over I²C is the full modular hardware stack.
