/*?? Hardware Features
Microcontroller: Adafruit Feather nRF52840

Motor Driver: DRV8874 (controlled via IN1/IN2 pins)

DAC: MCP4725 used to set motor current limit (via ILIM pin)

Status LED: Connected to pin 13, used for connection status feedback

?? BLE Functionality
BLE UART Service:

Receives integer speed/direction values from the controller (range -512 to 512).

Data is interpreted to rotate the motor (direction + PWM level).

Custom Configuration BLE Service (UUID: 0x1235)
Includes the following writable/readable BLE characteristics:

Name	UUID	Type	Description
charIlim	6E400020-B5A3-F393-E0A9-E50E24DCCA99	int	Sets motor current limit (mA)
charTargetLock	6E400021-B5A3-F393-E0A9-E50E24DCCA99	bool	Enables or disables Target Lock

These characteristics are visible in the nRF Connect app with custom labels like �Current Limit mA� and �Target Lock Mode�.

?? Motor Control
Accepts signed speed commands over BLE and:

Sets PWM on IN1 or IN2 pin for direction and speed.

Halts motor if command is 0.

PWM level is mapped from received command values:

0 to +512 ? forward

0 to -512 ? reverse

?? Safety & Power Management
Timeout Fail-Safe: If no valid BLE command is received within 2 seconds, the motor is stopped automatically.

Connection LED Behavior:

Blinks for 250 ms every 3 seconds while connected.

Off when disconnected.

?? Current Limit Control via MCP4725 DAC
Accepts BLE ilim_mA value and maps it to a DAC output voltage.

Used to dynamically control the DRV8874's overcurrent threshold.

Default limit is set to 1300 mA.


✅ The receiver rotator code has been updated to act as a BLE Central that:

Scans for devices with names starting with "BLE_Controller_Pedal_"

Automatically connects to them (up to 4)

Receives and applies motor control commands from each connected pedal

Let me know when you’re ready to test, refine input priority logic, or extend the config features.
*/

