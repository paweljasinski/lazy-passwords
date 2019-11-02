# lazy-passwords
Arduino based keyboard/mouse emulator to block screen saver and store/enter passwords

You need a arduino leonardo, or similar (atmega32u4).
Rotary encoder, ky-040
1602 based LCD (1602 Arduino LCD Keypad Shield), the keys are not used.


lazy-password performs 2 functions.
It tries to stop the usual corporate screen saver from locking your session.
It let you enter quickly pre recorded passwords as they would be typed from a keybord.

On start, the device is protected. A user has to enter 5 numbers to unlock it.
Once unlocked, mouse calibration is activated. Observe mouse movement on the screen.
Use rotary to adjust it. Make sure there are no missed steps. Calibration is completed automaticaly
after 5 seconds of inactivity, or by pressing the embeded rotary key.

In normal operation, rotary can be used to select a user name. Pressing the rotary key
triggers corresponding password entry.

In order to record password, you have to get lazy-password in normal operation mode.

Prepare a file containing <password_idx>:<password>\0
For example p0.txt:

0000000   0   :   q   u   e   r   y   1   2   3   4   5  \n  \0
         30  3a  71  75  65  72  79  31  32  33  34  35  0a  00

and execute on command line:

sudo sh -c 'cat p0.txt > /dev/ttyACM0'

Security consideration:
The passwords are stored unprotected in EEPROM. EEPROM can be dumped using standard atmega tools.
