# lazy-passwords
Arduino based keyboard/mouse emulator to block screen saver and store/enter passwords

lazy-password performs 2 functions:
* It tries to stop the usual corporate screen saver from locking your session.
* It let you enter quickly pre recorded passwords as they would be typed from a keyboard.

The number of passwords is set to 5.

## Hardware
You need:
1. Arduino leonardo, or similar (atmega32u4).
2. Rotary encoder, ky-040
3. 1602 based LCD (1602 Arduino LCD Keypad Shield)
4. Buzzer (optional)

## Operation
On start, the device is protected. In order to unlock it, a user has to enter a pin consisting 
of five hex numbers. Default pin is FF FF FF FF 00 and can be changed once unlocked.
Once unlocked, mouse calibration is activated. Observe mouse movement on the screen.
Use rotary to adjust it. Make sure there are no missed steps. Calibration is completed automaticaly
after 5 seconds of inactivity, or by pressing the rotary key.

In normal operation, rotary can be used to select a user name. Pressing the rotary key
triggers corresponding password entry.
Every minute a small mouse movement and shift key press are generated by a device.

When CAPS LOCK light is on, the buzzer is activated. It is possible to mute buzzer by pressing any of
the LCD Keyoad Shield.

## Password Update
In order to record password, you have to first unlock lazy-password.
Next you send a password update command using serial interface.

Prepare a command file containing: ```p<password_idx>:<password>\n\0```
         
For example password000.txt which sets first password to password000:

```
0000000   p   0   :   p   a   s   s   w   o   r   d   0   0   0  \n  \0
         70  30  3a  70  61  73  73  77  6f  72  64  30  30  30  0a  00
```
and execute on command line:

```
sudo sh -c 'cat password000.txt > /dev/ttyACM0'
```
The maximum length of a password is 48 characters.
The ```test``` directory contains numerous examples of files containing password and label update commands.


## Label (username) Update
Similar to password update, label update involves sending a command over a serial interface.

Command file contains: ```l<label_idx>:<label>\0```

For example label000.txt which sets first label to label000:

```
0000000   l   0   :   l   a   b   e   l   0   0   0  \0
         6c  30  3a  6c  61  62  65  6c  30  30  30  00
```
The maximum size of a label is 16 (length of the lcd display).
The ```test``` directory contains numerous examples of files containing password and label update commands.


## Keyboard
lazy-password includes support for 2 keyboard layouts, US and Swiss German. The keyboard layout can
be switched as needed. The default keyboard layout is preserved in EEPROM.

## Building
Standard Keyboard and HID require modifications to support CapsLock tracking and non-US keyboard layout.
You have to apply HID and Keyboard patches before sketch can be compiled and uploaded.

## Security consideration:
The passwords are stored unprotected in EEPROM and can be dumped using standard atmega tools.
However, with some effort you can protect access to EEPROM.

First of all you need an ISP programmer. You can either buy one or [build one](https://www.arduino.cc/en/Tutorial/ArduinoISP) using spare Arduino.

Please note, that default control pins of LCD module overlap with ISP. During programming LCD module must be disconnected.

Once communication to lazy-password via ISP is established, it is time to do the real work.

#### Check the fuses
```
sudo avrdude -patmega32u4 -c avrisp  -P/dev/ttyUSB0 -b19200 -t

avrdude: AVR device initialized and ready to accept instructions

Reading | ################################################## | 100% 0.02s

avrdude: Device signature = 0x1e9587 (probably m32u4)
avrdude> d lfuse
>>> d lfuse 
0000  ff                                                |.               |

avrdude> d hfuse
>>> d hfuse 
0000  d8                                                |.               |

avrdude> d efuse
>>> d efuse 
0000  cb                                                |.               |

avrdude> d lock
>>> d lock 
0000  2f                                                |?               |

avrdude> quit
>>> quit 

avrdude: safemode: Fuses OK (E:CB, H:D8, L:FF)

avrdude done.  Thank you.
```
The board I have is clone, so some values can be different. The most important bit to check is EESAVE in hfuse. It has to be inactive (1). 


#### lock access to flash and EEPROM

```
sudo avrdude -patmega32u4 -c avrisp  -P/dev/ttyUSB0 -b19200 -t

avrdude: AVR device initialized and ready to accept instructions

Reading | ################################################## | 100% 0.02s

avrdude: Device signature = 0x1e9587 (probably m32u4)
avrdude> d lock
>>> d lock 
0000  2f                                                |?               |

avrdude> w lock 0 0x2c
>>> w lock 0 0x2c 

avrdude> d lock   
>>> d lock 
0000  2c                                                |,               |

avrdude> quit
>>> quit 

avrdude: safemode: Fuses OK (E:CB, H:D8, L:FF)

avrdude done.  Thank you.
```

At this moment, you can no longer update anything using Arduino IDE.


#### restore unprotected operation
The only way to reverse conent of the lock register is to issue chip erase command. The chip erase command wipes out content of flash and EEPROM (if the EESAVE bit is inactive).

```
sudo avrdude -patmega32u4 -c avrisp -P/dev/ttyUSB0 -b19200 -t

avrdude: AVR device initialized and ready to accept instructions

Reading | ################################################## | 100% 0.02s

avrdude: Device signature = 0x1e9587 (probably m32u4)
avrdude> erase
>>> erase 
avrdude: erasing chip
avrdude> quit
>>> quit 

avrdude: safemode: Fuses OK (E:CB, H:D8, L:FF)

avrdude done.  Thank you.
```

And the last step to recover the original Arduino functionaliy. Start Arduino IDE and:
```
Select Tools => Programmer: Arduino as ISP
Select Tools => Burn Bootloader
```
