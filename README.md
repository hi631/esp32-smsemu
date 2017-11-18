# What is this?

This is a port of [esp31-sms](https://github.com/espressif/esp31-smsemu) to the Espressif ESP32 chip.
I was allowed to use this input / output[esp32-nesemu ](https://github.com/espressif/esp32-nesemu)

The original one is port of [SMSPlus](http://www.bannister.org/software/sms.htm) to the Espressif ESP31 chip. 
It will run most Sega Master System roms at full speed with video disabled. 
(However, because of the low speed of the SPI LCD used and the overhead of pushing pixels to it, the framerate is limited to 15-20FPS for now.)

# What do I need to use this?

You will need:
* A board containg an ESP32 chip and at least 4MB (32Mbit) of SPI flash, plus the tools to program it.
* A backup of a Sega Master System ROM game cartridge
* A 320x240 ILI9341 display, controllable by a 4-wire SPI interface. You can find modules with this LCD by
looking for '2.2 inch SPI 320 240 ILI9341' on eBay or other shopping sites. We used the one with the red PCB and the
SD-card on the bottom.
* Optional, but highly recommended: A Playstation 1 (PSX) or Playstation 2 controller to actually play the game

# How do I hoop up my board?

**LCD:**

    =====  =======================
    Pin    GPIO
    =====  =======================
    MISO   25
    MOSI   23
    CLK    19
    CS     22
    DC     21
    RST    18
    BCKL   5
    =====  =======================

(BCKL = backlight enable)

(Make sure to also wire up the backlight and power pins.)

**Sound(Analog Out)**

    =====  =====
    Pin    GPIO
    =====  =====
    SOUND  26
    =====  =====

(Obviously, also hook up the power pins and connect the sound output to an amp or headphones or so.)

**PSX/PS2 controller**

    =====  =====
    Pin    GPIO
    =====  =====
    CLK    14
    DAT    27
    ATT    16
    CMD    2
    =====  =====

Also connect the power and ground lines. Most PS1/PS2 controllers work fine from a 3.3V power supply, if a 5V one is unavailable.

# How do I program the chip?

Using a tool capable of flashing the SPI flash connected to the ESP32, program the following files to the following
addresses:
esptool.py --chip esp32 --port "/dev/ttyxx" --baud $((921600)) write_flash -fs 4MB 0x140000 xxxx.sms

**Flash**

    =====  =====
    nvs,      data, nvs,     0x9000,   0x6000
    phy_init, data, phy,     0xf000,   0x1000
    factory,  app,  factory, 0x10000,  0x0E0000
    smsgame,  0x40, 0x01,    0x140000, 0x100000 (SMS game ROM image)-> 0x140000
    =====  =====

Because of copyright reasons, you will have to supply the game rom image yourself.


# License/legal

SMSPlus is licensed under the GPL2. The Espressif support code is licensed under the MIT license. All 
trademarks, service marks, trade names and product names appearing in these files are the property of 
their respective owner(s).