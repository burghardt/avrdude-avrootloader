This patch implements support for Hagen Reddmanns bootloader into
avrdude. Currently, writing to EEPROM, encryption and versioning
is not supported.

This is an example of how to flash something using this bootloader:

./avrdude -C avrdude.conf -c avrootloader -P /dev/ttyUSB0 -p m32 \
-U flash:w:../OilquickBagger.hex \
-x key=BOOTLOADER -x trig=MIS Bootloader

The option -x key=BOOTLOADER sets the helo message to start
the bootloader. It can be seen as a hidden key neccessary to gain
bootloader functionality. However, remember this can easily be sniffed.

The option -x trig=MIS Bootloader is what is to be expected from
the bootloader when everything is fine and we gained bootloader
access.

If these options with their according parameter are not used, the
default values as per bootloader version 6 are used.
