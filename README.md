#Required tools#
- gcc
- kernel-devel
- avrdude
- avr-gcc
- avr-libc

#Building#

##Kernel module##
To build, make sure you have gcc and the kernel headers installed, then run

    make all

then run

    make reload

to load the module into the kernel, you will be prompted for a sudo password.

##Microcontroler##

To build, make sure you have gcc, avr-gcc, avr-libc, and avrdude installed, then, in the 32u4 directory, run

    make all

then run 

    make program <port>

to push the code to the microcontroller on that port, typically /dev/ttyACM0. The eeprom must also be initialized by running 

    make initialize <port>

eeprom initialization should only be done when first setting up the microcontroller or when default settings have been changed, repeated calls will cause unnecessarily wear.
