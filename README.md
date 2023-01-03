# RunCPM_RPi_Pico
RunCPM for the Challenger RP2040 SD/RTC board

<img src="https://usercontent.one/wp/ilabs.se/wp-content/uploads/2022/10/iso-1-2048.jpg" alt="Challenger_RP2040_SD_RTC" width="1024"/>
<img src="https://github.com/guidol70/RunCPM_RPi_Pico/raw/main/more_pictures/GL20221026_RP2040_262.jpg" alt="RunCPM_Pico_BootUpScreen"/>

Is using much of the RunCPM-Code for an Arduino-DUE (also HostOS 0x01 from the .ino)

does need

- RP2040 Hardware-/Board Support https://github.com/earlephilhower/arduino-pico

RunCPM for the Challenger board must be compiled in the Arduino-IDE up to 250Mhz<br/>
With 275Mhz or 300Mhz RunCPM does not start up.

```
34.78% speedup when you compile with -O3 option (at 250Mhz)
around 6.4 times faster - 25.6Mhz - 
than a Z80 with 4Mhz (Philips P2500 Z80@4MHz) :
```

### If you want to use the ESP8266SdFat of the RP2040 Arduino-Core
and not the (maybe) installed original SdFat-Library from Greiman:<br>
Replace #include <SdFat.h> with include <ESP8266SdFat.h> in your .ino<br>
and create the ESP8266SdFat.h in the following path<br>
```
C:\Users\[yourUser]\AppData\Local\Arduino15\packages\rp2040\hardware\rp2040\2.5.2\libraries\ESP8266SdFat\src
```
with the content
```
#include "SdFat.h"
```


### get rid / avoid the most compiler-warnings:

In

```
C:\Users\[user]\AppData\Local\Arduino15\packages\rp2040\hardware\rp2040\2.5.4\platform.txt
```
add in the top of the file where the compiler-warning-flags-lines are

```
compiler.cpp_warning_flags=-Wno-register -Werror=return-type
```
<br>
and change the compiler.cpp.flags line to

```
compiler.cpp.flags=-c {compiler.cpp_warning_flags} {compiler.defines} {compiler.flags} -MMD {compiler.includes} {build.flags.rtti} -std=gnu++17 -g -pipe
```
<br>
<br>

```
In
C:\Users\guido\Documents\Arduino\libraries\SdFat\src\SDFat.h
(to find the file replace guido with your username )
comment out the warning (because we use File32 instead)
```
![RunCPM_Pico_has_filename](https://github.com/guidol70/RunCPM_RPi_Pico/raw/main/more_pictures/SdFat_h_changes.jpg?raw=true)

### see also (in german):<br>
https://forum.classic-computing.de/forum/index.php?thread/25805-runcpm-auf-dem-raspberry-pi-pico<br>
