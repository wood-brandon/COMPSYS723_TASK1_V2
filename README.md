## COMPSYS723 ASSIGNMENT 1

The following is our COMPSYS723 FreeRTOS project for assignment 1. Using the nios2.sopcinfo file and Assignment_1_v3.elf file the board can be programmed. This can be run and built in a new project by copying over "main.c", "global.h" and "system.h"(altera timer is used).

Instructions:

The FPGA can be programmed with the nios2.spocinfo and enclosed .elf file in the usual way. All of the required information will be displayed through the VGA monitor, and can be interacted with through the PS2 keyboard, SW0 to SW4 and KEY1.

* KEY1 push button will toggle maintenance mode, causing the system to stop managing loads when on. This can be observed that the green LEDs turn off. A visual representation of maintenance mode is shown on the VGA monitor, the square below “System Uptime”. If it’s grey the system is not in maintenance and if it’s green the system is in maintenance mode. 

*	SW0 to SW4 control the loads. The load assigned to SW4 is deemed the highest priority, with SW0 the lowest.

*	The PS2 keyboard can be used to adjust both frequency threshold and rate of change threshold for detecting unstable conditions. The up and down arrows increase/decrease the rate of change threshold respectively, and the right and left arrows increase/decrease the frequency threshold respectively. Pressing the ESC key will reset these two values to their default values (10Hz/t for ROC and 48.5Hz for frequency max).
