# Voron 2.4 - Software Setup (Fluidd)

## Exordium

This manual should assist you to set up the software for your Voron 2.4. It is neither complete nor does it applies for everyone. But since sources are limited this is a collection of findings and testings I have done.
Read the Readme part. Don't jump into this because you can!

### MCU
Manufacturer: Fysetc
Model: Spider 466 V1.0

### Display
Manufacturer: Fysetc
Model: Mini 12864 Panel (RGB)
Version: 2.1

### Processing Unit
Manufacturer: Raspberry Pi Ltd
Model: Raspberry Pi4 - Model B
RAM: 2GB
SD: 16GB

### LED Lightstrip
Manufacturer: BTF-Lightning
Model: WS2812B (IP67)
Lenght: 1 meter
LED count: 60
Color Mode: GRB
Voltage: 5V

## Readme (Warrenty/Warnings)

```
ATTENTION
This config datei is fullfilled mit special electronische instructions.
Fingergrabbing and changing the cnoeppkes from the computers is allowed for die experts only!
So all the "lefthanders" stay away and do not disturben the brainstorming von here working intelligencies.
Otherwise you will be out thrown and kicked anderswhere!
Also: please keep still and only watchen astaunished the blinkenlights.

WARNING
You are operating on high and low voltage. The Fysetc Spider 1.0 board is powered with 110/230V depending on your region. The Author is not responsible if you harm yourself or others. If you are unsure or haven't done aynthing with electronics so far. Get help. This is serious and high voltage can kill you.

Disclaimer of Warranty
COVERED SOFTWARE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS, WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED SOFTWARE IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE COVERED SOFTWARE IS WITH YOU. SHOULD ANY COVERED SOFTWARE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE OF ANY COVERED SOFTWARE IS AUTHORIZED HEREUNDER EXCEPT UNDER THIS DISCLAIMER.
```

## Control

Pi User: pi
FluiddPi Init: raspberry

**Init password - change asap!**

## System Info

### Raspberry

Manufacturer: Raspberry Pi Ltd
Model: Raspberry Pi4 - Model B
RAM: 2GB
SD: 16GB

### Spider Control Board

Manufacturer: Fysetc
Model: Spider 466 V1.0

* Wiki [https://wiki.fysetc.com/Spider/](https://wiki.fysetc.com/Spider/)
* Wiki Wiring [https://wiki.fysetc.com/images/Spider\_v1.0\_wiring.jpg](https://wiki.fysetc.com/images/Spider_v1.0_wiring.jpg)

![Spider V1.0 Wiring](./Helper/pix/Spider_v1.0_wiring.jpg)

# Software (Rasperry PI)

## Installation Tutorial

### Install FluiddPi

[https://3dprintbeginner.com/how-to-install-fluiddpi-on-raspberry-pi/](https://3dprintbeginner.com/how-to-install-fluiddpi-on-raspberry-pi/)

### Update RaspberryPi Boot Settings

By default you can only use USB connection between Spider Board and FluiddPi.
To get UART running (via /dev/ttyAMA0) you need to change two files.
Otherwise you end up with error "unknown" in Fluidd Dashboard.
You cannot use KIAUH flash method via UART as well, same thing. No access to the device.

#### Raspi-Config

1. `sudo raspi-config`
2. Select: 3. Interface Options
3. Select: P6. Serial Port
4. First question NO second question YES
5. `sudo reboot`

#### cmdline.txt

```
sudo nano /boot/cmdline.txt
# Remove console=tty1, more or less everything before root=PARTUUID...
```

Now reboot

#### config.txt

```
sudo nano /boot/config.txt
# Add to the end of the file: dtoverlay=pi3-disable-bt
```

### Install kiauh

```
git clone https://github.com/th33xitus/kiauh.git
cd kiauh
chmod +x kiauh.sh scripts/*
./kiauh
```

### Spider Board V1.0 bootloader

1. Power off Spider Board
2. Set PINs from U5V to 2+3
3. Set Pin between 3.3V and BT0
4. Power via USB cable from Raspberry to Spiderboard
5. SSH -> `wget https://github.com/FYSETC/FYSETC-SPIDER/raw/main/bootloader/Bootloader_FYSETC_SPIDER.hex`
6. Upload -> `objcopy --input-target=ihex --output-target=binary Bootloader_FYSETC_SPIDER.hex Bootloader_FYSETC_SPIDER.bin && dfu-util -a 0 -s 0x08000000:mass-erase:force -D Bootloader_FYSETC_SPIDER.bin`

### Spider Board Firmware

1. Power down
2. Remove Jumper between 3.3 and BT0
3. Change Jumper U5V back to 1+2
4. Use KIAUH with option 4 +3 (Build only)
Extra Low-Level
MicroController: STM32
Processor: STM32466
Bootloader offset: 32KiB
Clock: 12MHz
Comm: USB on PA11/PA12
Q for quit and save
5. Download to you machine
6. Put the klipper.bin to SDCard
7. Rename klipper.bin to firmware.bin
8. SDCard -> SpiderBoard -> Power on
Flash should blink, after 30 secs around press the reboot button on SpiderBoard once
Check if firmware.bin is now called old.bin (flash successfull)

### Fluidd Configuration

You need to connect a thermistor! Otherwise you will run into problems.
If you have a NTC100K 53950 connect it to TE= PC0 like descibed in the image!

You will end up with an error message in the end on Fluidd.
This is ok, since we don't have connected everything right now.
But you should see temperature readings!

#### Printer.cfg

```
# This file contains common pin mappings for the Fysetc Spider board.
# To use this config, the firmware should be compiled for the STM32F446.
# When calling "menuconfig", enable "extra low-level configuration setup"
# and select the "12MHz crystal" as clock reference
# For flashing, write the compiled klipper.bin to memory location 0x08000000

# See docs/Config_Reference.md for a description of parameters.

## Voron Design VORON2 250/300/350mm Spider TMC2209 UART config

## *** THINGS TO CHANGE/CHECK: ***
## MCU paths							[mcu] section
## Thermistor types						[extruder] and [heater_bed] sections - See 'sensor types' list at end of file
## Z Endstop Switch location			[safe_z_home] section
## Homing end position				[gcode_macro G32] section
## Z Endstop Switch  offset for Z0		[stepper_z] section
## Probe points							[quad_gantry_level] section
## Min & Max gantry corner postions		[quad_gantry_level] section
## PID tune								[extruder] and [heater_bed] sections
## Fine tune E steps					[extruder] section

[mcu]
## Uncomment below if you're using the Raspberry uart0 to communicate with Spider
restart_method: command

##  You need to select 'Communication interface' to USB in 'make menuconfig'
##  when you compile Klipper for Spider
##	Obtain definition by "ls -l /dev/serial/by-id/" then unplug to verify
##--------------------------------------------------------------------
#serial: /dev/serial/by-id/usb-Klipper_stm32f446xx_230032000851363131363530-if00
serial: /dev/serial/by-id/usb-Klipper_stm32f446xx_3B002D000750305538333620-if00
##	If you want to use the Raspberry uart0 to communicate with Spider,
##  you need to select 'Communication interface' to 'Serial (on USART1 PA10/PA9)'
##  in 'make menuconfig' when you compile klipper and set the serial as below
##--------------------------------------------------------------------
#serial: /dev/ttyAMA0
##--------------------------------------------------------------------

[printer]
kinematics: corexy
max_velocity: 300
max_accel: 3000			        #Max 4000
max_z_velocity: 15			#Max 15 for 12V TMC Drivers, can increase for 24V
max_z_accel: 350
square_corner_velocity: 5.0

#####################################################################
#      X/Y Stepper Settings
#####################################################################

[stepper_x]
##	Connected to X-MOT (B Motor)
step_pin: PE11
dir_pin: !PE10
enable_pin: !PE9
rotation_distance: 40
microsteps: 16
full_steps_per_rotation:200  #set to 400 for 0.9 degree stepper
endstop_pin: ^PB14
position_min: 0

##--------------------------------------------------------------------

##	Uncomment below for 250mm build
#position_endstop: 250
#position_max: 250

##	Uncomment for 300mm build
#position_endstop: 300
#position_max: 300

##	Uncomment for 350mm build
position_endstop: 350
position_max: 350

##--------------------------------------------------------------------
homing_speed: 25   #Max 100
homing_retract_dist: 5
homing_positive_dir: true

##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 stepper_x]
uart_pin: PE7
interpolate: True
run_current: 0.8
hold_current: 0.7
sense_resistor: 0.110
stealthchop_threshold: 0

[stepper_y]
##	Connected to Y-MOT (A Motor)
step_pin: PD8
dir_pin: !PB12
enable_pin: !PD9
rotation_distance: 40
microsteps: 16
full_steps_per_rotation:200  #set to 400 for 0.9 degree stepper
endstop_pin: ^PB13
position_min: 0
##--------------------------------------------------------------------

##	Uncomment for 250mm build
#position_endstop: 250
#position_max: 250

##	Uncomment for 300mm build
#position_endstop: 300
#position_max: 300

##	Uncomment for 350mm build
position_endstop: 350
position_max: 350

##--------------------------------------------------------------------
homing_speed: 25  #Max 100
homing_retract_dist: 5
homing_positive_dir: true

##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 stepper_y]
uart_pin: PE15
interpolate: True
run_current: 0.8
hold_current: 0.7
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#   Z Stepper Settings
#####################################################################

## In Z-MOT Position
## Z0 Stepper - Front Left
[stepper_z]
step_pin: PD14
dir_pin: !PD13
enable_pin: !PD15
rotation_distance: 40
gear_ratio: 80:16
microsteps: 16
##  In Z- Position
endstop_pin: ^PA0
##  Z-position of nozzle (in mm) to z-endstop trigger point relative to print surface (Z0)
##  (+) value = endstop above Z0, (-) value = endstop below
##	Increasing position_endstop brings nozzle closer to the bed
##  After you run Z_ENDSTOP_CALIBRATE, position_endstop will be stored at the very end of your config
position_endstop: -0.5
##--------------------------------------------------------------------

##	Uncomment below for 250mm build
#position_max: 240

##	Uncomment below for 300mm build
#position_max: 290

##	Uncomment below for 350mm build
position_max: 340

##--------------------------------------------------------------------
position_min: -5
homing_speed: 8
second_homing_speed: 3
homing_retract_dist: 3

##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 stepper_z]
uart_pin: PD10
uart_address: 0
interpolate: True
run_current: 0.8
hold_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0

##	In E1-MOT Position
##	Z1 Stepper - Rear Left
[stepper_z1]
step_pin: PE6
dir_pin: PC13
enable_pin: !PE5
rotation_distance: 40
gear_ratio: 80:16
microsteps: 16

##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 stepper_z1]
uart_pin: PC14
interpolate: True
run_current: 0.8
hold_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0

##	In E2-MOT Position
##	Z2 Stepper - Rear Right
[stepper_z2]
step_pin: PE2
dir_pin: !PE4
enable_pin: !PE3
rotation_distance: 40
gear_ratio: 80:16
microsteps: 16

##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 stepper_z2]
uart_pin: PC15
interpolate: true
run_current: 0.8
hold_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0

##	In E3-MOT Position
##	Z3 Stepper - Front Right
[stepper_z3]
step_pin: PD12
dir_pin: PC4
enable_pin: !PE8
rotation_distance: 40
gear_ratio: 80:16
microsteps: 16

[tmc2209 stepper_z3]
uart_pin: PA15
interpolate: true
run_current: 0.8
hold_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#   Extruder
#####################################################################

##	In E0-MOT Position
[extruder]
step_pin: PD5
dir_pin: !PD6
enable_pin: !PD4

##	Update value below when you perform extruder calibration
##	If you ask for 100mm of filament, but in reality it is 98mm:
##	rotation_distance = <previous_rotation_distance> * <actual_extrude_distance> / 100
##  22.6789511 is a good starting point
rotation_distance: 22.6789511	#Bondtech 5mm Drive Gears
##	Update Gear Ratio depending on your Extruder Type
##	Use 50:17 for Afterburner/Clockwork (BMG Gear Ratio)
##	Use 80:20 for M4, M3.1
gear_ratio: 50:17				#BMG Gear Ratio
microsteps: 16
full_steps_per_rotation: 200	#200 for 1.8 degree, 400 for 0.9 degree
nozzle_diameter: 0.400
filament_diameter: 1.75
##      In E0 OUT Position
heater_pin: PB15
##	Validate the following thermistor type to make sure it is correct
sensor_type: ATC Semitec 104GT-2
sensor_pin: PC0 # TE0 Position
min_temp: 10
max_temp: 270
max_power: 1.0
min_extrude_temp: 170
control = pid
pid_kp = 26.213
pid_ki = 1.304
pid_kd = 131.721
##	Try to keep pressure_advance below 1.0
pressure_advance: 0.05
##	Default is 0.040, leave stock
pressure_advance_smooth_time: 0.040

##	In E0-MOT Position
##	Make sure to update below for your relevant driver (2208 or 2209)
[tmc2209 extruder]
uart_pin: PD7
interpolate: false
run_current: 0.5
hold_current: 0.4
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#   Bed Heater
#####################################################################
[heater_bed]
##	SSR Pin - In BED OUT position
heater_pin: PB4
sensor_type: NTC 100K MGB18-104F39050L32
sensor_pin: PC3 # TB Position
##	Adjust Max Power so your heater doesn't warp your bed
max_power: 0.6
min_temp: 0
max_temp: 120
control: pid
pid_kp: 58.437
pid_ki: 2.347
pid_kd: 363.769

#####################################################################
#	Probe
#####################################################################

[probe]
##	Inductive Probe - If you use this section , please comment the [bltouch] section
##	This probe is not used for Z height, only Quad Gantry Leveling
##	In Z+ position
##	If your probe is NC instead of NO, add change pin to ^PA3
pin: ^!PA3
x_offset: 0
y_offset: 25.0
z_offset: 0
speed: 10.0
samples: 3
samples_result: median
sample_retract_dist: 3.0
samples_tolerance: 0.006
samples_tolerance_retries: 3

#####################################################################
#	Bltouch
#####################################################################

#[bltouch]
##	Bltouch - If you use this section , please comment the [probe] section
##	More infomation at : https://www.klipper3d.org/BLTouch.html
##	This bltouch is not used for Z height, only Quad Gantry Leveling
##	In Z+ Position
#sensor_pin: PA0
##	In Y+ Position
#control_pin: PA2
#x_offset: 0
#y_offset: 25.0
#z_offset: 0
#speed: 10.0
#samples: 3
#samples_result: median
#sample_retract_dist: 3.0
#samples_tolerance: 0.006
#samples_tolerance_retries: 3

#####################################################################
#	Fan Control
#####################################################################

[heater_fan hotend_fan]
##	Hotend Fan - FAN0 Connector
pin: PB0
max_power: 1.0
kick_start_time: 0.5
heater: extruder
heater_temp: 50.0
##	If you are experiencing back flow, you can reduce fan_speed
#fan_speed: 1.0

[fan]
##	Print Cooling Fan - FAN1 Connector
pin: PB1
max_power: 0.4
kick_start_time: 0.5
##	Depending on your fan, you may need to increase this value
##	if your fan will not start. Can change cycle_time (increase)
##	if your fan is not able to slow down effectively
off_below: 0.10

[heater_fan controller_fan]
##	Controller fan - FAN2 Connector
pin: PB2
kick_start_time: 0.5
heater: heater_bed
heater_temp: 45.0

#[heater_fan exhaust_fan]
##  Exhaust fan - In E2 OUT Positon
#pin: PB3
#max_power: 1.0
#shutdown_speed: 0.0
#kick_start_time: 5.0
#heater: heater_bed
#heater_temp: 60
#fan_speed: 1.0

#####################################################################
#	LED Control
#####################################################################

#[output_pin caselight ]
##  Chamber Lighting - In 5V-RGB Position
#pin: PD3
#pwm: true
#shutdown_value: 0
#value:100
#cycle_time: 0.01

#####################################################################
#	Homing and Gantry Adjustment Routines
#####################################################################

[idle_timeout]
timeout: 1800

[safe_z_home]
##	XY Location of the Z Endstop Switch
##	Update -10,-10 to the XY coordinates of your endstop pin
##	(such as 157,305) after going through Z Endstop Pin
##	Location Definition step.
home_xy_position:-10,-10
speed:100
z_hop:10

[quad_gantry_level]
##	Use QUAD_GANTRY_LEVEL to level a gantry.
##	Min & Max gantry corners - measure from nozzle at MIN (0,0) and
##	MAX (250, 250), (300,300), or (350,350) depending on your printer size
##	to respective belt positions

#--------------------------------------------------------------------
##	Gantry Corners for 250mm Build
##	Uncomment for 250mm build
#gantry_corners:
#	-60,-10
#	310, 320
##	Probe points
#points:
#	50,25
#	50,175
#	200,175
#	200,25

##	Gantry Corners for 300mm Build
##	Uncomment for 300mm build
#gantry_corners:
#	-60,-10
#	360,370
##	Probe points
#points:
#	50,25
#	50,225
#	250,225
#	250,25

##	Gantry Corners for 350mm Build
##	Uncomment for 350mm build
gantry_corners:
	-60,-10
	410,420
##	Probe points
points:
	50,25
	50,275
	300,275
    300,25

#--------------------------------------------------------------------
speed: 100
horizontal_move_z: 10
retries: 5
retry_tolerance: 0.0075
max_adjust: 10

#####################################################################
#	Displays
#####################################################################

#--------------------------------------------------------------------

[display]
#	mini12864 LCD Display
lcd_type: uc1701
cs_pin: PC11
a0_pin: PD2
rst_pin: PC10
encoder_pins: ^PC6,^PC7
click_pin: ^!PA8
contrast: 63
#spi_bus: spi1
spi_software_mosi_pin: PA7
spi_software_miso_pin: PA6
spi_software_sclk_pin: PA5

[neopixel fysetc_mini12864]
#	To control Neopixel RGB in mini12864 display
pin: PC12
chain_count: 3
initial_RED: 0.1
initial_GREEN: 0.5
initial_BLUE: 0.0
color_order: RGB

#	Set RGB values on boot up for each Neopixel.
#	Index 1 = display, Index 2 and 3 = Knob
[delayed_gcode setdisplayneopixel]
initial_duration: 1
gcode:
        SET_LED LED=fysetc_mini12864 RED=1 GREEN=1 BLUE=1 INDEX=1 TRANSMIT=0
        SET_LED LED=fysetc_mini12864 RED=1 GREEN=0 BLUE=0 INDEX=2 TRANSMIT=0
        SET_LED LED=fysetc_mini12864 RED=1 GREEN=0 BLUE=0 INDEX=3

#--------------------------------------------------------------------


#####################################################################
#	Macros
#####################################################################

[gcode_macro G32]
gcode:
    BED_MESH_CLEAR
    G28
    QUAD_GANTRY_LEVEL
    G28
    ##	Uncomment for for your size printer:
    #--------------------------------------------------------------------
    ##	Uncomment for 250mm build
    #G0 X125 Y125 Z30 F3600

    ##	Uncomment for 300 build
    #G0 X150 Y150 Z30 F3600

    ##	Uncomment for 350mm build
    G0 X175 Y175 Z30 F3600
    #--------------------------------------------------------------------

[gcode_macro PRINT_START]
#   Use PRINT_START for the slicer starting script - please customise for your slicer of choice
gcode:
    G32                            ; home all axes
    G1 Z20 F3000                   ; move nozzle away from bed

[gcode_macro PRINT_END]
#   Use PRINT_END for the slicer ending script - please customise for your slicer of choice
gcode:
    M400                           ; wait for buffer to clear
    G92 E0                         ; zero the extruder
    G1 E-1.0 F3600                 ; retract filament
    G91                            ; relative positioning
    G0 Z1.00 X20.0 Y20.0 F20000    ; move nozzle to remove stringing
    TURN_OFF_HEATERS
    M107                           ; turn off fan
    G1 Z2 F3000                    ; move nozzle up 2mm
    G90                            ; absolute positioning
    G0  X125 Y250 F3600            ; park nozzle at rear
    BED_MESH_CLEAR

## 	Thermistor Types
##   "EPCOS 100K B57560G104F"
##   "ATC Semitec 104GT-2"
##   "NTC 100K beta 3950"
##   "Honeywell 100K 135-104LAG-J01"
##   "NTC 100K MGB18-104F39050L32" (Keenovo Heater Pad)
##   "AD595"
##   "PT100 INA826"

## Printer.cfg (Fix for errors)
[virtual_sdcard]
path: home/pi/gcode_files/

[pause_resume]



[gcode_macro CANCEL_PRINT]
description: Cancel the actual running print
rename_existing: CANCEL_PRINT_BASE
gcode:
   M220 S100 ; Reset Speed factor override percentage to default (100%)
#   M221 S100 ; Reset Extrude factor override percentage to default (100%)
#   G91 ; Set coordinates to relative
#   {% if printer.extruder.temperature >= 170 %}
#      G1 F1800 E-1 ; Retract filament 3 mm to prevent oozing
#   {% endif %}

#   ;if all axis are homed, lift the hotend to leave room for hot filament to ooze and to keep it clear of the bed.
#   {% if printer.toolhead.homed_axes == "xyz" %}
#      G1 F6000 Z10 ; Move Z Axis up 10 mm to allow filament ooze freely
#      G90 ; Set coordinates to absolute
#      G1 X0 Y221 F1000 ; Move Heat Bed to the front for easy print removal
#      M84 ; Disable stepper motors
#   {% endif %}

#   ;set part fan speed to zero.
#   M106 S0
#   ;bed and hotend are left at the print temps in case I want to restart.
```

#### fluidd.cfg

```
[virtual_sdcard]
path: home/pi/gcode_files

[pause_resume]

[display_status]

[gcode_macro CANCEL_PRINT]
description: Cancel the actual running print
rename_existing: CANCEL_PRINT_BASE
gcode:
   M220 S100 ; Reset Speed factor override percentage to default (100%)
#   M221 S100 ; Reset Extrude factor override percentage to default (100%)
#   G91 ; Set coordinates to relative
#   {% if printer.extruder.temperature >= 170 %}
#      G1 F1800 E-1 ; Retract filament 3 mm to prevent oozing
#   {% endif %}

#   ;if all axis are homed, lift the hotend to leave room for hot filament to ooze and to keep it clear of the bed.
#   {% if printer.toolhead.homed_axes == "xyz" %}
#      G1 F6000 Z10 ; Move Z Axis up 10 mm to allow filament ooze freely
#      G90 ; Set coordinates to absolute
#      G1 X0 Y221 F1000 ; Move Heat Bed to the front for easy print removal
#      M84 ; Disable stepper motors
#   {% endif %}

#   ;set part fan speed to zero.
#   M106 S0
#   ;bed and hotend are left at the print temps in case I want to restart.

[gcode_macro PAUSE]
description: Pause the actual running print
rename_existing: PAUSE_BASE
# change this if you need more or less extrusion
variable_extrude: 1.0
gcode:
    ##### read E from pause macro #####
    {% set E = printer["gcode_macro PAUSE"].extrude|float %}
    ##### set park positon for x and y #####
    # default is your max posion from your printer.cfg
    {% set x_park = printer.toolhead.axis_maximum.x|float - 5.0 %}
    {% set y_park = printer.toolhead.axis_maximum.y|float - 5.0 %}
    ##### calculate save lift position #####
    {% set max_z = printer.toolhead.axis_maximum.z|float %}
    {% set act_z = printer.toolhead.position.z|float %}
    {% if act_z < (max_z - 2.0) %}
        {% set z_safe = 2.0 %}
    {% else %}
        {% set z_safe = max_z - act_z %}
    {% endif %}
    ##### end of definitions #####
    PAUSE_BASE
    G91
    {% if printer.extruder.can_extrude|lower == 'true' %}
      G1 E-{E} F2100
    {% else %}
      {action_respond_info("Extruder not hot enough")}
    {% endif %}
    {% if "xyz" in printer.toolhead.homed_axes %}
      G1 Z{z_safe} F900
      G90
      G1 X{x_park} Y{y_park} F6000
    {% else %}
      {action_respond_info("Printer not homed")}
    {% endif %}

[gcode_macro RESUME]
description: Resume the actual running print
rename_existing: RESUME_BASE
gcode:
    ##### read E from pause macro #####
    {% set E = printer["gcode_macro PAUSE"].extrude|float %}
    #### get VELOCITY parameter if specified ####
    {% if 'VELOCITY' in params|upper %}
      {% set get_params = ('VELOCITY=' + params.VELOCITY)  %}
    {%else %}
      {% set get_params = "" %}
    {% endif %}
    ##### end of definitions #####
    {% if printer.extruder.can_extrude|lower == 'true' %}
      G91
      G1 E{E} F2100
    {% else %}
      {action_respond_info("Extruder not hot enough")}
    {% endif %}
    RESUME_BASE {get_params}
```

## Fysetc Display 12864

Connect EXP1 -> EXP2
Connect EXP2 -> EXP1
Insert the SD Card again with the old.bin and restart until it starts flashing longer than just the firmware of the board. Seems like the firmware is written to the display somehow. Took me 2 attempts to recognise this. In Fluidd restart Firmware and you should see something comming up. But as long as the printer is not connected to the board it went black again. But it works.

## Optional: LED (BTF-Lightning WS2812B LED Strip)

Model: BTF-5V-60L-B
Length: 1m
Voltage: 5V
Maximum Power: 18W
Special: IP67
Mode: GRB

Wires:
VCC = RED
GND = WHITE
DATA = GREEN

Two changes, both in printer.cfg

1. printer.cfg  [LED Control]
```
[neopixel caselight]
pin: PD3                     #   The pin connected to the neopixel. This parameter must be provided.
chain_count: 60              #   The number of Neopixel chips that are "daisy chained" to the provided pin. The default is 1 (which indicates only a single Neopixel is connected to the pin).
color_order: GRB             #   Set the pixel order required by the LED hardware. Options are GRB, RGB, GRBW, or RGBW. The default is GRB.
initial_RED: 0.1
initial_GREEN: 0.1
initial_BLUE: 0.1           #   Sets the initial LED color of the Neopixel. Each value should be between 0.0 and 1.0. The default for each color is 0.
```

2\. printer\.cfg
Add this to the "Macros" section
```
[gcode_macro LEDOFF]
gcode:  SET_LED LED=caselight RED=0.0 GREEN=0.0 BLUE=0.0

[gcode_macro LEDMIN]
gcode:  SET_LED LED=caselight RED=0.1 GREEN=0.1 BLUE=0.1

[gcode_macro LEDMAX]
gcode:  SET_LED LED=caselight RED=1.0 GREEN=1.0 BLUE=1.0
```

[https://github.com/Klipper3d/klipper/issues/2774](https://github.com/Klipper3d/klipper/issues/2774)

## Fluidd Test (a workaround)

So you connected the Raspberry Pi via UART, the Fysetc Mini Display 12864 a LED light strip and a Thermistor to TE0, right?
Still you get an error and Klipper goes into "shutdown".

Error log
```
static_string_id=ADC out of range
Transition to shutdown state: MCU 'mcu' shutdown: ADC out of range</span>
This generally occurs when a heater temperature exceeds its configured min_temp or max_temp.
```

Well thats because of the setting in printer.cfg called [heater\_bed] look on your Fluidd Dashboard. You will see a negativ number for the Heater Bed.
You need to trick the config to test it properly. But be aware - set it back after tests are done. This is crucial!

```
#####################################################################
#   Bed Heater
#####################################################################
[heater_bed]
##	SSR Pin - In BED OUT position
heater_pin: PB4
sensor_type: NTC 100K MGB18-104F39050L32
sensor_pin: PC3 # TB Position
##	Adjust Max Power so your heater doesn't warp your bed
max_power: 0.6
min_temp: -100     # <--- See that you need to change and change it back asap if you start assemble your printer
max_temp: 120
control: pid
pid_kp: 58.437
pid_ki: 2.347
pid_kd: 363.769
```

If you are not sure, you can always check the Fysetc default config: [https://github.com/FYSETC/FYSETC-SPIDER/blob/main/firmware/Klipper/printer.cfg](https://github.com/FYSETC/FYSETC-SPIDER/blob/main/firmware/Klipper/printer.cfg)

## Optional: MOTD (Neofetch)

```
sudo apt update
sudo apt install neofetch
sudo rm /etc/motd
sudo bash -c $'echo "neofetch" >> /etc/profile.d/mymotd.sh && chmod +x /etc/profile.d/mymotd.sh'
```

## Optional: Moonraker-Telegram-Bot

How to create the bot and some further documentation:
https://githubmemory.com/repo/barecool/moonraker-telegram

```
# Install in KIAUH
cd ~/moonraker-telegram-bot/scripts
./install
# Open https://api.telegram.org/bot%3CAPI-access-token%3E/getUpdates?offset=0
# Enter your API Token
# Send something in your chat doesnt matter and refresh page
# Safe ID from messages: "id":xxxxxxx
# Edit ~/klipper_config/telegram.conf
# chat_id = id from messages (browser)
# bot_token = your id the complete one!
# Save
```

Restart Klipper + Firmware via Fluidd and you are good to go - send "/status"

# Additional Info

## Todo

- printer.cfg heater_bed min_temp: 0 (set back!) Just testing -100
- What is LEDMIN?
- Jumper Check  
[https://3dwork.io/en/complete-guide-fysetc-spider/](https://3dwork.io/en/complete-guide-fysetc-spider/)


Videos:
[https://www.youtube.com/watch?v=TaTE5FQVnOo](https://www.youtube.com/watch?v=TaTE5FQVnOo)
