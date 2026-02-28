# ble-inclinometer

### Measure pitch and throw angles on RC aircraft and helis, using the Xiao NRF52840 Sense.

<img src="https://github.com/truglodite/ble-inclinometer/blob/main/images/IMG_2628_1.jpg" width="600">

## Features:
* Extremely Small and lightweight sensor form factor vs commercial inclinometers; won't spoil measurements on delicate/flexible surfaces with weak servos
* Very simple stand alone 1s battery only option works with Bluetooth LE using the "NRF Connect Mobile" app (iOS and Andriod)
* Optional OLED display and tare button, for convenient use without a phone (SSD1306)
* Works on surfaces at various angles (ailerons, vee tails, heli blades, etc)
* Sends accurate sub-degree roll and pitch angles ~1 per second, using a running average and floating point math with IMU optimizations
* Displays battery volts (the Xiao has a built in USB powered 1s lithium battery manager, with 50mA and 100mA charge options)
* Simplified PlatformIO flashing with open source libraries included
* LED status indicators (Blue = BLE connected, Green flash = Data updated, Red flash = Taring)
* 3D printable surface clip STL files included

## Hardware:
1@ Seeed Studio Xiao NRF52840 Sense board:

https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Pre-Soldered/dp/B09T94SZ8K/

1@ 1s lithium battery (pretty much any 1s will work):

https://www.amazon.com/flite-300mAh-3-7V-LiPo-Battery/dp/B01FTFZ1ZE/

1@ SSD1306 i2c OLED display (optional):

https://www.amazon.com/HiLetgo-Serial-128X64-Display-Color/dp/B06XRBTBTB/

1@ Momentary N.O. tactile button (optional):

https://www.amazon.com/DAOKI-100Pcs-6x6x5mm-Momentary-Tactile/dp/B07X8T9D2Q/
## Flashing:
#### Flash w/ PlatformIO (recommended):
This option might be easier, since the libraries are included in this repo. I won't go into details of how to use PlatformIO, but it is fairly simple. Download and install vscode, and install the platformIO extension in vscode. Download and unzip this repo to your "Projects" folder, and "Open Folder" in platformio home. Use the right arrow near the bottom to compile and flash.
#### Flash w/ Arduino IDE:
If you don't have the Arduino IDE and Seeed libraries installed on your PC and have a grasp on flashing an Arduino, you should start by reading the software section of this page:

  https://wiki.seeedstudio.com/XIAO_BLE/
  
Note this project will use the "Seeed nRF52 mbed-enabled Boards" since it needs the IMU. You will also have to install the Seeed gyro lib as shown on this page:

  https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
## Assembly:
Device Pin | Xiao Pin
------------ | ------------
battery +   | Bat+
battery -   | Bat-
OLED VCC    | 3v3
OLED GND    | GND
OLED SDA    | 4
OLED SCL    | 5
button 1a   | 10
button 1b   | gnd

The entire circuit uses very little current. So 30awg or even smaller wire should be adequate. Wire a 1s lithium battery to the BAT+/- pads on the back of the board. The battery can be anything from a rechargeable LR2032 coin cell up to a giant 5000mAh lipo. I recommend using a battery connector, as the battery will drain over time if left connected while not being used. I soldered some 30awg wire to a BT2.0 connector, and use one of my old/tired 1s300 tinywhoop lipos for power. If you are using an OLED and/or a tare button, wire them as shown in the table above.

Twisted 30awg silicone wire harnesses will have very little effect on movement, even with very weak servos. Avoid using a lot of heatshrink on the harness, or anything else that might make it stiffer. Make the harness long enough so the battery (and OLED+button) can can be comfortably positioned away from the surface without any wire tension during measurments. 6" is usually enough for planes; heli pilots may want a longer harness. Avoid going much longer than ~12" if possible when using an OLED screen; very long wires may result in i2c errors.

It's a good idea to add some protection so the wires don't get damaged over time. I cut pieces of thick/tacky rubber tape (3m 2242) to make a tunnel for the battery wires. This way the board lies in plane with the surface without pinching or rocking on wires. The included clip files are best printed in TPU. They are very gentle on planes, and hold the board securely when 2242 tape is used on the board. Of course you can just use clothespins or whatever else if a 3d printer is not available.
## General Use:
Clip the XIAO on the control surface to be measured and procede with measurements as needed. It is best to clip the sensor to your surface such that the roll axis (usb port) is aligned with the axis of rotation (aka "hinge line" or "feathering shaft"). When aligned perfectly, the pitch angle will not change with deflection, and the roll display will perfectly represent travel. Some small changes in pitch angle are fine. However if your measurement requires very high accuracy and you see larger changes in pitch while moving the surface (> +/-5degrees), you may want to try improving the alignment.

Take care the airframe does not move much while measuring (for example keep plane in a cradle so tailwheels/tillers are off the ground). Since only accelerometers are used, measuring yaw is not possible. Therefore you may have to tilt the airframe to measure surfaces that normally rotate about a vertical axis (ie rudders). It isn't necessary to have the surface completely horizontal; within 60 degrees of horizontal is usually good enough to get accurate measurements.

Use the Tare function (via button or BLE) to zero the angles at any position (ie sticks centered after subtrim). Use measurements to setup surface endpoints, control rates, etc.
#### Bluetooth Details:
<img src="https://github.com/truglodite/ble-inclinometer/blob/main/images/IMG_2629.PNG" height="600">

UUID | Name | Units
--- | -------- | ---
1001 | Roll axis | degrees
1002 | Pitch axis | degrees
1003 | Tare both axis | send "TRUE"
1004 | Battery Voltage | V

Install the "NRF Connect" app on your phone. When you power up your inclinometer, it will show up in the app as *"Angle Monitor"*. Connect to it, and the characteristics (sensors and controls) will appear in a list. Click the *"down-bar"* arrows on the sensor UUID's (1001, 1002, & 1003) to get continuously updated values. Click the *"quotes"* and select *"UTF-8"*. Now the angles and voltage should display correctly. Tare by clicking the "Up Arrow" on the tare UUID (1003), and send a Boolean "True" (or an UnsignedInt "1"). 

Proper descriptor names are included with all BLE characteristics. Unfortunately NRF Connect (and many similar apps) do not read or make use of them. If there's an app that does, actual sensor names are available in the transmissions.
## Notes:
* The roll is axis is oriented "going in to the USB"
* The pitch axis is oriented "across the width of the USB"
* Use the "chargeCurrent" compile option to select between 50mA and 100mA battery charging current (50mA default)
* "updateDelay" compile option to adjust refresh rate (1sec default, limited by the phone app)
* Compile using the mbed board definition: "Seeed NRF-52 mbed enabled boards\Xiao nRF52840 Sense (No Update)"
* Use the "oledFormatBig" compile option for a larger font. Best for monochrome SSD1306 displays (not so great with Y/B displays)
* Uploaded stl files in 3 sizes: 0mm, 3mm, and 6mm. Print a set with TPU to suit many different surface thicknesses... or just use a clothes pin.
## Project Roadmap:
* ...done?

I'm not a professional programmer, and this project is nothing special. Please use, modify, and/or share the code however you wish. ;)
