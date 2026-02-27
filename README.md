# ble-inclinometer

### Simple code for measuring pitch/throw angles on RC helis/aircraft, using the tiny Xiao NRF52840 Sense board and a free phone app.

<img src="https://github.com/truglodite/ble-inclinometer/images/IMG_2628_1.jpg" width="600">

<img src="https://github.com/truglodite/ble-inclinometer/images/IMG_2629.PNG" width="600">

## Features:
* Works with Bluetooth LE using the "NRF Connect Mobile" app on iOS and Andriod
* Sends updated roll and pitch angles every second, with sub-degree precision
* Works on surfaces at any angle (ailerons, vee tail surfaces, etc)
* Small and lightweight form factor vs commercial inclinometers; stays accurate on delicate/flexible surfaces
* Use the app to zero both axis
* Reads battery volts, for use with a small lithium battery attached to the B+/B- pads (the Xiao has a built in lithium charger)
* LED status indicators (Blue = BLE connected, Green flash = Data updated, Red flash = Taring)
* Optional OLED screen for use without a phone (SSD1306)
* Optinoal Tare button for use without a phone
* Printable clip STL files

## Hardware:
1@ Seeed Studio Xiao NRF52840 Sense board:

https://www.amazon.com/Seeed-Studio-XIAO-nRF52840-Pre-Soldered/dp/B09T94SZ8K/

1@ 1s lithium battery (pretty much any 1s will work):

https://www.amazon.com/flite-300mAh-3-7V-LiPo-Battery/dp/B01FTFZ1ZE/

## Install:
#### Flash the code using Arduino IDE:
If you don't have the Arduino IDE and Seeed libraries installed on your PC and have a grasp on flashing an Arduino, you should start by reading the software section of this page:

  https://wiki.seeedstudio.com/XIAO_BLE/
  
Note this project will use the "Seeed nRF52 mbed-enabled Boards" since it needs the IMU. You will also have to install the Seeed gyro lib as shown on this page:

  https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
  
Once the board is flashed, wire a 1s lithium battery to the BAT+/- pads on the back of the board. The battery can be anything from a rechargeable LR2032 coin cell up to a giant 5000mAh lipo. I recommend adding wires and a connector since this board does not charge very fast (100mA) and will drain the battery over time if left connected. I soldered some 30awg wire to a BT2.0 connector, and use a tinywhoop pack for power. 

Since the board will be clipped with the bottom side facing control surfaces, it's a good idea to add some protection so the wires don't get damaged. I cut pieces of thick 3m 2242 tape to make a tunnel for the wires. This way the board lies in plane with the surface without pinching wires. The printable clips are gentle on planes, and plenty secure especially if 2242 tape is used.

## Use:
Install the NRF Connect app on your phone. When you power up your inclinometer, it will show up as "Angle Monitor" in the app. Connect to it, and the characteristics (sensors and controls) will show up on a list.
#### Sensor List
UUID | Name | Units
--- | -------- | ---
1001 | Roll axis | degrees
1002 | Pitch axis | degrees
1003 | Tare both axis | send "TRUE"
1004 | Battery Voltage | V

Click the "down-bar" arrows near the sensors to show updated values. SClick the "quotes" and select "UTF-8". Now the angles and voltage should be displayed correctly. Clip the board on to the surface you need to measure, taking care to align an edge of the board with the hingeline. Move the surface to verify which axis is aligned (roll goes into the USB port, pitch goes across the USB). Zero both axis by clicking the "Up Arrow" on the tare UUID, and send a boolean "True" (or an UnsignedInt "1"). Now setup the surface endpoints, rates, etc, move to the next surface and rezero as needed.

## Notes:
* The roll is axis is oriented "going in to the USB"
* The pitch axis is oriented "across the USB"
* "chargeCurrent" compile option to select between 50mA and 100mA battery charging current (50mA default)
* "updateDelay" compile option to adjust refresh rate (1sec default, limited by the phone app)
* Compile using the mbed board definition: "Seeed NRF-52 mbed enabled boards\Xiao nRF52840 Sense (No Update)"
* Uploaded stl files in 3 sizes: 0mm, 3mm, and 6mm. Print a set with TPU to suit many different surface thicknesses... or just use a clothes pin.
## Possible additions:
* Button for hardware zeroing
* ...?

It's nothing special. Feel free to use and/or modify the code however you wish. ;)
