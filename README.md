# ble-inclinometer

## Simple code for measuring pitch/throw angles on RC helis/aircraft, using the tiny Xiao NRF52840 Sense board and a free phone app.

<img src="https://github.com/truglodite/ble-inclinometer/blob/main/IMG_2628_1.jpg" width="600">

### Features:
* Works with Bluetooth LE using the "NRF Connect Mobile" app on iOS and Andriod
* Sends updated roll and pitch angles every second
* Works on surfaces at any angle (ailerons, vee tail surfaces, etc)
* Small and lightweight form factor vs commercial inclinometers; stays accurate on delicate/flexible surfaces
* Use the app to zero both axis
* Reads battery % for use with a small lithium battery attached to the B+/B- pads (the Xiao has a built in lithium charger)

### Notes:
* Roll Sensor = "2c08"
* Pitch = "2c09"
* The roll is axis is oriented "going in to the USB"
* The pitch axis is oriented "across the USB"
* Zero both axis by sending a "True" boolean to the tare service (long uuid)
* "chargeCurrent" compile option to select between 50mA and 100mA battery charging current (50mA default)
* "updateDelay" compile option to adjust refresh rate (1sec default)
* Use the mbed board: "Seeed NRF-52 mbed enabled boards\Xiao nRF52840 Sense (No Update)"

### Possible additions:
* Button for hardware zeroing
* ...?

It's nothing special. Feel free to use and/or modify the code however you wish. ;)
