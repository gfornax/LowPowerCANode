# LowPowerCANode
A simple, low power CAN node for use in cars. Uses Arduino, MCP2515 and TJA1051

# Project overview
This is a simple example of a low power CAN node that originated while I was adding several CAN devices to my car. The goal was to have a device that can safely be left on the bus without draining the battery. I achieved about 1mA sleep current, with most of it consumed by the recessive CAN transceiver.
I was also aiming at using parts that were more or less easily available. Nonetheless, you should be able to solder SMD.

The controller will listen for any traffic on the CAN bus (defined by the filter masks), wake up via interrupt, and after a configurable time go to sleep again if the bus is silent. This approach is usually suitable for the infotainment CAN in typical cars.
I included a small example that will detect door state changes.

# DISCLAIMER

If you mess around with your car, you do it on your own responsibility. You should be aware that damage or safety-critical incidents may happen even unintentionally.
It's generally a good idea to stick to the iCAN and start with recessive/silent mode only.

# A few words on car battery powered devices
Most cars have a reasonably large battery, so why should you care about power consumption? First of all, car batteries are not made for cyclic use, which means you should always keep them close to their steady state charge level. Secondly, even a small controller consuming just 30-40mA would drain your battery in a matter of weeks enough so the car wouldn't start. Don't forget there are other devices in your car consuming standby power already. 

Another important part when building electronic devices for cars is proper filtering of the input voltage. The schematic below contains an example that worked well for me so far. Connecting an Arduino directly to the car's power is a bad idea.

# Preparation
The example contains of two devices which have to be prepared beforehand: A standard Arduino and a MCP2515 module.

## Preparing the Arduino
I propose using an Arduino mini as it comes WITHOUT the USB-serial converter which would consume power. Any clone will do of course.
Firstly, we need to remove the LED or its resistor.
Secondly, we also remove the linear power regulator which usually is a 3 or 5pin SOT23 device.
I circled these parts in the picture.
![modules](/pics/modules.png)

When finished, you can try any low power mode code example, the board should consume around 30-40uA in sleep mode.

Note: You can also use a 8MHz variant at 3.3V, which will save you a bit more power while active. The difference
while sleeping is negligible compared to the CAN transceiver. Moreover, you will need a 5V rail for the transceiver anyway.

## Preparing the MCP2515 module
The module I am using is a standard, inexpensive CAN bus shield that you can easily get. It uses the MCP2515 via SPI.
To reduce the power consumption, we also remove R1 to disable the LED.

Unfortunately, these modules mostly come with a TJA1050, which does NOT have a nice silent mode. Remove it.
We will replace it with a *TJA1051*. Before soldering it, bend the S pin (#8) up so it will not connect to the PCB.
Now, you can either directly connect it to the Vcc pin (#3) if listen-only is sufficient or connect it to the S-output of the arduino (see below).

Note: If you want to use a 3.3V circuit, use a TJA1051T/3 that will allow you to use 3.3V Vio.

# Setup
![schematic](/pics/schematic.png)
## Power supply
As you can see in the schematic, I used a simple inductive/capacitive filtering circuit. The part values are basically educated guesses, don't take them too seriously.

The conversion to 5V is a bit more difficult: Switching buck modules will definitely consume too much standby current, but also most of the commonly used linear regulators like the 7805 are not exactly good in that regard.
The best I've found so far is a Holtek HT7550-3 (mind the -3! Others have lower Vin max), which has a reasonably low quiescent current, while still allowing sufficient input voltage.

Keep in mind that you are wasting ~10V in the regulator. With Pmax 0.5W, that means you may not draw more than 50mA. Also, add a heat sink.
If you need to power more devices, you should add an additional buck module that is switched on while the node is active.

## Modules and other parts
The connection via SPI is straightforward. Depending on your setup, you can connect the Sleep/Silent line.
The LED is also optional and will just show you when the node is active.
There is also a simple ADC undervoltage detection circuit that will keep the controller in sleep mode and should disable any outputs (defined by you).

# Software
The arduino is programmed by the standard serial interface. Things you should have a look on:
 * controller type (168 vs 328) and frequency
 * undervoltage level (measured manually)
 * crystal on your MCP shield (mostly 8MHz)
 * CAN bus speed (depends on your car and which bus you are using)
 * Filters and masks: find out what you want to listen, set it accordingly
 * define if you want to listen only or send as well. Set the mode of the MCP accordingly.

I left some debug code that will output CAN messages etc. Use it as needed.

Note that I included the MCP library from https://github.com/zkemble/MCP_CAN_lib as it had some vital changes for this project.
Once these are merged into the "official" library, it should work seamlessly with it as well.

The example door state detection was successfully tested in a VW T5.

# Use case
The first device I build with this was a light controller that reacts on doors, ignition and also my own controllers that I hooked on the iCAN.
I uses a 16chan PWM module, WS2812B output and a BLE controller. If you are interested in details, please write me!

# Other remarks
If 1mA is still too much, you will most likely need an other approach, like putting the transceiver in a disabled state and waking up periodically.
Another interesting device worth having a look at is the TJA1145 - unfortunately more difficult to buy and you will need to design your own pcb.

