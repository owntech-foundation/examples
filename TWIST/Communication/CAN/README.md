# CAN Communication Example
This example shows how to use the CAN interface present on both TWIST
and OWNVERTER power shields.

# CAN protocol
Data uses the Thingset protocol. The full specification can be accessed here:
[thingset](https://thingset.io)

# Using CAN on OwnTech boards
CAN-related functions can be used by typing
`communication.can.` in your main.cpp file.
Autocompletion will give you insights into available API functions.

Currently the API supports two modes :
- Sending reports containing data over CAN, in this example live measurements
- Sending control commands over CAN

For now, two simple commands are supported:
- Sending a floating point reference.
- Sending a start-stop boolean.

# App.conf
The `app.conf` file allows you to simply add complex modules. In this example, CAN
communication is enabled by setting

```CONFIG_OWNTECH_COMMUNICATION_ENABLE_CAN=y```

Multiple configurations can be set or unset.

You might want to run GUIconfig in `OwnTech -> [Advanced] Run GUIconfig` to get
extra information on available options that can be defined in `app.conf`.

Relevant configs are found in `Modules->thingset-sdk->Thingset SDK->CAN interface`

# user_data_objects.h
This new file is a manifest that permits the user to define custom values that
should be broadcast over CAN.

Follow the syntax provided with the default measurements to add yours.
We also strongly suggest reading the [Thingset protocol specification](https://thingset.io/spec/v0.6/introduction/abstract.html).


# Using a CAN dongle adapter to read CAN frames on a computer

Make sure you have a suitable hardware adapter to link `RJ45` terminals of the
OwnTech board to the `D Sub - 15` terminal of the adapter. `GND` `CAN-RX` and
`CAN-TX` are required to be able to receive and send messages.

PEAK CAN-to-USB adapter is supported. It works natively on Linux.
The following configuration should also work for other CAN dongle adapters,
granted that you've installed any required driver.

To visualize the CAN data stream

1. Open a terminal
2. Set dongle parameters ``` sudo ip link set can0 type can bitrate 500000 restart-ms 500 ```
3. Enable can interface  ``` sudo ip link set can0 up ```
4. Dump the data stream  ``` candump can0 ```

# Using Download Firmware Upgrade over CAN

Now that the CAN interface is running, you can take advantage of it to download
new firmware through CAN as well.

For that :

1. Set the CAN to USB interface
``` sudo ip link set can0 type can bitrate 500000 restart-ms 500 ```
2. Enable the CAN interface ``` sudo ip link set can0 up ```
3. Build your firmware as you would do normally.
4. Once built execute the following script from a terminal
``` ~/.platformio/packages/framework-zephyr/_pio/thingset-zephyr-sdk/scripts/thingset-dfu-can.py -t 1 .pio/build/USB/firmware.mcuboot.bin ```
