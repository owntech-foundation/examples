# CAN Communication Example
This example shows how to use the CAN interface present on both TWIST
and OWNVERTER power shields.

# CAN protocol
Data is using Thingset protocol. Full specification can be accessed here :
[thingset](thingset.io)

# App.conf
App.conf file permits to simply add complex modules. In this example, CAN
communication is enabled by setting

```CONFIG_OWNTECH_COMMUNICATION_ENABLE_CAN=y```

# Using a CAN dongle adapter

PEAK CAN to USB adapter is supported.
It works natively on linux.
To visualize CAN data stream

1. Open a terminal
2. Set dongle parameters ``` sudo ip link set can0 type can bitrate 500000 restart-ms 500ms ```
3. Enable can interface  ``` sudo ip link set can0 up ```
4. Dump the data stream  ``` can-dump can0 ```

# Visualizing CAN bus using a python script

The provided script can be used to plot incoming data.