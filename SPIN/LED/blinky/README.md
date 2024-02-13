# Blinking a led

Here is a simple example to start with SPIN : making a led blink.

## Hardware setup and requirements

![Schematic](Image/schema.png)
*figure 1*

You will need : 

- 1 spin
- A usb-c cable to supply power to the spin, and also upload the code from computer

## Software setup

The led is toggled in the background task called each 1s, which means the leed will blink at the rate of 1s : 

```cpp
    spin.led.toggle();
```

## Expected result 

Visual result : the led should turn on and off. 