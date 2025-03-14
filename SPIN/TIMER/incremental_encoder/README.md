# Using the spin with an incremental encoder

An incremental encoder is a device that converts mechanical motion into digital signals. It typically consists of a rotating disk with evenly spaced slots and a sensor that detects these slots as the disk turns. The sensor generates electrical pulses corresponding to the motion, which can be used to track position, speed, or direction.

in this example we'll see how to use an incremental encoder with spin.


## Hardware setup and requirement

![schema](Image/spin_wiring_diagram.drawio)

You will need :

- A spin
- A usb-c cable to supply the spin
- A rotary incremental encoder
- A 5V supply for your incremental encoder

In this example you can use two different timers with your encoder. 
Choose the appropriate timer according to the image above. 

## Software setup

We initialize the incremental encoder :

```cpp
    spin.timer.startLogIncrementalEncoder(TIMER3);
    spin.timer.startLogIncrementalEncoder(TIMER4);
```

The value from the incremental encoder is updated in the background task (called every 100ms). 

```cpp
    incremental_value_timer_3 = spin.timer.getIncrementalEncoderValue(TIMER3);
    incremental_value_timer_4 = spin.timer.getIncrementalEncoderValue(TIMER4);
    printk("TIM3: %u , TIM4: %d\n", incremental_value_timer_3, incremental_value_timer_4);

```


This value is displayed in the serial monitor.

## Expected result

You should see the value in the serial monitor either increasing or decresing depending on how you turning the rotary incremental encoder (clokc-wise or not).

Every turn your values should be reset to a reference value that will depend on your encoder. 