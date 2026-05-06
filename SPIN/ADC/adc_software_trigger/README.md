# Triggering measure from ADC via software trigger

An ADC, or Analog-to-Digital Converter, is a crucial component in modern electronics that converts continuous analog signals into discrete digital data. In simpler terms, it takes real-world phenomena, like sound or temperature, and turns them into numbers that a computer can understand and process. This conversion is essential for various applications, in power electronics it allows us to get real-time measurements from the circuit like voltage and current. 

This example will show you how to get measurements from the ADC by calling a function that will trigger the measurements: this is what we call **a software trigger**.

## Hardware setup and requirements

![Schematic](Image/schema.png)

You will need: 

- 1 SPIN
- A USB-C cable to supply power to the SPIN, and also upload the code from a computer
- A signal generator to create a waveform to measure it from the ADC, it can be a sine wave, triangle wave...etc. This signal must be between **0 V and 2.048 V**

Connect the signal generator to pin C4, and GND. 

## Software setup 

The ADC 2 is used here, it is initialized like this : 

```cpp
    spin.adc.configureTriggerSource(2, software); // ADC 2 configured in software mode
```
We use the 5th channel of the ADC, which is the GPIO C4 on the SPIN (also numbered as pin 35). To enable the acquisition from this pin, we use the function `enableAcquisition`: 

```cpp
data.enableAcquisition(2, 35) // Enable acquisition for ADC2, for channel 5 (localized in GPIO C4 / pin number 35)
```
When we want to retrieve measurements from ADC2 all we need to do is trigger ADC2 and retrieve the measured value in GPIO C4 / pin number 35:

```cpp
   spin.data.triggerAcquisition(2);
    adc_value =spin.data.getLatestValue(2, 35);
```

There is a total of 8 possible pins from where you can get analog measurements:

| GPIO | PIN number | ADC and channels                |
|------|------------|---------------------------------|
| PC4  | 35         | ADC2 channel 5                  |
| PA1  | 30         | ADC1 channel 2 / ADC2 channel 2 |
| PA0  | 29         | ADC1 channel 1 / ADC2 channel 1 |
| PC3  | 27         | ADC1 channel 9 / ADC2 channel 9 |
| PC2  | 26         | ADC1 channel 8 / ADC2 channel 8 |
| PC1  | 25         | ADC1 channel 7 / ADC2 channel 7 |
| PC0  | 24         | ADC1 channel 6 / ADC2 channel 6 |
| PB15 | 6          | ADC4 channel 5                  |

You can configure any of the above ADCs with the same steps.

## Expected results

The analog value measured from the ADC is stored inside the variable `adc_value`, which is printed in the serial monitor every 100 ms. You can then watch the measurements on [OwnPlot](https://github.com/owntech-foundation/OwnPlot). 

If everything went correctly, you should observe the same waveform on OwnPlot that you generate via the signal generator. 
