# Triggering measure from ADC via HRTIM trigger

An ADC, or Analog-to-Digital Converter, is a crucial component in modern electronics that converts continuous analog signals into discrete digital data. In simpler terms, it takes real-world phenomena, like sound or temperature, and turns them into numbers that a computer can understand and process. This conversion is essential for various applications, in power electronics it allows us to get real-time measures from the circuit like voltage and current. 

The spin uses the stm32G4 MCU, which has a high resolution timer (HRTIM) which can produce high resolution PWM. This example will show you how to use the HRTIM in order to trigger the measures. 

## Hardware setup and requirements

![Schematic](Image/schema.png)
*figure 1*

You will need : 

- 1 SPIN
- A usb-c cable to supply power to the spin, and also upload the code from computer
- A signal generator to create a waveform to measure it from ADC, it can be sinewave, triangle wave...etc. This signal must between **0V and 2.048V**

There is a total of 8 possible pins from where you can get synchronous analog measurements :

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


In this example we connect the signal generator to pin `35`, and a `GND` pin. 

## Software setup 

The HRTIM has 6 different timer (A,B,C,D,E,F) but you can choose up to two of them to trigger the measures. In this example we'll show you how you can use one of them **the timer A**.

On the picture below, you can see the waveform of the timer A pwm, the carrier, the duty cycle and the colored zone represents the timing where the measures is triggered. 

![trigger_waveform](Image/Hrtim_trigger.png)
*figure 2*

The measure is triggered with the same frequency as the switching frequency, the measure is done around the trough of the carrier on the positive slope. We will explain here which functions to call to setup the ADC trigger. 

First of all, we start by initializing the PWMA : 

```cpp
    spin.pwm.setFrequency(200000); // Set frequency of pwm

    spin.pwm.setModulation(PWMA, UpDwn); // Modulation mode, here up-down (triangle wave carrier)
    spin.pwm.setAdcEdgeTrigger(PWMA, EdgeTrigger_up); // Trigger on the positive slope

    spin.pwm.initUnit(PWMA); // timer initialization
```
The function `setAdcEdgeTrigger` allows us to choose where we want to trigger the measures : on the positive slope `EdgeTrigger_up` of the carrier like in fig.2, or the negative slope `EdgeTrigger_down` of the carrier.

After the initialization of the PWM, we can link it to a trigger : 

```cpp
    spin.pwm.setAdcTrigger(PWMA, ADC_2); // PWMA is linked to ADC_2
    spin.pwm.setAdcTriggerInstant(PWMA, 0.06); // set the trigger instant
    spin.pwm.enableAdcTrigger(PWMA); // enable the trigger
```
We are linking PWMA to a trigger, here `ADC_2`.  
There is two ADCs that support synchronous data acquisition : `ADC_1` and `ADC_2`.

`setAdcTriggerInstant` will set the moment when we trigger a measure with a parameter between 0 (corresponding to the trough of the carrier)and 1 (corresponding to the crest of the carrier). Here we took 0.06, so we'll get the data around the trough of the carrier (as we have seen on fig.2)

And finally, we set the `ADC2` to be triggered by the PWM `TRIG_PWM`.

```cpp
    spin.adc.configureTriggerSource(ADC_2, TRIG_PWM); // ADC 2 configured to be triggered by the PWM
```
We use the 5th channel of the adc, which is the gpio `35` on the spin (also numbered as C4). To enable the acquisition from this pin, we use the function `enableAcquisition`: 

```cpp
    spin.data.enableAcquisition(35, ADC_2); // Enable acquisition for ADC2, for channel 5 (localized in GPIO C4 / pin number 35)
```
When are now set, we must be sure that there is a critical task defined and active, and make sure that `PWMA` is active because it is now the ADC trigger source.

```cpp
    spin.pwm.startDualOutput(PWMA);
```

## Add a second ADC triggered by another PWM channel.

You can also use `ADC_1` to get a second independent trigger event, for instance 
we can bind `PWMC`to `ADC_1`.
Below, we reproduce the same step but by using the `ADC1` channel 2 localized on pin `30` (PA1).

start PWMC :

```cpp
    spin.pwm.setFrequency(200000); // Set frequency of pwm

    spin.pwm.setModulation(PWMC, UpDwn); // Modulation mode, here up-down (triangle wave carrier)
    spin.pwm.setAdcEdgeTrigger(PWMC, EdgeTrigger_up); // Trigger on the positive slope

    spin.pwm.initUnit(PWMC); // timer initialization
```
link `ADC_1` to `PWMC` : 

```cpp
    spin.pwm.setAdcTrigger(PWMC, ADC_1); // PWMA is linked to ADCTRIG_1
    spin.pwm.setAdcTriggerInstant(PWMC, 0.06); // set the trigger instant
    spin.pwm.enableAdcTrigger(PWMC); // enable the trigger
```
then set `ADC1` channel 2 to be triggered by the PWM : 

```cpp
    spin.adc.configureTriggerSource(ADC_1, TRIG_PWM); // ADC 1 configured to be triggered by the PWM
   spin.data.enableAcquisition(30, ADC_1); // ADC 1 enabled
```

## Expected results

The analog value measured from the adc is stored inside the variable `adc_value`, which is printed in the serial monitor every 100ms you can then watch the measured on [ownplot](https://github.com/owntech-foundation/OwnPlot). 

If everything went correctly, you should observe the same waveform on ownplot that you generate via the signal generator. 