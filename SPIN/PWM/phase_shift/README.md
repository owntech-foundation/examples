# Phase shifting PWM

Phase shift in Pulse Width Modulation (PWM) refers to the intentional offset of the timing between multiple PWM signals. This offset alters the switching instants of the signals, affecting the distribution of power delivery and minimizing ripple in power electronic systems.

Phase shifting PWM is used in interleaved topology for power electronics. In this example we'll show you how setup a phase shift for a 2 leg interleaved configuration for example.

## Hardware setup and requirements

The spin can use up to 5 different PWM : PWMA, PWMC, PWMD, PWME and PWMF. This example will detail how to work with two of them in phase shifted mode : PWMA and PWMB.

![schema](Image/schema.png)
*Figure 1*

You will need :

- A spin
- A usb-c cable to supply the spin
- An oscilloscope to watch PWM waveform

We can watch PWMA1 on gpio A8 and PWMC1 on gpio B12.

## Software setup

We start by initializing PWMA and PWMC :

```cpp
    spin.pwm.setFrequency(200000); // Set frequency of pwm

    /* PWM A initialization */
    spin.pwm.initUnit(PWMA); // timer initialization
    spin.pwm.startDualOutput(PWMA); // Start PWM

    /* PWM C initialization */
    spin.pwm.initUnit(PWMC); // timer initialization
    spin.pwm.setPhaseShift(PWMC, 180); // Phase shift of 180° for 2 legs interleaved configuration
    spin.pwm.startDualOutput(PWMC); // Start PWM
```

The frequency for the pwm is initalized to 200kHz but you are free to choose another value.

PWMC is shifted of 180° from PWMA.

The duty cycle is updated in the high-speed control task :

```cpp
    spin.pwm.setDutyCycle(PWMA, duty_cycle);
    spin.pwm.setDutyCycle(PWMC, duty_cycle);
```

The duty cycle is the same for both PWMA and PWMC.

You can control the duty cycle from the serial monitor :
- press `u` to increase the duty cycle
- press `d` to decrease the duty cycle

See [ownplot](https://github.com/owntech-foundation/OwnPlot) if you would like a better graphical interface for the serial monitor.

## Expected result

![waveform](Image/waveform_phase_shift.png)

On the oscilloscope you should observe that PWMC1 is phase shifted of 180° from PWMA1 (which means they are complementary).