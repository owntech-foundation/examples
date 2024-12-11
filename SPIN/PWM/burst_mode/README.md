# Burst mode PWM

Burst mode is an Advanced PWM capability. In few words, instead of generating a continuous stream of pulses, burst mode sends groups of PWM (Pulse Width Modulation) signals at regular intervals. This method reduces energy consumption, minimizes heat generation, and is especially useful in applications like motor control, LED dimming, and power supplies.

In power electronics, Burst mode PWM is used two reduce losses under light load conditions
In this example we'll show you how to setup burst mode for two phase shifted H-bridges.
It could be used to control a Dual Active Bridge (DAB) in Single Phase Shift modulation modulation under light load conditions.

## Hardware setup and requirements

You will need :

- A spin
- A usb-c cable to supply the spin
- An oscilloscope to observe PWM waveform

PWMs can be observed on:
- Pin 12 and Pin 14 for PWMA
- Pin 2 and Pin 4 for PWMC
- Pin 10 and Pin 11 for PWME
- Pin 7 and PIN 9 for PWMF

## Software setup

During initialization:

- The frequency for the pwm is set to 200kHz, you are free to choose another value.
- PWMA and PWMC for the first H-bridge.
- PWME and PWMF for the second H-bridge.

```cpp
    spin.pwm.setFrequency(200000); // Set frequency of pwm

    /* Set switch convention to have proper H bridges */
    spin.pwm.setSwitchConvention(PWMC, PWMx2);
    spin.pwm.setSwitchConvention(PWMF, PWMx2);

    spin.pwm.initUnit(PWMA); // timer initialization
    spin.pwm.initUnit(PWMC);
    spin.pwm.initUnit(PWME);
    spin.pwm.initUnit(PWMF);
```

- All PWM are initialized in phase.

```cpp
    /* Set initial phase shifts*/
    spin.pwm.setPhaseShift(PWMC, 0);
    spin.pwm.setPhaseShift(PWME, 0);
    spin.pwm.setPhaseShift(PWMF, 0);
```

Burst mode initialization:

- Burst mode duty is set to 8. It means that PWM output will be void during 8 PWM events
- Burst mode Period is set to 10. It means that PWM output will be off for 8 PWM event every 10 PWM events.

```cpp
uint8_t burst_duty = 8;
uint8_t burst_period = 10;
```

```cpp
    spin.pwm.initBurstMode();
    spin.pwm.setBurstMode(burst_duty, burst_period);
    spin.pwm.startBurstMode();
```

## Expected result

You can control the phase shift from the serial monitor :
- press `u` to increase the phase shift
- press `d` to decrease the phase shift

You can control the burst mode duty from the serial monitor :
- press `e` to increase the burst mode duty
- press `r` to decrease the burst mode duty

You can control the burst mode period from the serial monitor :
- press `t` to increase the burst mode period
- press `y` to decrease the burst mode period

On the oscilloscope you should observe

- That PWMs are ON only **two periods out of ten**
- That you can control the burst duty and period to change the number of active PWM periods
- That you can control the second H-bridge (PWME and PWMF) phase shift from the first H-bridge (PWMA PWMC)

**NB:**

PWMs can be observed on:
- Pin 12 and Pin 14 for PWMA
- Pin 2 and Pin 4 for PWMC
- Pin 10 and Pin 11 for PWME
- Pin 7 and PIN 9 for PWMF


See [OwnPlot](https://github.com/owntech-foundation/OwnPlot) if you would like a better graphical interface for the serial monitor.