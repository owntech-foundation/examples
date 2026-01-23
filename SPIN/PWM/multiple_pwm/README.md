# Working with multiple PWM

The SPIN has a total of 5 PWM channels with 2 complementary outputs each. In this example, we'll detail how to use each of them.

## Hardware setup and requirements

![schema](Image/schema.png)
*figure 1*

You will need:

- A SPIN
- A USB-C cable to supply the SPIN
- An oscilloscope to watch PWM waveform

We can watch:

- PWMA1 on GPIO A8
- PWMC1 on GPIO B12
- PWMD1 on GPIO B14
- PWME1 on GPIO C8
- PWMF1 on GPIO C6

## Software setup

This example is initializing every PWM, and making a phase shift of 72° (= 360/5) as if we were working in interleaved mode. See the [phase shift](../phase_shift/README.md) example for more details.

The duty cycle is the same for both PWMA and PWMC.

You can control the duty cycle from the serial monitor :
- press `u` to increase the duty cycle
- press `d` to decrease the duty cycle

See [OwnPlot](https://github.com/owntech-foundation/OwnPlot) if you would like a better graphical interface for the serial monitor.

## Expected result

![waveform](Image/waveform_multiple_pwm.png)

You should observe 5 PWMs with a phase shift of 72° between them.
