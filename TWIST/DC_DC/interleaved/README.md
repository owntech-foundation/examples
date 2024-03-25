# Interleaved operation

## Overview

In the case of a two-phase interleaved buck converter, the term "interleaved" implies that there are two power stages operating, and they are out of phase by 180 degrees. This means that while one power stage is in its on-state (conducting), the other is in its off-state (non-conducting), and vice versa.

The 180-degree phase shift ensures that there is always at least one power stage active, reducing the overall ripple and improving the efficiency of the buck converter. The interleaved operation helps distribute the load more evenly, minimizing the stress on individual components and resulting in a more efficient power conversion process.

![circuit](https://qph.cf2.quoracdn.net/main-qimg-27bbfbbb9466a9f8a6ee4c105a884f26)

![waveform](https://qph.cf2.quoracdn.net/main-qimg-102ebabc9c40d17b58069d02025a0175)

This example will implement interleaved operation using the two legs of the TWIST.

## Hardware setup and requirements

![schema](Image/buck_m.png)

You will need :

- 1 TWIST
- A DC voltage power supply
- A resistor (or a DC electronic load)

## Software setup

We import `control_library` with platformio.ini via the line :

```
lib_deps=
    control_lib = https://github.com/owntech-foundation/control_library.git
```

We can use this library to initialize a PID control with the function :

```cpp
pid.init(pid_params);
```

the initial parameters are defined using the following lines :

```cpp
static Pid pid; // define a pid controller.

static float32_t Ts = control_task_period * 1.e-6F;
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
```

## Expected results

The voltage reference is initially 15V, but you can increase/decrease it with the serial monitor with 'u' and 'd' on you keyboard.

