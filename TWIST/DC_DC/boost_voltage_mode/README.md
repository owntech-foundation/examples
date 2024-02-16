# Boost with PID controlled output voltage

A voltage mode boost converter regulates voltage by comparing the output voltage to a reference voltage. It adjusts the duty cycle of its switching signal to keep the output voltage stable. This type of converter efficiently steps up voltage levels, making it useful in various electronic devices such as converting photovoltaic panel voltage.

This example will implement a voltage mode boost converter to control the output.


## Hardware setup and requirement


![schema](Image/boost_m.png)

You will need :
- 1 TWIST
- A dc power supply (**max 10V**)
- A resistor (or a dc electronic load)

## Software setup

We import control_pid library with src/owntech.ini via the line :

```python
lib_deps=
    control_pid = https://gitlab.laas.fr/owntech/power-api/opalib-control-pid.git
```

We can use this library to initialize a PID control with the function :

```c
opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);
```

the initial parameters are :

```c
static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;
static uint32_t control_task_period = 100;
```

## Expected result

This code will control the output voltage to have 15V, you can control the output voltage with the serial monitor :

- press `u` to increase the voltage
- press `d` to decrease the voltage

