This is a branch to test the board in boost voltage mode closed-loop.

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

The voltage reference is initially 15V, but you can increase/decrease it with the serial monitor with 'u' and 'd' on you keyboard.

