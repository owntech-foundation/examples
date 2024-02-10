This is a branch to test the board in buck interleaved.

In the case of a two-phase interleaved buck converter, the term "interleaved" implies that there are two power stages operating, and they are out of phase by 180 degrees. This means that while one power stage is in its on-state (conducting), the other is in its off-state (non-conducting), and vice versa.

The 180-degree phase shift ensures that there is always at least one power stage active, reducing the overall ripple and improving the efficiency of the buck converter. The interleaved operation helps distribute the load more evenly, minimizing the stress on individual components and resulting in a more efficient power conversion process.

![circuit](https://qph.cf2.quoracdn.net/main-qimg-27bbfbbb9466a9f8a6ee4c105a884f26)

![waveform](https://qph.cf2.quoracdn.net/main-qimg-102ebabc9c40d17b58069d02025a0175)


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

