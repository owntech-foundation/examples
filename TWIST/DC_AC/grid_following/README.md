# Ac current source follower

In this example you need to have a first Twist with the [Grid Forming](../grid_forming/README.md) example.

<div style="text-align:center"><img src="Image/schema_grid_following.png" alt="Schematic p2p" width="600"></div>

The parameters are:

* $U_{DC} = 40 V$
* $R_{LOAD} = 15 \Omega$.

In the second Twist we use a software phase locked loop ( _"PLL"_ ).
By this way we are synchronised with the grid voltage and we can then inject current
with a power factor of one. The current is regulated using a proportional resonant (_"PR"_)
regulator.

## Software overview 
### Import a library

the _"pll"_ and _"pr"_ are provided by the OwnTech control library which must be included 
in the file `platfomio.ini`.

```
lib_deps=
    control_lib = https://github.com/owntech-foundation/control_library.git
```
### Define a regulator

The Proportional Resonant regulator is initialized with the lines above:

```cpp
PrParams params = PrParams(Ts, Kp, Kr, w0, 0.0F, -Udc, Udc);
prop_res.init(params);
```

The parameters are defined with these values:

```cpp
static Pr prop_res; // controller instanciation. 
static float32_t Kp = 0.2F;
static float32_t Kr = 3000.0F;
static float32_t Ts = control_task_period * 1.0e-6F;
static float32_t w0 = 2.0 * PI * 50.0;   // pulsation
```

### Configure the PLL

You have to define a PLL:
```cpp
static PllSinus pll;
static PllDatas pll_datas;
```

Then initialize it:
```
float32_t rise_time = 50e-3;
pll.init(Ts, Vgrid_amplitude, f0, rise_time);
```

and use it:
```cpp
pll_datas = pll.calculateWithReturn(V1_low_value - V2_low_value);
```

The calculation return a structure with 3 fields:

1. the pulsation `w` in [rad/s]
2. the angle `angle` in [rad]
3. the angle error `error` in [rad/s]

## Link between voltage output and duty cycle

The voltage source is defined by the voltage difference: $U_{12} = V_{1low} - V_{2low}$.

Link with the duty cycle:

* The leg1 is fixed in buck mode then: $V_{1low} = \alpha_1 . U_{DC}$
* The leg2 is fixed in boost mode then: $V_{2low} = (1-\alpha_2) . U_{DC}$

We change at the same time $\alpha_1$ and $\alpha_2$, then we have : $\alpha_1 = \alpha_2 = \alpha$. <br>
And then: $U_{12} = (2.\alpha - 1).U_{DC}$

$\alpha = \dfrac{U_{12}}{2.U_{DC}}  + 0.5$

## Retrieve recorded datas

During tests, you can get recorded datas by pressing 'r'.
if you can record the datas printed in a file, you can the use the python script
`script.py` to format the datas and have a csv file.


