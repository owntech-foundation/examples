# Ac Voltage Source

In this example we build an AC voltage source using a Twist and supply a resistor.

<div style="text-align:center"><img src="Image/grid_forming.png" alt="Schematic p2p" width="600"></div>

The parameters are:

* $U_{DC} = 40 V$
* $R_{LOAD} = 30 \Omega$.


The voltage regulation will be done by a proportional resonant regulator.
This component is provided by the OwnTech control library which must be included 
in the file `src/owntech.ini`.

```
lib_deps=
    control_lib = https://github.com/owntech-foundation/control_library.git
```

The voltage source is defined by the voltage difference: $U_{12} = V_{1low} - V_{2low}$.

Link with the duty cycle:

* The leg1 is fixed in buck mode then: $V_{1low} = \alpha_1 . U_{DC}$
* The leg2 is fixed in boost mode then: $V_{2low} = (1-\alpha_2) . U_{DC}$

We change at the same time $\alpha_1$ and $\alpha_2$, then we have : $\alpha_1 = \alpha_2 = \alpha$. <br>
And then: $U_{12} = (2.\alpha - 1).U_{DC}$

$\alpha = \dfrac{U_{12}}{2.U_{DC}}  + 0.5$

