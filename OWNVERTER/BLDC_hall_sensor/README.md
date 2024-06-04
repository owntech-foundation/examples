In a BLDC (Brushless DC) motor, the rotor's position is typically divided into six sectors, each corresponding to a specific state of the Hall effect sensors. The control of the motor is achieved by appropriately energizing the stator windings in a sequence that creates a rotating magnetic field, which in turn causes the rotor to follow. The six sectors ensure that the motor phases are commutated correctly.

Here is a detailed explanation of the six sectors and the switching sequence for each sector. 

### Six Sectors of BLDC Motor

The six sectors are determined based on the Hall effect sensor outputs, which change as the rotor rotates. The Hall sensors provide three signals (H1, H2, H3), which can be interpreted as a 3-bit binary code. Each unique combination of these signals corresponds to one of the six sectors.

It means that H1 is the first bit, H2 is the second bit and H3 is the third bit. 

In code this is done with :  

```hall_state = hall1_value + 2*hall2_value + 4*hall3_value; ```

### Controling the motor

Voltage applied to the motor windings is proportional to the duty cycle.  
 
```Vwindings = 2 * (Duty_cycle - 0.5) * Vdc```

In this example duty_cycle can be increased or lowered by pressing `u` and `d` in the serial monitor. 
It should directly affect motor speed.

### Commutation Sequence

The commutation sequence involves energizing pairs of motor phases (A, B, C) while leaving one phase floating. This creates a magnetic field that moves the rotor to the next position. The table below shows the state of the switches for each sector. 

- "1" indicates the switch is on.
- "0" indicates the switch is off.
- The phases are labeled as A, B, and C.

#### Switch Notation
- U: Upper switch
- L: Lower switch
- A, B, C: Phases

### Phase implementation 

| Electrical Phase | PWM       | LEG       |
|------------------|-----------|-----------|
| Phase A          | PWMA      | LEG1       |
| Phase B          | PWMC      | LEG2       |
| Phase C          | PWME      | LEG3       |

### Commutation Table

| Hall Sensors | Sector | Switch U_A | Switch L_A | Switch U_B | Switch L_B | Switch U_C | Switch L_C |
|--------------|--------|------------|------------|------------|------------|------------|------------|
| 001          | 1      | 1          | 0          | 0          | 0          | 0          | 1          |
| 010          | 2      | 0          | 0          | 1          | 0          | 0          | 1          |
| 011          | 3      | 0          | 1          | 1          | 0          | 0          | 0          |
| 100          | 4      | 0          | 1          | 0          | 0          | 1          | 0          |
| 101          | 5      | 0          | 0          | 0          | 1          | 1          | 0          |
| 110          | 6      | 1          | 0          | 0          | 1          | 0          | 0          |

### Explanation of the Commutation Table

1. **Sector 1 (Hall: 001)**
   - U_A = 1: Upper switch of phase A is on.
   - L_B = 1: Lower switch of phase C is on.
   - Current flows from phase A to phase C.

2. **Sector 2 (Hall: 010)**
   - U_B = 1: Upper switch of phase B is on.
   - L_C = 1: Lower switch of phase C is on.
   - Current flows from phase B to phase C.

3. **Sector 3 (Hall: 011)**
   - L_C = 1: Lower switch of phase A is on.
   - U_B = 1: Upper switch of phase B is on.
   - Current flows from phase B to phase A.

4. **Sector 4 (Hall: 100)**
   - L_B = 1: Lower switch of phase A is on.
   - U_B = 1: Upper switch of phase C is on.
   - Current flows from phase C to phase A.

5. **Sector 5 (Hall: 101)**
   - L_B = 1: Lower switch of phase B is on.
   - U_C = 1: Lower switch of phase C is on.
   - Current flows from phase C to phase B.

6. **Sector 6 (Hall: 110)**
   - L_C = 1: Lower switch of phase C is on.
   - U_A = 1: Upper switch of phase A is on.
   - Current flows from phase A to phase C

This sequence ensures that the magnetic field rotates in such a way that the rotor is pulled along with it, allowing for smooth and efficient operation of the BLDC motor. Each transition between sectors corresponds to a change in the Hall sensor readings, which the control logic uses to determine the appropriate switches to energize.