"""
Copyright (c) 2021-2024 LAAS-CNRS

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

SPDX-License-Identifier: LGLPV2.1
"""

"""
@brief  This is a communcication script that deploys a PV emulator with
        hardware-in-the-loop with a Twitst 1.4.1

@author Luiz Villa <luiz.villa@laas.fr>
@author Thomas Walter <thomas.walter@laas.fr>
@author Guillaume Arthaud <guillaume.arthaud@laas.fr>
@author Amalie Alchami <amalie.alchami@utc.fr>
"""

import serial
import owntech.lib.USB.comm_protocol.src.find_devices as find_devices
from  owntech.lib.USB.comm_protocol.src.Twist_Class import Twist_Device

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import xmlrpc.client as xml
import time
import matplotlib.pyplot as plt
import numpy as np

leg_to_test = "LEG1"                               #leg to be tested in this script
reference_names = ["V1","V2","VH","I1","I2","IH"]  #names of the sensors of the board

twist_vid = 0x2fe3
twist_pid = 0x0101

Twist_ports = find_devices.find_twist_device_ports(twist_vid, twist_pid)
print(Twist_ports)

Twist = Twist_Device(twist_port= Twist_ports[0])


#Reference
# ----- Establsih communication with the PLCES software via Local Host ---------------------------------------------------------
plecs = xml.Server("http://localhost:1080/RPC2").plecs
model = 'PV_MPPT_Digital_Twin'                                               # PLECS Model Name
plecs.set(model+'/Sun', 'Value', str(1))                        # Set the Initial Irradiance to 1

count = np.linspace(0,22,10)                                    #creates a counter useful for traking IV curves

#-----------MPPT variables------------------------------------------------------------------------------------------------------
vmppt = 22                                                      # Initial V2 or VMPPT
sign = -1                                                       # Sign variable to be used in MPPT Equation
vmppt_step = 0.2                                                # delta V of the MPPT
Pnow = 0                                                        # Initial power at time t
Pbefore = 0                                                     # Initial power at time t-1

ref_base_value =1
ref_step = 0.01
ref_max_value = 1.5
reference = ref_base_value # initializes the reference

# ----- Create a figure and axis for an animation window -----------------------------------------------------------------------------------------------------------
fig, ax = plt.subplots()
line1, = ax.plot([], [], lw=2, label='I1')                      # Current I1 Plot
line2, = ax.plot([], [], lw=2, label='I2')                      # Current I2 Plot
line3, = ax.plot([], [], lw=2, label='VMPPT/10')                # VMPPT/10 Plot : Divide by 10 to respect figure limit range
line4, = ax.plot([], [], lw=2, label='Power/10')                # Power/10 Plot : Divide by 10 to respect figure limit range
frame_limit = 200                                               # Set the frame limit of the animation window
xdata, ydata1, ydata2, ydata3, ydata4 = [], [], [], [], []      # xdata : time, ydata1 : I1, ydata2 : I2, ydata3 : VMPPT/10,  ydata4 : Power/10

# Set up the plot parameters
ax.set_xlim(0, frame_limit)
ax.legend()
ax.set_ylim(-2, 7)
ax.set_xlabel('Time')
ax.set_ylabel('Value')
ax.set_title('Real-time Plot')

# Function to initialize the plot
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    return line1, line2, line3, line4

# Function to update the plot
def update(frame):

  global vmppt
  global sign
  global vmppt_step
  global Pnow
  global Pbefore

  if frame == frame_limit:                                      # Clear Plot when frame limit is reached
      xdata.clear()
      ydata1.clear()
      ydata2.clear()
      ydata3.clear()
      ydata4.clear()
      ax.set_xlim(frame, frame + frame_limit)

  else:

    xdata.append(frame)
    # Measure V1 and I1 of LEG1 to calculate P1
    Pnow = Twist.getMeasurement('V1')*Twist.getMeasurement('I1')

    # Calculate VMPPT by comapring the power and adjusting V
    if (Pnow<Pbefore) : sign = -sign                            # Change sign if Pnow is lower than Pbefore

    vmppt = vmppt + sign * vmppt_step
    Pbefore = Pnow                                              # Update Power

    Twist.sendCommand("REFERENCE","LEG2","V2",vmppt)            # Send Voltage reference to LEG 2

    time.sleep(10e-3)                                           # Pause to ensure stability

    v1_meas = Twist.getMeasurement('V1')                        # Measure V1 over LEG1

    plecs.set(model+'/Vref', 'Value', str(v1_meas))             # Send the measured voltage over LEG 1 to PLECS Software

    # Change Sun value to see the evolution of the model, than send this value to PLECS software

    if frame == 30 : plecs.set(model+'/Sun', 'Value', str(0.8))
    if frame == 50 : plecs.set(model+'/Sun', 'Value', str(0.6))
    if frame == 75 : plecs.set(model+'/Sun', 'Value', str(0.4))
    if frame == 100 : plecs.set(model+'/Sun', 'Value', str(0.6))
    if frame == 125 : plecs.set(model+'/Sun', 'Value', str(0.8))
    if frame == 150 : plecs.set(model+'/Sun', 'Value', str(1))

    # Run the Simulation on PLECS
    data = plecs.simulate(model)

    #Get the Simulation Time and Output
    values1 = data['Values'][0]

    # Calculate the average value of the last 10 data so that can be sent as a current reference to LEG1
    last_10_values1 = values1[-10:]
    average_values1 = sum(last_10_values1) / len(last_10_values1)

    # Send Current reference to LEG 1
    Twist.sendCommand("REFERENCE","LEG1","I1",average_values1)

    # Measurement to Plot in Real Time
    ydata1.append(Twist.getMeasurement('I1')  )                  # Append the I1 measure to ydata1
    ydata2.append(Twist.getMeasurement('I2'))                    # Append the I2 measure to ydata2
    ydata3.append(vmppt/10)                                      # Append the VMPPT to ydata3 : divide it by 10 to respect data range or axe limit compared to other values
    ydata4.append(Pnow/10)                                       # Append the Power to ydata4 : divide it by 10 to respect data range or axe limit compared to other values
    # Add to the line
    line1.set_data(xdata, ydata1)
    line2.set_data(xdata, ydata2)
    line3.set_data(xdata, ydata3)
    line4.set_data(xdata, ydata4)

  return line1, line2, line3, line4


# ---------------HARDWARE IN THE LOOP PV EMULATOR CODE ------------------------------------
message1 = Twist.sendCommand("IDLE")
print(message1)

message = Twist.sendCommand( "BUCK", "LEG1", "ON")
print(message)

message = Twist.sendCommand( "BUCK", "LEG2", "ON")
print(message)

message = Twist.sendCommand("LEG","LEG1","ON")
print(message)

message = Twist.sendCommand("LEG","LEG2","ON")
print(message)

message1 = Twist.sendCommand("REFERENCE","LEG1","I1",0.5)
print(message1)

message = Twist.sendCommand("POWER_ON")
print(message)


try:
  ani = animation.FuncAnimation(fig, update, frames=range(frame_limit), init_func=init, blit=True)
  plt.grid()
  plt.show()
finally:
  message1 = Twist.sendCommand("IDLE")
  print(message1)
