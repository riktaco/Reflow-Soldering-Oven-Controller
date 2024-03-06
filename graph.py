import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, math
from playsound import playsound
import socket

xsize=500
y_list=[]

ser = serial.Serial(
    port='COM4',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()

last_val = None  # Initialize a variable to store the last value
color = 'red'
temp_type = 'Celcius'
state0flag = 0
state1flag = 0
state2flag = 0
state3flag = 0
state4flag = 0
title = 'Reflow Oven'


def serial_read(ser):
    global last_val, color, temp_type, title, state0flag, state1flag, state2flag, state3flag, state4flag  # Use the global variable to keep track across function calls
    while 1:
        strin = ser.readline()
        strin = strin.rstrip()
        strin = strin.decode()
        current_val = float(strin)  # Convert the current string to float

        if current_val >= 300:
            temp = current_val
            current_val = last_val
            if temp == 300:
                color = 'red'
                title = 'Ramp to Soak'
                state0flag = 1
            elif temp == 301:
                color = 'orange'
                title = 'Soak'
                state1flag = 1
            elif temp == 302:
                color = 'yellow'
                title = 'Ramp to Reflow'
                state2flag = 1
            elif temp == 303:
                color = 'green'
                title = 'Reflow'
                state3flag = 1
            elif temp == 304:
                color = 'blue'
                title = 'Cooling'
                state4flag = 1
            elif temp == 305:
                plt.savefig(r'C:\Users\rikta\OneDrive\Documents\UBC 2nd Year\ELEC291\plot.png')  # Save the figure
                # Optionally, stop the animation or close the plot if needed
                plt.close(fig)  # This will close the plot window if you're done with plotting
        
        # Check and print the trend
        if last_val is not None:  # Ensure last_val has been set at least once
            if current_val > last_val:
                print("Temperature is increasing")
            elif current_val < last_val:
                print("Temperature is decreasing")

            # If current_val == last_val, do nothing (temperature is stable)

        last_val = current_val  # Update last_val for the next iteration
        yield current_val

        
        
def data_gen():
    t = data_gen.t
    while True:
        t+=0.5
        val= next(serial_read(ser))
        yield t, val
        
def run(data):
    global state0flag, state1flag, state2flag, state3flag, state4flag
    # update the data
    t,y = data
    if t>-1:
        xdata.append(t)
        ydata.append(y)
        if t>xsize: # Scroll to the left.
            ax.set_xlim(t-xsize, t)
        line.set_data(xdata, ydata)
        line.set_color(color)


        ax.set_ylabel(temp_type)
        ax.set_title(title)
        fig.canvas.draw_idle()
        latest_temp_text.set_text(f'Latest Temp: {y:.2f} {temp_type}')
    return line,latest_temp_text

def on_close_figure(event):
    sys.exit(0)

data_gen.t = -1
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line, = ax.plot([], [], lw=2, color = color)
ax.set_ylim(0, 300)
ax.set_xlim(0, xsize)
ax.grid()
xdata, ydata = [], []
ax.set_xlabel('Time (s)')
ax.set_ylabel(temp_type)
ax.set_title(title)
latest_temp_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, verticalalignment='top')

# Important: Although blit=True makes graphing faster, we need blit=False to prevent
# spurious lines to appear when resizing the stripchart.
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100,repeat=False)
plt.show()
