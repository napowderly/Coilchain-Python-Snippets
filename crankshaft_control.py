#This aims at replicating the adaptive regenerative breaking fot Coilchain


import numpy as np
import matplotlib.pyplot as plt
from math import pi
import time

#Sampling rate and parameters init
dt = 0.1 #Sampling rate in s
prev_teta =0
teta = 0 #initial position of the crankshaft
prev_omega=0
omega = 0 #initial velocity of the crankshaft
alpha = 0
prev_alpha = 0

#Cyclist torque shape behaviour
a = 0.5
b = 1

prev_inputTorque = 0
inputTorque = 0 #input torque is cosine shaped and depend on crankshaft position

prev_brakeTorque = 0
brakeTorque = 0

#Simulated 1/inertia of the legs+generator system
inertia = 0.1


# initialize stored data
e_prev = 0
t_prev = -100
I = 0

#total simulation time
tfinal = 500

#Setup plot
xdata = []
ydata = []
ydata1 = []
ydata2 = []
ydata3 = [] 
 
plt.show()
 
axes = plt.gca()
axes.set_xlim(0, 500)
axes.set_ylim(-3, 3)
line, = axes.plot(xdata, ydata, 'r-')
line1, = axes.plot(xdata, ydata1, 'b-')
line2, = axes.plot(xdata, ydata2, 'c-')
line3, = axes.plot(xdata, ydata3, 'm-')

prev_ctrl = 0
ctrl = 0

#PID controller settings
Kp= 8
Ki= 0.1
Kd = 1
Kf = 0.5

e=0

for t in np.linspace(0, tfinal, tfinal/dt):

    #set angular velocity
    set_omega = 0.2 if t < 100 else 0.4#in rad/s 0.1rad/s ~= 1rpm

    # PID calculations
    e = set_omega - omega
    
    P = Kp*e
    I = I + Ki*e*dt
    D = Kd*(e - e_prev)/dt
    
    #feed-forward with respect to crankshaft position
    FF = - Kf * (np.cos(teta)+0.5)
    
    ctrl =  P + I + D + FF
    
    # update stored data for next iteration
    e_prev = e
    
    #update the system with the new computed control
    
    brakeTorque = -ctrl
    
    #cyclist torque on the crankshaft with respect to crankshaft position
    a = 0.5 if t < 200 else 1 
    b = 1 if t < 200 else 1.5 
    inputTorque = a*np.cos(teta) + b
    
    #the sum of all torques are equal to the inertia of the system times acceleration
    alpha = inertia * (inputTorque - brakeTorque) #Newton laws of torque
    
    #integration of angular velocity
    omega = prev_omega + alpha * dt 
    
    #integration of angular position
    teta = prev_teta + omega * dt
    
    #Update the data fields for the graphs
    xdata.append(t)
    ydata.append(inputTorque) #red
    ydata1.append(omega) #blue
    ydata2.append(brakeTorque) #cyan
    ydata3.append(set_omega) #magenta
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    line1.set_xdata(xdata)
    line1.set_ydata(ydata1)
    line2.set_xdata(xdata)
    line2.set_ydata(ydata2)
    line3.set_xdata(xdata)
    line3.set_ydata(ydata3)
    plt.draw()
    plt.pause(1e-17)
    time.sleep(0.01)
    
    #update system for next iteration
    prev_omega=omega
    prev_teta = teta
    

#keep plot at the end of the simulation
plt.show()


#use ctrl-C in terminal to quit