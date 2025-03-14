#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray


room_size=15
grid_size=50
x_lin=np.linspace(0,room_size,grid_size)
y_lin=np.linspace(0,room_size,grid_size)
xx,yy=np.meshgrid(x_lin,y_lin)
AP_positions=np.array([[1.5,8],[5,5.5],[8.56,2.34],[2,2.57]])

def pathloss_rssi(x,y,ap_x,ap_y):
    d=np.sqrt((x-ap_x)**2+(y-ap_y)**2)+1e-3
    return -20.0*np.log10(d)
def interference(x,y):
    d=np.sqrt(x**2+y**2)
    return np.where(d<2.0,-5.0,0.0)

maps=[]
for ap in AP_positions:
    rssi_map=pathloss_rssi(xx,yy,ap[0],ap[1])
    rssi_map+=interference(xx,yy)
    rssi_map=np.clip(rssi_map,-100,0)
    maps.append(rssi_map)
pub=rospy.Publisher("rssi_simulation",Float32MultiArray,queue_size=10)
rospy.init_node("RSSI_Simulation_House")
rate=rospy.Rate(0.2)
while not rospy.is_shutdown():
    data=np.array(maps).flatten()
    msg=Float32MultiArray()
    msg.data=data.tolist()
    pub.publish(msg)
    rate.sleep()
