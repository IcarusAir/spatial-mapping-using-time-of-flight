

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import open3d as o3d
import numpy as np
import math

frames = 2
steps = 32
s = serial.Serial('COM4',baudrate=115200,timeout=10)
                            
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())
data = []

# recieve characters from UART of MCU
for i in range(frames):
    x = s.readline()                    # read one byte
    x = x.decode()
    data.append(x.split(", "))
    print(x)

       
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()



#Use Open3D___________________________________________________________
#Create file to hold point cloud data
#data[] now holds the distance data for every frame
#_____________________________________________________________________

f = open("distanceDataFinal.xyz", "w")

for i in range(frames):
    for j in range(steps):
        distance = data[i][j]
        angle = (math.pi/16)*j
        x = 330*i
        y = round(int(distance)*math.sin(angle))
        z = round(int(distance)*math.cos(angle))
        f.write(str(x)+" "+str(y)+" "+str(z)+"\n")


f.close()
pcd = o3d.io.read_point_cloud('distanceDataFinal.xyz', format="xyz")

#Create the lines
verticies = []
for i in range(0,frames*steps):
    verticies.append([i])

lines = []
for i in range(0,frames*steps,steps):
    for j in range(steps):
        if j != steps-1:
            lines.append([verticies[i+j],verticies[i+j+1]])
        else:
            lines.append([verticies[i+j],verticies[i]])

for i in range(0,(frames-1)*steps,steps):
    for j in range(0,steps):
        lines.append([verticies[i+j],verticies[i+j+steps]])
    
        



line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np
        .asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])






