## Title O3D Create Test Data
#
#   Purpose: This example Python program simulates data received from a
#   sensor by writing the expected format of example data to a file. The
#   visualization will be demonstrated using a Python module called Open3D.
#
#   Special notes:
#       1. Open3D only works with Pythons 3.6-3.9.  It does not work with 3.10
#       2. For this eample you should run it in IDLE.  Anaconda/Conda/Jupyter
#       require different Open3D graphing methods (these methods are poorly documented)
#       3. Under Windows 10 you may need to install the MS Visual C++ Redistributable bundle
#           https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170
#       4. VirtualBox does not support OpenGL. If you're running Windows under Virtualbox or
#       your system doesn't support OpenGL (very rare), then you can install an OpenGL emulator dll
#           https://fdossena.com/?p=mesa/index.frag (unzip and copy opengl32.dll into Python dir)
#
#   T. Doyle
#   March 18, 2022 (Updated 2020 example)


import numpy as np
import open3d as o3d
import serial
import math
import time
s = serial.Serial('COM3', baudrate=115200, timeout=10)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode()) #encode the character into the bytes format  


f = open("tof_radar.xyz", "w")    #create a new file for writing 

for i in range(9):
    l = s.readline()
    print(l.decode())
    

angle = 11.25*math.pi/180 #adjust for 32 

#user input for number of yz slices
count = int(input("Enter number of slices:"))


for j in range(count):
    
    x=int(input("Enter x displacement:"))#take user input for x displacement

    print("Press Start Button")
    time.sleep(7)

    #read measurements 
    for i in range(32): #loop for 32 measurements. have to run everytime 
        d = s.readline()
        string = d.decode()
        pt = string.split("\r")[0]

        if pt == "STOP":
            break; 

        dNum = int(pt)
        y = dNum*math.cos(angle)
        z = dNum*math.sin(angle)

        f.write((str(x)+" "+str(y)+" "+str(z)+"\n").format(x))
        print(string)
        angle+=11.25*math.pi/180

        
          

f.close()
      
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])





#OK, good, but not great, lets add some lines to connect the vertices
#   For creating a lineset we will need to tell the packahe which vertices need connected
#   Remember each vertex actually contains one x,y,z coordinate
##
###Give each vertex a unique number
##yz_slice_vertex = []
##for x in range(0,40):
##    yz_slice_vertex.append([x])
##
###Define coordinates to connect lines in each yz slice        
##lines = []  
##for x in range(0,40,4):
##    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
##    lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
##    lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
##    lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x]])
##
###Define coordinates to connect lines between current and next yz slice        
##for x in range(0,35,4):
##    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+4]])
##    lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+5]])
##    lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+6]])
##    lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+7]])
 
total=32*count
yz_slice_vertex = []
for x in range(0, total):
    yz_slice_vertex.append([x])

# Define coordinates to connect lines in each yz slice
lines = []

for x in range(0, total, 32):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 1]])
    lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 2]])
    lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 3]])
    lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 4]])
    lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 5]])
    lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 6]])
    lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 7]])
    lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 8]])
    lines.append([yz_slice_vertex[x + 8], yz_slice_vertex[x + 9]])
    lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 10]])
    lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 11]])
    lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 12]])
    lines.append([yz_slice_vertex[x + 12], yz_slice_vertex[x + 13]])
    lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 14]])
    lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 15]])
    lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 16]])
    lines.append([yz_slice_vertex[x + 16], yz_slice_vertex[x + 17]])
    lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 18]])
    lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 19]])
    lines.append([yz_slice_vertex[x + 19], yz_slice_vertex[x + 20]])
    lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 21]])
    lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 22]])
    lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 23]])
    lines.append([yz_slice_vertex[x + 23], yz_slice_vertex[x + 24]])
    lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 25]])
    lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 26]])
    lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 27]])
    lines.append([yz_slice_vertex[x + 27], yz_slice_vertex[x + 28]])
    lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 29]])
    lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 30]])
    lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 31]])
    lines.append([yz_slice_vertex[x + 31], yz_slice_vertex[x]])

# Define coordinates to connect lines between current and next yz slice
# total numner of slices-the number of x horizotnal slices
for x in range(0, total-32, 32):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 32]])
    lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 33]])
    lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 34]])
    lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 35]])
    lines.append([yz_slice_vertex[x+ 4], yz_slice_vertex[x + 36]])
    lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 37]])
    lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 38]])
    lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 39]])
    lines.append([yz_slice_vertex[x+ 8], yz_slice_vertex[x + 40]])
    lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 41]])
    lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 42]])
    lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 43]])
    lines.append([yz_slice_vertex[x+ 12], yz_slice_vertex[x + 44]])
    lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 45]])
    lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 46]])
    lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 47]])
    lines.append([yz_slice_vertex[x+ 16], yz_slice_vertex[x + 48]])
    lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 49]])
    lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 50]])
    lines.append([yz_slice_vertex[x+ 19], yz_slice_vertex[x + 51]])
    lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 52]])
    lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 53]])
    lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 54]])
    lines.append([yz_slice_vertex[x+ 23], yz_slice_vertex[x + 55]])
    lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 56]])
    lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 57]])
    lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 58]])
    lines.append([yz_slice_vertex[x+ 27], yz_slice_vertex[x + 59]])
    lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 60]])
    lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 61]])
    lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 62]])
    lines.append([yz_slice_vertex[x+ 31], yz_slice_vertex[x + 63]])



 # This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                    lines=o3d.utility.Vector2iVector(lines))

# Lets see what our point cloud data with lines looks like graphically
o3d.visualization.draw_geometries([line_set])

#lines = []
#This line maps the lines to the 3d coordinate vertices
#line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
#o3d.visualization.draw_geometries([line_set])

   
#close the port
print("Closing: " + s.name)
s.close()
                                    
    
 
