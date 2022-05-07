# ToF-Sensor-Project

**This Project was done for the course COMPENG 2DX3**

This system generates a 3D visualization of distance measurements sent by a VL531LX Time Of Flight (ToF) sensor connected to a MSP432E401Y microcontroller.
The UART communication line between the PC and MSP432E401Y microcontroller is established by running the python code. This initializes the ToF sensor and prompts the user to enter the current position of the device on the x-axis and the number of desired YZ slices that will be measured. Then, the push of the pull-up resistor (start button) triggers the stepper motor to begin rotating 360 degrees clockwise with the ToF sensor mounted. The ToF sensor measures the distance within a single vertical Y-Z plane and this data is sent to the MSP432E401Y microcontroller via I2C protocol. This process can be interrupted by pushing a second button. These measurements are then read by the PC and are stored as y, z coordinates. Using the Open3D library in Python, the spatial information is mapped and a 3D graph is generated. Figure below demonstrates an example of a simulated hallway with 6 YZ planes each with their own respective x coordinates. 

![image](https://user-images.githubusercontent.com/70678143/167233625-35417989-b455-422b-8cc6-0320f767e5ff.png)

