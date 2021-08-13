import serial
import time
import numpy as np
import re
import matplotlib.pyplot as plt

ser = serial.Serial('COM9',baudrate= 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
print(ser.name)
loop = 202
coord_x = np.array([])
coord_y = np.array([])
coord_z = np.array([])

coord_x_2 = np.array([])
coord_y_2 = np.array([])
coord_z_2 = np.array([])
for i in range (loop):
    bytes = ser.readline()
    data = bytes.decode('UTF-8')
    print(bytes.decode('UTF-8'))
    data = re.split(",|\r\n",data)
    if (loop<=200):
        coord_x =  np.append(coord_x, float(data[0]))
        coord_y = np.append(coord_y, float(data[1]))
        coord_z = np.append(coord_z, float(data[2]))
        coord_x_2 = np.append(coord_x_2, float(data[3]))
        coord_y_2 = np.append(coord_y_2, float(data[4]))
        coord_z_2 = np.append(coord_z_2, float(data[5]))
    loop -= 1


# print((re.split(",|\r\n",data)))
print(type(coord_x[0]))
# print(coord_pitch)

t0 = 0 # Initial time
tf = 15 # Final time
N = int(200) # Numbers of points in time span
t = np.linspace(t0, tf, N) # Create time span
fig = plt.figure()

plt.xlim([t0, tf])
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
plt.grid(True)

plt.plot(t, coord_x, label ='X')
plt.plot(t, coord_y, label ='Y')
plt.plot(t, coord_z, label ='Z')

plt.title('Gyrosope\'s data 1 ')
plt.legend()
plt.show()



plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
plt.grid(True)
plt.plot(t, coord_x_2, label ='X')
plt.plot(t, coord_y_2, label ='Y')
plt.plot(t, coord_z_2, label ='Z')

plt.title('Gyroscope\'s data 2 ')
plt.show()
# import serial.tools.list_ports
# ports = list(serial.tools.list_ports.comports())
# for p in ports:
#     print(p)

# b'\xb2'
# b'7'
# b'\x93'
# b'\xaa'
# b'\r'
# b'\xe1'
# b'0'
# b'\xc9'
# b'\xb2'
