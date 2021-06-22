import serial
import time
from threading import Thread
import ekf.EKF as EKF
import numpy as np
import csv

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def disect_output(s):
    return([ float(x) for i, x in enumerate(str(s).replace('g', '').split(' '))  if i % 2 != 0 ])


def getAccelVector(a):
    accel = np.array(a).transpose()
    accelMag = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5
    return accel / accelMag

class SerialRead:
    def __init__(self, serial_port = '', serial_baud = ''):
        self.sp = serial_port
        self.sb = serial_baud

        self.thread = None

        self.rec = False    # IS RECEIVING
        self.run = True     # IS RUNNING

        self.file_writer = csv.writer(open('data/GandA.csv', mode='w'), delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        self.YAW = []
        self.PITCH = []
        self.ROLL = []
        self.TIME = []

        self.start_time = 0

        self.old_time = 0

        self.ekf = EKF.EKF(dt = 0.1, qqgain=0.001, qbgain=0.001, rgain=0.01)


        print('Trying to connect to: ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')

        try:
            self.sc = serial.Serial(serial_port, serial_baud, timeout=4)
            print('Connected to ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
            exit()


    def relax(self):
        self.YAW.pop(0)
        self.ROLL.pop(0)
        self.PITCH.pop(0)
        self.TIME.pop(0)

    def start_serial(self):
        if self.thread == None:
            self.thread = Thread(target=self.back_thread)
            self.thread.start()
            while self.rec != True:
                time.sleep(0.1)

    def process_output(self, output):

        new_time = time.time() - self.start_time

        dt = new_time - self.old_time

        self.ekf.set_step(dt)

        self.old_time = new_time

        
        self.ekf.predict([output[3]*np.pi/180, output[4]*np.pi/180, output[5]*np.pi/180])
        self.ekf.update([output[0], output[1], output[2]])
        num = EKF.getEulerAngles(self.ekf.xHat[0:4])


        print('------------------------------')
        print('Time: %.5f sek; dt: %.5f sek' %(new_time, dt))
        print('Gx: %.5f; Gy: %.5f; Gz: %.5f' % (output[3], output[4], output[5]))
        print('Ax: %.5f; Ay: %.5f; Az: %.5f' % (output[0], output[1], output[2]))
        print('Yaw: %.5f; Pitch: %.5f; Roll: %.5f' % num)

        self.YAW.append(num[0])
        self.PITCH.append(num[1])
        self.ROLL.append(num[2])
        self.TIME.append(new_time)

        
        self.file_writer.writerow([new_time] + list(output) + list(num))

    def back_thread(self):   
            time.sleep(1.0)  
            self.sc.reset_input_buffer()
            self.start_time = time.time()

            while (self.run):
                self.rec = True
                output = disect_output(self.sc.readline())
                #print(output)
                self.process_output(output)
                #print(self.ekf.xHat[4:7])

    def close(self):
        self.run = False
        self.thread.join()
        self.sc.close()
        print('DISCONNECTED')




def port_config():
    serial_port = '/dev/ttyACM0'
    #portName = 'COM6'
    serial_baud = 115200

    sr = SerialRead(serial_port,serial_baud)
    sr.start_serial()

    counter = 0

    def animate(i):
        
        ax1.plot(sr.TIME, sr.PITCH, linestyle='-',color='red', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=1,  linewidth=1)

        ax2.plot(sr.TIME, sr.ROLL, linestyle='-',color='red', markerfacecolor='black', marker='.', markeredgecolor="black", markersize=1,  linewidth=1)

        counter =+ 1

        
        sr.relax()
            
        time.sleep(.01)




    plt.style.use('bmh')
    fig = plt.figure(figsize = (10,10))

    #plt.ylim(-90,90)
    #plt.xlim(0,linecount)


    ax1 = plt.subplot(211)
    ax1.set_ylim(-90,90)
    ax1.set_ylabel('PITCH [deg.]')
    ax1.set_title('PITCH')

    ax2 = plt.subplot(212)
    ax2.set_ylim(-180,180)
    ax2.set_xlabel('time [sek.]')
    ax2.set_ylabel('ROLL [deg.]')
    ax2.set_title('ROLL')

    ani = FuncAnimation(plt.gcf(), animate, 10)

    plt.tight_layout()
    plt.show()



port_config()
