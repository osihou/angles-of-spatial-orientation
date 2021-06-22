import numpy as np
from operator import add

import quaternion as quat
import EKF as EKF
import vmath as vmath
import csv

import errors

def roundlst(lst,r):
    return list(map(lambda x : round(x, r), lst))

class Sensor:
    def __init__(self, 
                gnoise_mu = [0.01, 0.01, 0.01], 
                gnoise_std = [[0.001, 0, 0], [0, 0.001, 0], [0,0,0.001]],
                gbias = [0.0005 , 0.0005, 0.0005], 
                anoise_mu = [0.0001, 0.0001, 0.0001],
                anoise_std = [[0.00001,0,0], [0, 0.00001,0], [0,0,0.00001]],
                abias = [0.000001 , 0.000001 , 0.000001]):
                
        self.gnoise_mu = gnoise_mu
        self.gnoise_std = gnoise_std
        self.gbias = gbias
        
        self.anoise_mu = anoise_mu
        self.anoise_std = anoise_std
        self.abias = abias

    def gen_omega(self, omega, dt):

        x, y, z = np.random.multivariate_normal(self.gnoise_mu, self.gnoise_std ).T

        return [ omega[0] + omega[0] * x + self.gbias[0] * dt,
                 omega[1] + omega[1] * y + self.gbias[1] * dt , 
                 omega[2] + omega[2] * z + self.gbias[2] * dt]


    def gen_accel(self, a, dt):
        x, y, z = np.random.multivariate_normal(self.anoise_mu, self.anoise_std ).T

        return [ a[0] + x * a[0] + self.abias[0] * dt,
                 a[1] + y * a[1] + self.abias[1] * dt, 
                 a[2] + z * a[2] + self.abias[2] * dt]

    def get_from_omega(self, omega, bquat, dt):


        ans = quat.quaternion(bquat, omega)

        rangs = EKF.getEulerAngles(ans[0])

        fomega = self.gen_omega(omega, dt)


        raccel = vmath.nvec(rangs)

        faccel = self.gen_accel(raccel, dt)

        return (ans[0], fomega, faccel,raccel, rangs)


sim = Sensor() 

def loop_case_test(omega, steps):
    bquat = [1,0,0,0]

    ekf = EKF.EKF(dt = 1, qqgain=0.01, qbgain = 1, rgain=0.01)

    file_writer = csv.writer(open('data/test2.csv', mode='w'), delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    for dt in range(steps):

        bquat, fomega, faccel, raccel, rangs  = sim.get_from_omega(omega, bquat,dt)


        ekf.predict(fomega)

        ekf.update(faccel)

        num = EKF.getEulerAngles(ekf.xHat[0:4])

                
        cerror = errors.cape_error(rangs,num)
        omerror = errors.cape_error(omega, fomega)
        accerror = errors.cape_error(raccel, faccel)

        print('------------------------------')
        print('Yaw: %.5f; Pitch: %.5f; Roll: %.5f' % num)
        print('RYaw: %.5f; RPitch: %.5f; RRoll: %.5f' %  rangs)
        print('EYaw: %.5f; EPitch: %.5f; ERoll: %.5f' %   cerror )

        file_writer.writerow(list(rangs)+list(num)+list(cerror)+list(omerror)+list(accerror))



loop_case_test([1 * np.pi/180, 0 * np.pi/180,  0 * np.pi/180], 90)