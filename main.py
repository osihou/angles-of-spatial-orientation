from os import error
import tests as t
import csv
import ekf.EKF as EKF
import numpy as np
import errors


def read_case(inpath, outpath, ekf, params):

    start_time = 0

    g = 9.80665

    file_writer = csv.writer(open(outpath, mode='w'), delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    file_reader = csv.reader(open(inpath) , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    line_count = 0

    for row in file_reader:
        if line_count == 0:
            print('------------------------------')
            old_time = 0
        else:

            time = float(row[params[0]])

            if(time >= start_time):

                dt = time - old_time

                ekf.set_step(dt)

                old_time = time

                Ax = float(row[params[1]])
                Ay = float(row[params[2]])
                Az = float(row[params[3]])

                
                P = float(row[params[4]])
                Q = float(row[params[5]])
                R = float(row[params[6]])

                rangles = (float(row[3]), float(row[1]) , float(row[2]))

                ekf.predict([P , Q , R ])
                ekf.update([Ax, Ay, Az])

                num = EKF.getEulerAngles(ekf.xHat[0:4])

                rerror = errors.relative_error(rangles, num)
                cerror = errors.cape_error(rangles,num)

                print('------------------------------')
                print('Time: %.5f sek; dt: %.5f sek' %(time, dt))
                print('Gx: %.5f rad/s; Gy: %.5f rad/s; Gz: %.5f rad/s' % (P, Q, R))
                print('Ax: %.5fg; Ay: %.5fg; Az: %.5fg' % (Ax, Ay, Az))
                print('Yaw: %.5f; Pitch: %.5f; Roll: %.5f' % num)
                print('RYaw: %.5f; RPitch: %.5f; RRoll: %.5f' %  rangles)
                print('EYaw: %.5f; EPitch: %.5f; ERoll: %.5f' %   cerror )

                file_writer.writerow([time] +[dt] +list((Ax, Ay, Az)) + list((P, Q, R))+  list(num) + list(rangles) + list(cerror)+list(rerror) )

                if(line_count > 2000): break

            
        line_count += 1



if __name__ == '__main__':

    ekf = EKF.EKF(dt = 0.005, qqgain=0.01, qbgain=1, rgain=0.01)

    read_case("data/testdata2.csv","data/testout2.csv",ekf, [0, 4, 5, 6, 7, 8, 9 ] )

