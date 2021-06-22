import numpy as np

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

def getRotMat(q):
    c00 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    c01 = 2 * (q[1] * q[2] - q[0] * q[3])
    c02 = 2 * (q[1] * q[3] + q[0] * q[2])
    c10 = 2 * (q[1] * q[2] + q[0] * q[3])
    c11 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    c12 = 2 * (q[2] * q[3] - q[0] * q[1])
    c20 = 2 * (q[1] * q[3] - q[0] * q[2])
    c21 = 2 * (q[2] * q[3] + q[0] * q[1])
    c22 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    rotMat = np.array([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]])
    return rotMat

def getEulerAngles(q):
    m = getRotMat(q)
    test = -m[2, 0]
    if test > 0.9999999999:
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(m[0, 1], m[0, 2])
    elif test < -0.9999999999:
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-m[0, 1], -m[0, 2])
    else:
        yaw = np.arctan2(m[1, 0], m[0, 0])
        pitch = np.arcsin(-m[2, 0])
        roll = np.arctan2(m[2, 1], m[2, 2])

    yaw = rad2deg(yaw) 
    pitch = rad2deg(pitch)
    roll = rad2deg(roll)

    return yaw, pitch, roll

def normalize_quat(q):
        mag = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)**0.5
        return q / mag

def S_omega(omega):
    return np.array([   [0, -omega[0], -omega[1], -omega[2]],
                        [omega[0], 0 , omega[2], -omega[1]],
                        [omega[1], -omega[2], 0 , omega[0]],
                        [omega[2], omega[1], -omega[0], 0]])

def S_quat(q):
    return np.array([   [-q[1], -q[2], -q[3]],
                        [q[0], -q[3], q[2]],
                        [q[3], q[0], -q[1]],
                        [-q[2], q[1], q[0]]])
                        

class EKF():

    def __init__(self, 
                accelref = [0, 0, -1], 
                bias = [0,0,0], 
                dt = 1, 
                qqgain = 0.01,
                qbgain = 0.01, 
                rgain = 0.1):

        quaternion = np.array([1, 0, 0, 0])     # Initial estimate of the quaternion
        self.xHat = np.concatenate((quaternion, bias)).transpose()

        self.yHatBar = np.zeros(3).transpose()
        self.Pk = np.identity(7) * 0.01

        self.Q = np.concatenate((np.concatenate((np.identity(4) *qqgain, np.zeros((4,3))), axis=1),
                        np.concatenate((np.zeros((3, 4)), np.identity(3) * qbgain), axis=1)), 
                        axis=0)

        self.R = np.identity(3) * rgain

        self.dt = dt

        self.xHatBar = None
        self.PkBar = None
        self.xHatPrev = None
        self.K = None

        self.A = None
        self.B = None
        self.C = None
        

        self.accelref = np.array(accelref).transpose()


    def set_step(self, ndt):
        self.dt = ndt

    #predict funcs
    def A_matrix(self):
        return np.concatenate(( np.concatenate((np.identity(4), -self.dt / 2 * S_quat(self.xHat[0:4])), axis=1),
                                np.concatenate((np.zeros((3, 4)), np.identity(3)), axis=1)), 
                                axis=0)

    def B_matrix(self):
        return np.concatenate((self.dt / 2 * S_quat(self.xHat[0:4]), np.zeros((3, 3))), axis=0)


    def get_xHatBar(self, omega):
        xHatBar = np.matmul(self.A, self.xHat) + np.matmul(self.B, np.array(omega).transpose())
        xHatBar[0:4] = normalize_quat(xHatBar[0:4])
        return xHatBar

    def get_PkBar(self):
        return np.matmul(np.matmul(self.A, self.Pk), self.A.transpose()) + self.Q

    def predict(self, omega):

        self.A = self.A_matrix()
        self.B = self.B_matrix()

        self.xHatBar = self.get_xHatBar(omega)
        self.xHatPrev = self.xHat
        self.PkBar = self.get_PkBar()


    #update funcs
    def getJacobianMatrix(self, qHatPrev, ref):

        e00 = qHatPrev[0] * ref[0] + qHatPrev[3] * ref[1] - qHatPrev[2] * ref[2]
        e01 = qHatPrev[1] * ref[0] + qHatPrev[2] * ref[1] + qHatPrev[3] * ref[2]
        e02 = -qHatPrev[2] * ref[0] + qHatPrev[1] * ref[1] - qHatPrev[0] * ref[2]
        e03 = -qHatPrev[3] * ref[0] + qHatPrev[0] * ref[1] + qHatPrev[1] * ref[2]
        e10 = -qHatPrev[3] * ref[0] + qHatPrev[0] * ref[1] + qHatPrev[1] * ref[2]
        e11 = qHatPrev[2] * ref[0] - qHatPrev[1] * ref[1] + qHatPrev[0] * ref[2]
        e12 = qHatPrev[1] * ref[0] + qHatPrev[2] * ref[1] + qHatPrev[3] * ref[2]
        e13 = -qHatPrev[0] * ref[0] - qHatPrev[3] * ref[1] + qHatPrev[2] * ref[2]
        e20 = qHatPrev[2] * ref[0] - qHatPrev[1] * ref[1] + qHatPrev[0] * ref[2]
        e21 = qHatPrev[3] * ref[0] - qHatPrev[0] * ref[1] - qHatPrev[1] * ref[2]
        e22 = qHatPrev[0] * ref[0] + qHatPrev[3] * ref[1] - qHatPrev[2] * ref[2]
        e23 = qHatPrev[1] * ref[0] + qHatPrev[2] * ref[1] + qHatPrev[3] * ref[2]

        jacobianMatrix = 2 * np.array([[e00, e01, e02, e03],
                                       [e10, e11, e12, e13],
                                       [e20, e21, e22, e23]])
        return jacobianMatrix

    def getAccelVector(self, a):
        accel = np.array(a).transpose()
        accelMag = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5
        return accel / accelMag

    def C_matrix(self):
        return np.concatenate((self.getJacobianMatrix(self.xHatPrev[0:4], self.accelref), np.zeros((3, 3))), axis=1)

    def get_K(self):
        tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.PkBar), self.C.transpose()) + self.R)
        K = np.matmul(np.matmul(self.PkBar, self.C.transpose()), tmp1)
        return K

    def get_xHat(self , K, a ):
        m = self.getAccelVector(a)
        zk = np.matmul(getRotMat(self.xHatBar).transpose(), self.accelref)
        xHat = self.xHatBar + np.matmul(K, m - zk)
        xHat[0:4] = normalize_quat(xHat[0:4])
        return xHat

    def get_Pk(self, K ):
        Pk = np.matmul(np.identity(7) - np.matmul(K, self.C), self.PkBar)
        return Pk
        
    def update(self,a):

        self.C = self.C_matrix()

        self.K = self.get_K()
        self.xHat = self.get_xHat(self.K, a)
        self.Pk = self.get_Pk(self.K)
        
