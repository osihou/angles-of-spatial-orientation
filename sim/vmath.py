import numpy as np

def CNB (yaw, pitch, roll):
    return np.array([[np.cos(yaw) * np.cos(pitch),  np.sin(yaw) * np.cos(pitch) ,   - np.sin(pitch)] ,
                    [np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll) , np.cos(pitch) * np.sin(roll)  ],
                    [np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll), np.cos(pitch) * np.cos(roll) ]])


def nvec(angles):
    return np.matmul(CNB(angles[0] * np.pi/180 ,angles[1] * np.pi/180 ,angles[2] * np.pi/180), [0,0,-1])

