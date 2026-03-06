import math
import numpy as np
class RotationSolver: 
    #endregion Calibration
    def rot_x(self,r):
        return np.array([
            [1, 0, 0],
            [0, math.cos(r), -math.sin(r)],
            [0, math.sin(r),  math.cos(r)]
        ])

    def rot_y(self,r):
        return np.array([
            [ math.cos(r), 0, math.sin(r)],
            [0, 1, 0],
            [-math.sin(r), 0, math.cos(r)]
        ])

    def rot_z(self,r):
        return np.array([
            [math.cos(r), -math.sin(r), 0],
            [math.sin(r),  math.cos(r), 0],
            [0, 0, 1]
        ])

    def convertZYX_Zup_to_Xup(self,yaw, pitch, roll):

        yaw = math.radians(yaw)
        pitch = math.radians(pitch)
        roll = math.radians(roll)

        # ZYX rotation
        R = self.rot_z(yaw) @ self.rot_y(pitch) @ self.rot_x(roll)

        T = np.array([
            [0,0,1],
            [0,1,0],
            [-1,0,0]
        ])

        R_new = T.T @ R @ T

        pitch2 = -math.asin(R_new[2,0])
        roll2  = math.atan2(R_new[2,1], R_new[2,2])
        yaw2   = math.atan2(R_new[1,0], R_new[0,0])

        return (
            math.degrees(yaw2),
            math.degrees(pitch2),
            math.degrees(roll2)
        )