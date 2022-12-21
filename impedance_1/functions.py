# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/9 19:14
import os
import pybullet as p
import numpy as np

Force = 500
Max_velocity = 3
Position_Gain = 1
Velocity_Gain = 1


def IK_2D(self, x, z):
    cos_theta3 = (x ** 2 + z ** 2 - (self.l2 ** 2 + self.l3 ** 2)) / (2 * self.l2 * self.l3)
    theta3_rad = np.arccos(cos_theta3)
    cos_alpha1 = (self.l2 ** 2 + x ** 2 + z ** 2 - self.l3 ** 2) / (2 * self.l2 * np.sqrt(x ** 2 + z ** 2))
    alpha1 = np.arccos(cos_alpha1)
    theta2_rad = alpha1 - np.arctan2(x, z)
    # print("alpha=={},alpha2=={}".format(np.rad2deg(alpha1), np.rad2deg(np.arctan2(x, z))))
    return theta2_rad, theta3_rad


def positions_control(body_name,LF, RF ,LH, RH):
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=0,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LF[0],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LF[1],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=2,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LF[2],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=3,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RF[0],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=4,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RF[1],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=5,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RF[2],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=6,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LH[0],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )

    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=7,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LH[1],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )

    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=8,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=LH[2],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )
    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=9,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RH[0],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )

    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=10,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RH[1],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )

    p.setJointMotorControl2(bodyIndex=body_name,
                            jointIndex=11,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=RH[2],
                            positionGain=Position_Gain,
                            velocityGain=Velocity_Gain,
                            force=Force,
                            maxVelocity=Max_velocity
                            )






