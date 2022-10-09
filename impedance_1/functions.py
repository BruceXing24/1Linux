# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/9 19:14
import os
import pybullet as p


Force = 500
Max_velocity = 3
Position_Gain = 1
Velocity_Gain = 1


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






