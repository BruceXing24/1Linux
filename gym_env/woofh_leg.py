import pybullet as p
import numpy

class Leg():
    def __init__(self, name, shoulder2hip=0.1, hip2knee=0.1, knee2end=0.05 ):
        self.l1 = shoulder2hip
        self.l2 = hip2knee
        self.l3 = knee2end
        self.name = name
        self.Position_Gain = 1.0
        self.Velocity_Gain = 1.0
        self.force = 10
        self.Max_velocity = 5

        #                    LF      RF     LB    RB
        self.joint_angle  = [0,0,0, 0,0,0 ,0,0,0 ,0,0,0]


    def positions_control(self, body_name, LF, RF, LB, RB):

        self.joint_angle[0:3] += LF
        self.joint_angle[3:6] += RF
        self.joint_angle[6:9] += LB
        self.joint_angle[9:] += RB

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=2,
                                controlMode=p.POSITION_CONTROL,  targetPosition=-RB[0],
                                positionGain=self.Position_Gain, velocityGain=self.Velocity_Gain,
                                force=self.force,                 maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex = 3,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = RB[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=4,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=-RB[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=7 ,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=-LB[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=8 ,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LB[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=9,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LB[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=12,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LF[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=13,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=-LF[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=14,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=LF[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=17,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=RF[0],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=18,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=-RF[1],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=19,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=-RF[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )

