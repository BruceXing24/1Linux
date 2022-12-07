import pybullet as p
import numpy as np

class Leg():
    def __init__(self, name, shoulder2hip=0.047, hip2knee=0.1, knee2end=0.105):
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


    def InverseK(self,x,y,z):
        D = (x ** 2 + y ** 2 - self.l1 ** 2 + z ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
    
        theta3_rad = np.arctan2(-np.sqrt(1 - D ** 2), D)
    
        theta1_rad = -np.arctan2(z, y) - np.arctan2(np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2), -self.l1)
    
        theta2_rad = np.arctan2(-x, np.sqrt(y ** 2 + z ** 2 - self.l1 ** 2)) - np.arctan2(self.l3 * np.sin(theta3_rad),
                                                                                     self.l2 + self.l3 * np.cos(theta3_rad))
        return -theta1_rad, theta2_rad, theta3_rad



    def IK_R(self,x,y,z):
        D=(x**2+y**2+z**2-self.l1**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)
        theta3 = np.arctan2(-np.sqrt(1-D**2),D)
        theta1 = -np.arctan2(-z,-y)-np.arctan2(np.sqrt(y**2+z**2-self.l1**2),-self.l1)
        theta2 = np.arctan2(x,np.sqrt(y**2+z**2-self.l1**2))-np.arctan2(self.l3*np.sin(theta3),self.l2+self.l3*np.cos(theta3))


        return [theta1,-theta2,-theta3]

    def IK_L(self,x,y,z):

        D=(x**2+y**2+z**2-self.l1**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)

        theta3 = np.arctan2(-np.sqrt(1-D**2),  D)
        theta1 = -np.arctan2(-z,y)-np.arctan2(np.sqrt(y**2+z**2-self.l1**2),-self.l1)
        theta2 = np.arctan2(x,np.sqrt(y**2+z**2-self.l1**2))-np.arctan2(self.l3*np.sin(theta3),self.l2+self.l3*np.cos(theta3))

        return [-theta1,-theta2,-theta3]

    def IK_2D(self,x,z):
        cos_theta3 =( x**2+z**2-(self.l2**2+self.l3**2) )/( 2 * self.l2 * self.l3 )
        theta3_rad = np.arccos(cos_theta3)
        cos_alpha1 =( self.l2**2+x**2+z**2-self.l3**2)/(2 * self.l2 * np.sqrt(x**2+z**2))
        alpha1 = np.arccos(cos_alpha1)
        theta2_rad =  alpha1 - np.arctan2(x,z)

        print("alpha=={},alpha2=={}".format(np.rad2deg(alpha1), np.rad2deg( np.arctan2(x,z)  )))

        return -theta2_rad, theta3_rad




    def positions_control(self, body_name, LF, RF, LB, RB):

        self.joint_angle[0:3] += LF
        self.joint_angle[3:6] += RF
        self.joint_angle[6:9] += LB
        self.joint_angle[9: ] += RB

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex=2,
                                controlMode=p.POSITION_CONTROL,  targetPosition=-RB[0],
                                positionGain=self.Position_Gain, velocityGain=self.Velocity_Gain,
                                force=self.force,                maxVelocity=self.Max_velocity
                                )

        p.setJointMotorControl2(bodyIndex=body_name,
                                jointIndex = 3,
                                controlMode = p.POSITION_CONTROL,targetPosition = RB[1],
                                positionGain=self.Position_Gain, velocityGain=self.Velocity_Gain,
                                force=self.force,                maxVelocity=self.Max_velocity
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


    def positions_control2(self, body_name, LF, RF, LB, RB):
        # calibration for the urdf error


        LF[1] = LF[1] + np.deg2rad(3)
        # LF[2] = LF[2] - np.deg2rad(45)
        LF[2] = LF[2]-np.deg2rad(135)


        RF[1] = RF[1] - np.deg2rad(2)
        # RF[2] = RF[2] - np.deg2rad(40)
        RF[2] = RF[2] - np.deg2rad(140)


        LB[1] = LB[1] + np.deg2rad(3)
        # LB[2] = LB[2] - np.deg2rad(45)
        LB[2] = LB[2] - np.deg2rad(135)


        RB[1] = RB[1] + np.deg2rad(3)
        # RB[2] = RB[2] - np.deg2rad(45)
        RB[2] = RB[2] - np.deg2rad(135)

        self.joint_angle[0:3] += LF
        self.joint_angle[3:6] += RF
        self.joint_angle[6:9] += LB
        self.joint_angle[9: ] += RB


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
                                targetPosition=RB[2],
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
                                targetPosition=-LB[2],
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
                                targetPosition=-LF[2],
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
                                targetPosition=RF[2],
                                positionGain=self.Position_Gain,
                                velocityGain=self.Velocity_Gain,
                                force=self.force,
                                maxVelocity=self.Max_velocity
                                )
