import cv2
import numpy as np
from math import pow , atan2,sqrt , degrees,asin

from .Clock_Node import Clock

class MotionPlanning():
    KP = [0.02,0.15]
    KI = [0.000001,0.001]
    KD = [0.001,0.1]

    E_I = [0,0]
    E_D = [0,0]

    MIN_ERROR = 10

    def __init__(self):
        ######################################################################
        self.Counter = 0

        self.Initial_Position = 0
        self.Init_Angle = 0
        self.Bot_angle = 0
        self.Bot_angle_s = 0
        self.Angle_Relation = 0
        self.Angle_relation_computed = False

        self.is_Goal_Reached = False
        self.Goal_pose = (0,0)
        self.Goal_iteration = 0

        self.Clock = Clock()
        self.prev_time = self.Clock.Get_time()

        self.Done = False
        self.Bot_Path = []


    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians
  
    def Calculate_Pose(self,data):

        # We get the bot_turn_angle in simulation Using same method as Gotogoal.py
        quaternions = data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        yaw_deg = degrees(yaw)

        # [Maintaining the Consistency in Angle Range]
        if (yaw_deg>0):
            self.Bot_angle_s = yaw_deg
        else:
            # -160 + 360 = 200, -180 + 360 = 180 . -90 + 360 = 270
            self.Bot_angle_s = yaw_deg + 360
        
        #              Bot Rotation 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]

    @staticmethod
    def bck_to_orig(pt,transform_arr,rot_mat):

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H
        
        # point --> (col(x),row(y)) XY-Convention For Rotation And Translated To MazeCrop (Origin)
        #pt_array = np.array( [pt[0]+st_col, pt[1]+st_row] )
        pt_array = np.array( [pt[0], pt[1]] )
        
        # Rot Matrix (For Normal XY Convention Around Z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T# [x,y]
        
        # Translating Origin If neccasary (To get whole image)
        rot_cols = tot_cols#tot_rows
        rot_rows = tot_rows#tot_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) ) + st_col  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) ) + st_row 
        return rot_center

    @staticmethod
    def Calculate_Angle(pt_a,pt_b):
        # Trignometric rules Work Considering.... 
        #
        #       [ Simulation/Normal Convention ]      [ Image ]
        #
        #                    Y                    
        #                     |                     
        #                     |___                     ____ 
        #                          X                  |     X
        #                                             |
        #                                           Y
        #
        # Solution: To apply same rules , we subtract the (first) point Y axis with (Second) point Y axis
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_a[1] - pt_b[1]

        # Calculating distance between two points
        distance = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        # Calculating angle between two points [Output : [-Pi,Pi]]
        angle = atan2(error_y,error_x)
        # Converting angle from radians to degrees
        angle_deg = degrees(angle)

        if (angle_deg>0):
            return (angle_deg),distance
        else:
            # -160 +360 = 200, -180 +360 = 180,  -90 + 360 = 270
            return (angle_deg + 360),distance
        
        #             Angle bw Points 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]

    @staticmethod
    def Get_Distance(ptA, ptB):
        return sqrt( pow(ptB[0]-ptA[0],2) + pow(ptB[1]-ptA[1],2) )

    def PID(self,Position):
        
        Angle_Goal, Error_distance = self.Calculate_Angle(Position, self.Goal_pose)
        Angle_turn = Angle_Goal - self.Bot_angle
        
        E_x = self.Goal_pose[0] - Position[0] 
        E_y = self.Goal_pose[1] - Position[1]

        time_ = self.Clock.Get_time()

        dt = time_-self.prev_time

        self.E_I[0] = Error_distance*dt + self.E_I[0]
        self.E_I[1] = Angle_turn*dt + self.E_I[1]

        Der_E_D = (Error_distance - self.E_D[0])/dt
        Der_E_A = (Angle_turn - self.E_D[1])/dt

        U_x = self.KP[0] * Error_distance + self.E_I[0] * self.KI[0] + Der_E_D * self.KD[0] 
        U_z = self.KP[1] * Angle_turn + self.E_I[1] * self.KI[1] + Der_E_A * self.KD[1] 

        if(U_x > 4):
            U_x = 4.0
        elif(U_x < -4):
            U_x = -4.0

        if(U_z > 4):
            U_z = 4.0
        elif(U_z < -4):
            U_z = -4.0

        print("U_x: {}, P: {}, G:{}".format(U_x, Position, self.Goal_pose))
        print("U_z: {}".format(U_z))
        print("Bot: {}, goal: {}, E: {}, sabe: {}\n".format(int(self.Bot_angle), int(Angle_Goal), int(Angle_turn), degrees(atan2(E_y, E_x))))

        self.E_D[0] = Error_distance
        self.E_D[1] = Angle_turn
        self.prev_time = time_

        return U_x, U_z

    def Get_NextPosition(self, Path, Position):
        if self.Goal_iteration + 2 < len(Path):
            self.Goal_iteration+=1
            if self.Get_Distance(Path[self.Goal_iteration],Path[self.Goal_iteration+1]) < self.MIN_ERROR:
                
                self.Get_NextPosition(Path,Position)
        else:
            self.Done = True

    def Go_2_Goal(self, Position, Path, Velocity, Velocity_Pub):
        
        if not self.Done:
            if self.Get_Distance(Position, Path[self.Goal_iteration]) < self.MIN_ERROR:
                self.Get_NextPosition(Path, Position)

            self.Goal_pose = Path[self.Goal_iteration]      
            
            U_x, U_z = self.PID(Position)
            self.Bot_Path.append(Position)

            Velocity.linear.x = U_x
            Velocity.angular.z = U_z
            Velocity_Pub.publish(Velocity)
        else:
            Velocity.linear.x = 0.0
            Velocity.angular.z = 0.0
            Velocity_Pub.publish(Velocity)

            print("Done")

    def Bot_Navigation(self, Position, Path, Velocity, Velocity_Pub):

        if self.Goal_iteration == 0:
            self.Goal_iteration = 5
            self.Goal_pose = Path[self.Goal_iteration]

        if self.Counter > 20:

            if not self.Angle_relation_computed:
                Velocity.linear.x = 0.0
                Velocity_Pub.publish(Velocity)

                self.Bot_angle, _ = self.Calculate_Angle(self.Initial_Position, Position) 
        
                self.Init_Angle = self.Bot_angle

                self.Angle_Relation = self.Bot_angle_s - self.Bot_angle
                self.Angle_relation_computed = True
                self.prev_time = self.Clock.Get_time()
            
            else:
                self.Bot_angle = self.Bot_angle_s - self.Angle_Relation

                self.Go_2_Goal(Position, Path, Velocity, Velocity_Pub)
                #print("Im Car Angle: {}, Relation: {}".format(self.Bot_angle,self.Angle_Relation ))
                #print("Sim Angle: {}, Position: {}".format(self.Bot_angle_s,Position ))
                #print("Initial: {} \n".format(self.Init_Angle))


        else:
            
            if type(self.Initial_Position) == int:
                self.Initial_Position = Position
            
            Velocity.linear.x = 0.2
            Velocity_Pub.publish(Velocity)
            
            self.Counter += 1
    
    def Display(self, frame_display, Path, Bot_Localizer):

        if not self.Done:
            x,y = self.Goal_pose
            x,y = self.bck_to_orig((x,y), Bot_Localizer.Transfor_arr, Bot_Localizer.rot_mat_rev)

            frame_display = cv2.circle(frame_display, (int(x),int(y)), 2, (0,0,255), 2)
        else:
            for path in self.Bot_Path:
                x,y = path

                x,y = self.bck_to_orig((x,y), Bot_Localizer.Transfor_arr, Bot_Localizer.rot_mat_rev)
                frame_display = cv2.circle(frame_display, (int(x),int(y)), 1, (0,0,255), 2)

        cv2.imshow("FrameDisplay", frame_display)
        cv2.waitKey(1)