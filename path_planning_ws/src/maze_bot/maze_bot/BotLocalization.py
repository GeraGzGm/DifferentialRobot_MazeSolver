import cv2
import numpy as np

class Bot_Localizer():

    def __init__(self):
        self.is_bg_extracted = False

        self.Background = []
        self.Occupancy_grid = []

        #Transformations parameters 
        self.orig_X, self.orig_Y  = (0,0)
        self.orig_Row, self.orig_Col = (0,0)

        self.Transfor_arr = []

        self.orig_rot, self.rot_mat = (0,[])

        self.Bot_Location = []
   
    def FindMin_Contour(self, contour) -> int:
        min_idx, min_area = -1, 1000

        for idx, cnt in enumerate(contour):
            area = cv2.contourArea(cnt)

            if (area < min_area) and (area > 10):
                min_area = area
                min_idx = idx
        
        return min_idx
    
    @staticmethod
    def Crop_Maze(ROI_MASK, contours):
        maze_crop = np.zeros_like(ROI_MASK)

        if contours:
            contours_ = np.concatenate(contours)
            contours_ = np.array(contours_)

            cv2.fillConvexPoly(maze_crop, contours_, 255)
        
        contours_largest = cv2.findContours(maze_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        
        hull = cv2.convexHull(contours_largest[0])
        cv2.drawContours(maze_crop, [hull], 0, 255)

        #cv2.imshow("maze_crop", maze_crop)
        #cv2.waitKey(1)      

        return hull

    def Update_FrameParameters(self, X: int, Y: int, W: int, H: int, Rotation: float) -> None:
        self.orig_X = X
        self.orig_Y = Y
        self.orig_Row = H
        self.orig_Col = W
        self.orig_rot = Rotation

        self.Transfor_arr = [X,Y,W,H]

        self.rot_mat = np.array([[np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))], 
                                 [-np.sin(np.deg2rad(self.orig_rot)), np.cos(np.deg2rad(self.orig_rot))]])

        self.rot_mat_rev = np.array([[np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))], 
                                 [-np.sin(np.deg2rad(-self.orig_rot)), np.cos(np.deg2rad(-self.orig_rot))]])

    def Extract_Background(self, frame: np.array):

        frame_ = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_gauss = cv2.GaussianBlur(frame_, (3,3), cv2.BORDER_CONSTANT)

        #Obtain edges from the frame
        Edges = cv2.Canny(image = frame_gauss, threshold1 = 60, threshold2 = 150, edges = None, apertureSize = 3)
        Edges = cv2.dilate(Edges, kernel=(3,3), iterations=2)

        #Obtain the multiple contours
        Contours = cv2.findContours(Edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        ROIS_mask = np.zeros((frame.shape[0],frame.shape[1]), dtype=np.uint8)

        for idx,cnt in enumerate(Contours):
            cv2.drawContours(ROIS_mask, Contours, idx, 255, -1)

        Background_noCar = ROIS_mask.copy()
        Background_onlyCar = np.zeros_like(Background_noCar)

        #Find minimum contour which is the car
        min_cnt_idx = self.FindMin_Contour(Contours)

        if min_cnt_idx != -1:
            cv2.drawContours(Background_noCar, Contours, min_cnt_idx, 0, -1)
            
            cv2.drawContours(Background_onlyCar, Contours, min_cnt_idx, 255, -1)
            cv2.drawContours(Background_onlyCar, Contours, min_cnt_idx, 255, 5)

            Mask_notCar = cv2.bitwise_not(Background_onlyCar)
            frame_CarRemoved = cv2.bitwise_and(frame, frame, mask=Mask_notCar)
            
            base_color = frame_CarRemoved[1][1]
            ground_ = np.ones_like(frame) * base_color
            self.Background = cv2.bitwise_and(ground_, ground_, mask = Background_onlyCar)
            self.Background = cv2.bitwise_or(self.Background , frame_CarRemoved)

        hull = self.Crop_Maze(ROIS_mask, Contours)
        [X,Y,W,H] = cv2.boundingRect(hull)
        
        cropped_maze = Background_noCar[Y:Y+H,X:X+W]
        Occupancy_grid = cv2.bitwise_not(cropped_maze)

        self.Occupancy_grid = cv2.rotate(Occupancy_grid, cv2.ROTATE_90_COUNTERCLOCKWISE)

        self.Update_FrameParameters(X,Y,W,H,90)

        #cv2.imshow("maze_crop", cropped_maze)
        #cv2.waitKey(1)   
         
        #cv2.imshow("Background no car and only car", np.concatenate((Background_noCar, Background_onlyCar), axis=1))
        #cv2.waitKey(1) 
        #cv2.imshow("occupancy_grid", self.Occupancy_grid)  
        #cv2.imshow("Background", self.Background)
        #cv2.waitKey(1)      
    
    @staticmethod
    def Calculate_Centroid(frame: np.array) -> tuple:
        M = cv2.moments(frame)
        
        x = M["m10"]/M["m00"]
        y = M["m01"]/M["m00"]

        return (int(x),int(y))

    def Get_BotLocalization(self, Frame_Forground: np.array) -> None:
        Bot_Col,Bot_Row = self.Calculate_Centroid(Frame_Forground)

        #Transform point
        Bot_transformation = np.array([Bot_Col,Bot_Row ])
        Bot_transformation[0] = Bot_transformation[0] - self.orig_X
        Bot_transformation[1] = Bot_transformation[1] - self.orig_Y

        Bot_Location = np.dot(self.rot_mat, Bot_transformation.T).T

        #Translate origin
        rot_col = self.orig_Row
        rot_row = self.orig_Col

        Bot_Location[0] = Bot_Location[0] + (rot_col * (Bot_Location[0]<0))
        Bot_Location[1] = Bot_Location[1] + (rot_row * (Bot_Location[1]<0))

        self.Bot_Location = Bot_Location.astype('int16')

    def Localize_bot(self, frame: np.array, frame_display: np.array):

        if not self.is_bg_extracted:
            self.Extract_Background(frame)
            self.is_bg_extracted = True
        
      
        Foreground = cv2.absdiff(frame, self.Background)
        Foreground = cv2.cvtColor(Foreground, cv2.COLOR_BGR2GRAY)
        Foreground = cv2.threshold(Foreground, 15, 255, cv2.THRESH_OTSU)[1]

        self.Get_BotLocalization(Foreground)

        bot_cnt = cv2.findContours(Foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

        center, radius = cv2.minEnclosingCircle(bot_cnt[0])
        Bot_CircularMask = cv2.circle(Foreground.copy(), 
                                        (int(center[0]),int(center[1])), int(radius), (255,0,0),1)
        Bot_CircularMask = cv2.bitwise_xor(Bot_CircularMask, Foreground)

        #frame_display[Foreground > 0] = frame_display[Foreground > 0] + (0,64,0)
        frame_display[Bot_CircularMask > 0] = (0,255,0)

        #cv2.imshow("Foreground", Foreground)
        #cv2.imshow("frame_display", frame_display)
        #cv2.waitKey(1)    

        return frame_display