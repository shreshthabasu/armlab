"""!
Class to represent the kinect.
"""

import cv2
import csv
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import os
import math
import apriltag
script_path = os.path.dirname(os.path.realpath(__file__))

color_ranges = {
    "Blue": (np.array([42, 14, 126]), np.array([177, 49, 210])),
    }


class Kinect():
    """!
    @brief      This class describes a kinect.
    """

    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.VideoFrame = np.array([])
        self.DepthFrameRaw = np.array([]).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((480, 640, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])

        """initialize kinect & turn off auto gain and whitebalance"""
        freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)
        # print(freenect.sync_set_autoexposure(False))
        freenect.sync_set_autoexposure(False)
        # print(freenect.sync_set_whitebalance(False))
        freenect.sync_set_whitebalance(False)
        """check depth returns a frame, and flag kinectConnected"""
        if(freenect.sync_get_depth_with_res(format=freenect.DEPTH_11BIT) == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True

        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.array(list(csv.reader(open("depth2rgb_affine.cfg"))), dtype=float)
        self.kinectCalibrated = False
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)

        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])

        """camera calibration"""
        self.cam_int_mat = []  # np.array([])
        self.cam_ext_mat = []
        self.dist_coeffs = []

        """depth info"""
        self.max_depth = 0.95
        self.affine_transform = False

    def toggleExposure(self, state):
        """!
        @brief      Toggle auto exposure

        @param      state  False turns off auto exposure True turns it on
        """
        if state == False:
            freenect.sync_get_video_with_res(
                resolution=freenect.RESOLUTION_HIGH)
            # print(freenect.sync_set_autoexposure(False))
            freenect.sync_set_autoexposure(False)
            # print(freenect.sync_set_whitebalance(False))
            freenect.sync_set_whitebalance(False)
        else:
            freenect.sync_get_video_with_res(
                resolution=freenect.RESOLUTION_HIGH)
            # print(freenect.sync_set_autoexposure(True))
            freenect.sync_set_autoexposure(True)
            # print(freenect.sync_set_whitebalance(True))
            freenect.sync_set_whitebalance(True)

    def captureVideoFrame(self):
        """!
        @brief Capture frame from Kinect, format is 24bit RGB
        """
        if(self.kinectConnected):
            self.VideoFrame = freenect.sync_get_video_with_res(
                resolution=freenect.RESOLUTION_HIGH)[0]
            
        else:
            self.loadVideoFrame()
        self.processVideoFrame()

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(
            self.VideoFrame, self.block_contours, -1, (255, 0, 255), 3)

    def captureDepthFrame(self):
        """!
        @brief Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.DepthFrameRaw = self.registerDepthFrame(
                    freenect.sync_get_depth_with_res(format=freenect.DEPTH_11BIT)[0])
            else:
                self.DepthFrameRaw = freenect.sync_get_depth_with_res(
                    format=freenect.DEPTH_11BIT)[0]
        else:
            self.loadDepthFrame()

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw
        self.DepthFrameHSV[..., 1] = 0x9F
        self.DepthFrameHSV[..., 2] = 0xFF
        self.DepthFrameRGB = cv2.cvtColor(
            self.DepthFrameHSV, cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread(script_path + "/data/rgb_image.png", cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread(
            script_path + "/data/raw_depth.png", 0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (640, 480))
            img = QImage(frame,
                         frame.shape[1],
                         frame.shape[0],
                         QImage.Format_RGB888
                         )
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
        @brief      Converts colormaped depth frame to format suitable for Qt

        @return     QImage
        """
        try:
            img = QImage(self.DepthFrameRGB,
                         self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0],
                         QImage.Format_RGB888
                         )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

                    TODO: Rewrite this function to take in an arbitrary number of coordinates and find the transform without
                    using cv2 functions

        @param      coord1  The coordinate 1  #depth
        @param      coord2  The coordinate 2  #rgb

        @return     Affine transform between coordinates.
        """
        num_coords = 2 * len(coord1)
        A = np.zeros((num_coords, 6))
        b = []
        for point2 in coord2:
            b.append(float(point2[0]))
            b.append(float(point2[1]))
        b = np.asarray(b)
        i = 0
        for point1 in coord1:
            A[i, 0:2] = point1[0:2]
            A[i, 2] = 1
            A[i+1, 3:5] = point1[0:2]
            A[i+1, 5] = 1
            i += 2
        A = np.asarray(A)
        b = np.asarray(b)
        x = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), b.T)
        self.depth2rgb_affine = np.reshape(x, (2, 3))
        csv.writer(open("depth2rgb_affine.cfg", "w+", newline=''), delimiter=',').writerows(self.depth2rgb_affine)
        # else:
        #     x = np.vstack([np.reshape(x,(2,3)),[0,0,1]])
        #     self.cam_ext_mat = x
        # A = [point[i,j+0:j+3].astype(np.float32) for i,point in enumerate(coord1) if i%2 == 0]
        # pts1 = coord1[0:3].astype(np.float32)
        # pts2 = coord2[0:3].astype(np.float32)
        # print(cv2.getAffineTransform(pts1, pts2))
        # return cv2.getAffineTransform(pts1, pts2)

    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

                    TODO: Using an Affine transformation, transform the depth frame to match the RGB frame using
                    cv2.warpAffine()

        @param      frame  The frame

        @return     { description_of_the_return_value }
        """
        frame = cv2.warpAffine(frame, self.depth2rgb_affine,
                               (frame.shape[1], frame.shape[0]))
        return frame

    def loadCameraCalibration(self, file_name=None):
        """!
        @brief      Load camera intrinsic matrix from file.

        @param      file  The file
        """

        mat_str = []
        if file_name == None:
            file_str = "/home/student/armlab-w20/util/calibration.cfg"
        else:
            file_str = file_name
        with open(file_str, 'r') as f:
            for line in f:
                line = line.replace('[', '')
                line = line.replace(']', '')
                line = line.replace('\n', '')
                mat_str.append(line)
        cam_mat_str = mat_str[1:4]
        dist_coeffs = mat_str[-2:]
        dist_coeffs = "".join(dist_coeffs)
        dist_coeffs = dist_coeffs.split()
        dist_coeffs = [float(coeff) for coeff in dist_coeffs]
        self.cam_int_mat = []
        for row in cam_mat_str:
            mat_row = []
            mat_row = row.split()
            mat_row = [float(i) for i in mat_row]
            self.cam_int_mat.append(mat_row)
        self.cam_int_mat = np.asarray(self.cam_int_mat)
        self.dist_coeffs = np.asarray(dist_coeffs)

    #determines color of block from 4 corner points
    def determine_color(self, img ,pts):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
        # find center of rectangle
        center = [(pts[0][0] + pts[2][0]) // 2 , (pts[0][1] + pts[2][1]) // 2]
        sqr_l = 7
        rng = np.zeros((sqr_l * sqr_l, 3), dtype=int)
        i = 0
        x = 0
        while(x < sqr_l):
            y = 0
            while(y < sqr_l):
                xprime = center[0] - 3 + x
                yprime = center[1] - 3 + y
                rng[i,:] = img[yprime][xprime]#img[xprime][yprime]
                i = i + 1
                y = y + 1
            x = x + 1

        # Count up the pixel colors
        r, c = np.shape(rng)
        i = 0
        colors = ["unknown", "pink", "red", "orange", "yellow", "green", "blue", "purple", "black"]
        count = np.zeros((9,1))
        while(i < r):
            hue = rng[i,0] / 179
            sat = rng[i,1] / 255
            val = rng[i,2] / 255
            clr_class = "unknown"
            if sat < 0.56 and val < 0.22:
                clr_class = "black"
                count[8] = count[8] + 1
            elif hue < 0.92:
                if hue <= 0.92 and hue > 0.82:
                    clr_class = "purple"
                    count[7] = count[7] + 1
                elif hue <= 0.69 and hue > 0.59:
                    clr_class = "blue"
                    count[6] = count[6] + 1
                elif hue <= 0.4 and hue > 0.27:
                    clr_class = "green"
                    count[5] = count[5] + 1
                elif hue <= 0.18 and hue > 0.08:
                    clr_class = "yellow"
                    count[4] = count[4] + 1
                elif hue <= 0.08 and hue > 0.0:
                    clr_class = "orange"
                    count[3] = count[3] + 1
            elif hue >= 0.92:
                if (sat <= 0.95 and sat > 0.775) and (val <= 0.6 and val > 0.5):
                    clr_class = "red"
                    count[2] = count[2] + 1
                elif (sat <= 0.775 and sat > 0.7) and (val <= 0.9 and val > 0.72):
                    clr_class = "pink"
                    count[1] = count[1] + 1
            if clr_class == "unknown":
                count[0] = count[0] + 1
            i = i + 1

        max_index = np.argmax(count)
        return colors[max_index]

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        depth_range_dict = {'1':[173,178],'2':[169,172],'3':[165,169],'4':[159,163],'5':[156,158],'6':[147,155],'7':[139,146],'8':[132,138]}
        depth_frame = self.DepthFrameRaw
        rgb_frame = self.VideoFrame
        rgb_frame = cv2.resize(rgb_frame, (640,480))
        depth_frame = cv2.resize(depth_frame, (640, 480))
        np.clip(depth_frame,0,2**10 - 1,depth_frame)
        depth_frame >>= 2
        depth_frame = depth_frame.astype(np.uint8)
        filt_block = []
        for k,v in depth_range_dict.items():
            thresh = cv2.inRange(depth_frame,v[0],v[1])
            cv2.imwrite("/home/student/armlab-w20/log/img.jpeg", thresh)
            _ , contours, _ = cv2.findContours(thresh, 1, 2)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 400 and area < 700:
                    block = []
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    color = self.determine_color(rgb_frame, box)
                    org = (box[0][0], box[0][1])
                    rgb_frame = cv2.putText(rgb_frame, color, org,cv2.FONT_HERSHEY_SIMPLEX , 0.5 ,(0,0,0),2, cv2.LINE_AA)
                    rgb_frame = cv2.drawContours(rgb_frame,[box],0,(0,0,0),0)
                    self.VideoFrame = rgb_frame
                    block.append(box)
                    block.append(int(k))
                    block.append(color)
                    filt_block.append(block)
        return filt_block


    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        # img = cv2.cvtColor(self.VideoFrame, cv2.COLOR_RGB2HSV)
        # img = cv2.resize(img, (640, 480))
        # cv2.imwrite('hsv.jpg', img)
        # mask = None
        # for color in color_ranges:
        #     mask = cv2.inRange(img, color_ranges[color][0], color_ranges[color][1])
        #     cv2.imwrite('blockdetect.jpg', cv2.bitwise_and(img, img, mask=mask))
        blocks = self.detectBlocksInDepthImage()
        pick_up_locs = []
        for block in blocks:
            coords = block[0]
            u = (coords[0][0] + coords[2][0]) // 2
            v = (coords[0][1] + coords[2][1]) // 2
            self.VideoFrame = cv2.circle(self.VideoFrame,(u,v), 1, (0,0,0))
            d = self.DepthFrameRaw[u,v]
            d = self.convertDepthToSI(d)
            world_coords = self.calculatePixelToWorld(u, v, d)
            world_coords[2] = self.max_depth - d
            pick_up_locs.append(world_coords)
        return pick_up_locs

    def calculatePixelToWorld(self, u, v, d):
        cam_coords = d * np.linalg.inv(self.cam_int_mat) @ np.array([[u], [v], [1]])
        return np.linalg.inv(self.cam_ext_mat) @ np.vstack((cam_coords, [[1]]))

    def convertDepthToSI(self, depth_val):
        depth = 0.1236 * math.tan(depth_val/2842.5+1.1863)
        return depth

    def eulerAnglesToRotationMatrix(self, theta):
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])
        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])
        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def getAffineTransformExt(self, coord1, coord2):
        num_coords = 3 * len(coord1)
        A = np.zeros((num_coords, 12))
        b = []
        for point2 in coord2:
            b.append(float(point2[0]))
            b.append(float(point2[1]))
            b.append(float(point2[2]))
        b = np.asarray(b)
        i = 0
        for point1 in coord1:
            A[i, 0:3] = point1[0:3]
            A[i, 3] = 1
            A[i+1, 4:7] = point1[0:3]
            A[i+1, 7] = 1
            A[i+2, 8:11] = point1[0:3]
            A[i+2, 11] = 1
            i += 3
        A = np.asarray(A)
        b = np.asarray(b)
        x = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), b.T)
        x = np.reshape(x, (3, 4))
        return x

    def createExtrinsicCamMat(self, world_coords, pixel_coords):
        print("\nWorld coords")
        print(world_coords)
        print("\npixel coords")
        print(pixel_coords)
        print("\ndist_coeffs")
        print(self.dist_coeffs)
        (_, rotation_vector, translation_vector) = cv2.solvePnP(
            world_coords[:4], pixel_coords[:4], self.cam_int_mat, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        rotation_mat, _ = cv2.Rodrigues(rotation_vector)
        ext_mat = np.hstack([rotation_mat, translation_vector])
        ext_mat = np.vstack((ext_mat, [0, 0, 0, 1]))
        self.cam_ext_mat = ext_mat
        print(self.cam_ext_mat)
    


    
