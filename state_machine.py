"""!
The state machine that implements the logic.
"""

import time
import numpy as np
import csv
import threading
import cv2
from apriltag import apriltag


class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rexarm, planner, kinect):
        """!
        @brief      Constructs a new instance.

        @param      rexarm   The rexarm
        @param      planner  The planner
        @param      kinect   The kinect
        """
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [0.0,           0.0,            0.0,            0.0],
            [np.pi * 0.1,   0.0,            np.pi / 2,      0.0],
            [np.pi * 0.25,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
            [np.pi * 0.4,   np.pi / 2,      -np.pi / 2,     0.0],
            [np.pi * 0.55,  0,              0,              0],
            [np.pi * 0.7,   0.0,            np.pi / 2,      0.0],
            [np.pi * 0.85,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
            [np.pi,         np.pi / 2,      -np.pi / 2,     0.0],
            [0.0,           np.pi / 2,      np.pi / 2,      0.0],
            [np.pi / 2,     -np.pi / 2,     np.pi / 2,      0.0]]
        self.learned_joints = []

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

                    This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rexarm":
            self.initialize_rexarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute_tp":
            self.execute_tp()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "learn":
            self.learn()

        if self.next_state == "remember":
            self.remember()

        if self.next_state == "write":
            self.write()

        if self.next_state == "get_color":
            self.get_color()

        if self.next_state == "find_blocks":
            self.find_blocks()

        # if self.next_state == "dance":
        #     self.execute_dance()

    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the Rexarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()

    def write_joint_pos(self, file_name):
        f = open(file_name+".csv", 'w')
        while self.current_state == 'execute' or self.current_state == 'execute_tp':
            f.write(','.join([str(i) for i in self.rexarm.position_fb]))
            f.write('\n')
            time.sleep(1)
        f.close()

    def execute_tp(self):
        """!
        @brief      Go through all waypoints
        """
        self.status_message = "State: Execute TP- Executing Motion Plan with trajectory planner"
        self.current_state = "execute_tp"
        self.next_state = "idle"
        collect_info = threading.Thread(
            target=self.write_joint_pos, args=("tp_wp",))
        collect_info.start()
        for wp in self.waypoints:
            # Ensure the correct number of joint angles
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(wp)] = wp
            self.tp.set_initial_wp()
            self.tp.set_final_wp(full_wp)

            if self.next_state == "estop":
                break
            # TODO: Set the positions and break if estop is needed
            self.tp.go()
            # self.rexarm.set_positions(wp)
            # time.sleep(1.5)

    def remember(self):
        self.status_message = "Recording position"
        self.current_state = "remember"
        self.waypoints.append(self.rexarm.get_positions())
        self.next_state = "estop"

    def write(self):
        self.status_message = "Writing joint positions to file"
        self.current_state = "write"
        f = open('waypoints.csv', 'w')
        for line in self.waypoints:
            f.write(','.join([str(i) for i in line]))
            f.write("\n")
        f.close()
        self.next_state = "idle"

    def learn(self):
        """!
        @brief      Go through all waypoints
        """

        self.status_message = "State: Ready to Learn"
        if self.current_state != "remember":
            self.waypoints = []
        self.current_state = "learn"
        self.next_state = "estop"
        self.rexarm.disable_torque()
        # self.waypoints = []

    def execute(self):
        """!
        @brief      Go through all waypoints with the trajectory planner.
        """
        self.status_message = "State: Execute - Executing Motion Plan"
        self.current_state = "execute"
        self.next_state = "idle"
        collect_info = threading.Thread(
            target=self.write_joint_pos, args=("notp_wp",))
        collect_info.start()
        for wp in self.waypoints:
            # Ensure the correct number of joint angles
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(wp)] = wp
            # TODO: Send the waypoints to the trajectory planner and break if estop
            if self.next_state == "estop":
                break
            self.rexarm.set_positions(full_wp)
            time.sleep(1.5)

    def calibrate(self):
        """!
        @brief      Gets the calibration clicks
        """
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.kinect.loadCameraCalibration()
        # location_strings = ["lower left corner of board",
        #                     "upper left corner of board",
        #                     "upper right corner of board",
        #                     "lower right corner of board",
        #                     "center of shoulder motor"]
        img = cv2.cvtColor(self.kinect.VideoFrame, cv2.COLOR_RGB2GRAY)
        img = cv2.resize(img, (640, 480))
        detector = apriltag("tagStandard41h12", decimate=1.0)
        result = detector.detect(img)
        if len(result) < 4:
            seen_april = [i['id'] for i in result]
            occd_april = [str(i) for i in range(4) if i not in seen_april]
            self.status_message = "April tags Occluded: {}".format(
                ', '.join(occd_april))
            print(f"April tags occluded: {', '.join(occd_april)}")
            time.sleep(2)
            return
        for i in range(4):
            self.kinect.rgb_click_points[i] = result[i]['lb-rb-rt-lt'][(4-i)%4]
        # print("Result april:\n", self.kinect.rgb_click_points)
        # Uncomment below to redo affine
        # for j in range(5):
        #     self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
        #     while (self.kinect.new_click == False):
        #         time.sleep(.1)
        #     self.kinect.rgb_click_points[j] = self.kinect.last_click.copy()
        #     self.kinect.new_click = False
        # print("Result manual:\n", self.kinect.rgb_click_points)
        # i = 0
        # for j in range(5):
        #     self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
        #     while (self.kinect.new_click == False):
        #         time.sleep(.1)
        #     self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
        #     self.kinect.new_click = False
        # self.kinect.getAffineTransform(
        #     self.kinect.depth_click_points, self.kinect.rgb_click_points)

        """TODO Perform camera calibration here"""
        world_coords = np.array([[-0.297, -0.2876, 0], [-0.297, 0.3183, 0],
                                 [0.307, 0.3183, 0], [0.3067, -0.2876, 0], [0.0, 0.083, 0.225]])
        self.status_message = "Calibration - Creating Extrinsic"
        rgb_click_points_SI = np.float32(self.kinect.rgb_click_points)
        self.kinect.createExtrinsicCamMat(world_coords, rgb_click_points_SI)
        self.status_message = "Calibration - Completed Calibration"
        self.kinect.kinectCalibrated = True
        cv2.imwrite('depthframergb.jpg', self.kinect.DepthFrameRGB)
        cv2.imwrite('depthframeraw.jpg', self.kinect.DepthFrameRaw)
        cv2.imwrite('videoframe.jpg', self.kinect.VideoFrame)
        time.sleep(1)

    def initialize_rexarm(self):
        """!
        @brief      Initializes the rexarm.
        """
        self.current_state = "initialize_rexarm"

        if not self.rexarm.initialize():
            print('Failed to initialize the rexarm')
            self.status_message = "State: Failed to initialize the rexarm!"
            time.sleep(5)
        self.next_state = "idle"


    
    # #returns hsv values of 100 clicked points in csv file
    # def get_color(self):
    #     """Gets a color range for pixels you click on in the rgb image."""
    #     self.current_state = "get_color"
    #     self.next_state = "idle"
    #     self.status_message = "Retrieving color, click fast!"
    #     img = cv2.cvtColor(self.kinect.VideoFrame, cv2.COLOR_RGB2HSV)
    #     img = cv2.resize(img, (640, 480))
    #     rng = np.zeros((100, 3), dtype=int)
    #     i = 0
    #     while i < 100:
    #         if self.kinect.new_click == True:
    #             y, x = self.kinect.last_click
    #             rng[i,:] = img[x][y]
    #             #print("y, x", y, x)
    #             #print()
    #             self.kinect.new_click = False
    #             i = i + 1
    #         time.sleep(.05)
    #     with open("something.csv", 'w') as csvfile:
	#         csvwriter = csv.writer(csvfile)
	#         csvwriter.writerows(rng)

    #gets color of clicked blovk
    # def get_color(self):
    #     self.current_state = "get_color"
    #     self.next_state = "idle"
    #     self.status_message = "Retrieving color, click the middle of the block"
    #     img = cv2.cvtColor(self.kinect.VideoFrame, cv2.COLOR_RGB2HSV)
    #     img = cv2.resize(img, (640, 480))

    #     # find center of rectangle
    #     #center = [((pts[0,0] + pts[2,0]) / 2) , ((pts[0,1] + pts[2,1]) / 2)]
    #     yt, xt = self.kinect.last_click
    #     center = [xt,yt]
        
    #     # create square around the center with l = 1/3*min_rect_l and input hsv vals in rng
    #     # delx1 = np.abs(pts[0,0] - pts[1,0])
    #     # dely1 = np.abs(pts[0,1] - pts[1,1])
    #     # delx2 = np.abs(pts[1,0] - pts[2,0])
    #     # dely2 = np.abs(pts[1,1] - pts[2,1])
    #     # l = [0,0]
    #     # l[0] = np.sqrt(delx1 * delx1 + dely1 * dely1)
    #     # l[1] = np.sqrt(delx2 * delx2 + dely2 * dely2)
    #     # min_l = np.min(l)
    #     # sqr_l = np.floor(1/3 * min_l)
    #     # if sqr_l % 2 == 0:
    #     #     sqr_l = sqr_l + 1
    #     sqr_l = 5 
    #     rng = np.zeros((sqr_l * sqr_l, 3), dtype=int)
    #     i = 0
    #     x = 0
    #     while(x < sqr_l):
    #         y = 0
    #         while(y < sqr_l):
    #             xprime = center[0] - 2 + x
    #             yprime = center[1] - 2 + y
    #             rng[i,:] = img[xprime][yprime]
    #             i = i + 1
    #             y = y + 1
    #         x = x + 1

    #     # Count up the pixel colors
    #     r, c = np.shape(rng)
    #     i = 0
    #     colors = ["unknown", "pink", "red", "orange", "yellow", "green", "blue", "purple", "black"]
    #     count = np.zeros((9,1))
    #     while(i < r):
    #         hue = rng[i,0] / 179
    #         sat = rng[i,1] / 255
    #         val = rng[i,2] / 255
    #         clr_class = "unknown"
    #         if sat < 0.56 and val < 0.22:
    #             clr_class = "black"
    #             count[8] = count[8] + 1
    #         elif hue < 0.92:
    #             if hue <= 0.92 and hue > 0.82:
    #                 clr_class = "purple"
    #                 count[7] = count[7] + 1
    #             elif hue <= 0.69 and hue > 0.59:
    #                 clr_class = "blue"
    #                 count[6] = count[6] + 1
    #             elif hue <= 0.4 and hue > 0.27:
    #                 clr_class = "green"
    #                 count[5] = count[5] + 1
    #             elif hue <= 0.18 and hue > 0.08:
    #                 clr_class = "yellow"
    #                 count[4] = count[4] + 1
    #             elif hue <= 0.08 and hue > 0.0:
    #                 clr_class = "orange"
    #                 count[3] = count[3] + 1
    #         elif hue >= 0.92:
    #             if sat <= 0.95 and sat > 0.775 and val <= 0.6 and val > 0.5:
    #                 clr_class = "red"
    #                 count[2] = count[2] + 1
    #             elif sat <= 0.775 and sat > 0.7 and val <= 0.9 and val > 0.72:
    #                 clr_class = "pink"
    #                 count[1] = count[1] + 1
    #         if clr_class == "unknown":
    #             count[0] = count[0] + 1
    #         i = i + 1

    #     max_index = np.argmax(count)
    #     #return colors[max_index]
    #     self.status_message = colors[max_index]
    #     time.sleep(3)
    
    def find_blocks(self):
        self.current_state = "find_blocks"
        self.next_state = "idle"
        self.status_message = "Looking for blocks!"
        self.kinect.blockDetector()
        time.sleep(1)
