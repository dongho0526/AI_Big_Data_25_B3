import cv2
import numpy as np
import matplotlib.image as mpimg

def hist(img):
    bottom_half = img[img.shape[0]//2:,:]
    return np.sum(bottom_half, axis=0)

class LaneLines:
    """ Class containing information about detected lane lines.

    Attributes:
        left_fit (np.array): Coefficients of a polynomial that fit left lane line
        right_fit (np.array): Coefficients of a polynomial that fit right lane line
        parameters (dict): Dictionary containing all parameters needed for the pipeline
        debug (boolean): Flag for debug/normal mode
    """
    def __init__(self):
        """Init Lanelines.

        Parameters:
            left_fit (np.array): Coefficients of polynomial that fit left lane
            right_fit (np.array): Coefficients of polynomial that fit right lane
            binary (np.array): binary image
        """
        self.left_fit = None
        self.right_fit = None
        self.binary = None
        self.nonzero = None
        self.nonzerox = None
        self.nonzeroy = None
        self.clear_visibility = True
        self.dir = []
        # self.left_curve_img = mpimg.imread('/home/piai/바탕화면/lanelane/LaneDetectionfromDongho/Advanced-Lane-Lines/images/left_turn.png')
        # self.right_curve_img = mpimg.imread('/home/piai/바탕화면/lanelane/LaneDetectionfromDongho/Advanced-Lane-Lines/images/right_turn.png')
        # self.keep_straight_img = mpimg.imread('/home/piai/바탕화면/lanelane/LaneDetectionfromDongho/Advanced-Lane-Lines/images/straight.png')
        # self.left_curve_img = cv2.normalize(self.left_curve_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # self.right_curve_img = cv2.normalize(self.right_curve_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # self.keep_straight_img = cv2.normalize(self.keep_straight_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # HYPERPARAMETERS
        # Number of sliding windows
        self.nwindows = 9
        self.margin = 25  # Adjusted margin for smaller image width
        self.minpix = 15  # Adjusted min pixels found to recenter window


    def forward(self, img):
        """Take a image and detect lane lines.

        Parameters:
            img (np.array): An binary image containing relevant pixels

        Returns:
            Image (np.array): An RGB image containing lane lines pixels and other details
        """
        self.extract_features(img)

        return self.fit_poly(img)

    def pixels_in_window(self, center, margin, height):
        """ Return all pixel that in a specific window

        Parameters:
            center (tuple): coordinate of the center of the window
            margin (int): half width of the window
            height (int): height of the window

        Returns:
            pixelx (np.array): x coordinates of pixels that lie inside the window
            pixely (np.array): y coordinates of pixels that lie inside the window
        """
        topleft = (center[0] - margin, center[1] - height // 2)
        bottomright = (center[0] + margin, center[1] + height // 2)
        condx = (topleft[0] <= self.nonzerox) & (self.nonzerox <= bottomright[0])
        condy = (topleft[1] <= self.nonzeroy) & (self.nonzeroy <= bottomright[1])
        return self.nonzerox[condx & condy], self.nonzeroy[condx & condy]

    def extract_features(self, img):
        """ Extract features from a binary image. """
        self.binary = img
        self.window_height = int(img.shape[0] // self.nwindows)
        self.nonzero = img.nonzero()
        self.nonzerox = np.array(self.nonzero[1])
        self.nonzeroy = np.array(self.nonzero[0])

    def find_lane_pixels(self, img):
        """Find lane pixels from a binary warped image.

        Parameters:
            img (np.array): A binary warped image

        Returns:
            leftx (np.array): x coordinates of left lane pixels
            lefty (np.array): y coordinates of left lane pixels
            rightx (np.array): x coordinates of right lane pixels
            righty (np.array): y coordinates of right lane pixels
            out_img (np.array): A RGB image that use to display result later on.
        """
        assert(len(img.shape) == 2)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img, img, img))

        histogram = hist(img)
        midpoint = histogram.shape[0]//2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Current position to be update later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base
        y_current = img.shape[0] + self.window_height//2

        # Create empty lists to reveice left and right lane pixel
        leftx, lefty, rightx, righty = [], [], [], []

        # Step through the windows one by one
        for _ in range(self.nwindows):
            y_current -= self.window_height
            center_left = (leftx_current, y_current)
            center_right = (rightx_current, y_current)

            good_left_x, good_left_y = self.pixels_in_window(center_left, self.margin, self.window_height)
            good_right_x, good_right_y = self.pixels_in_window(center_right, self.margin, self.window_height)

            # Append these indices to the lists
            leftx.extend(good_left_x)
            lefty.extend(good_left_y)
            rightx.extend(good_right_x)
            righty.extend(good_right_y)

            if len(good_left_x) > self.minpix:
                leftx_current = int(np.mean(good_left_x))
            if len(good_right_x) > self.minpix:
                rightx_current = int(np.mean(good_right_x))

        return leftx, lefty, rightx, righty, out_img

    def fit_poly(self, img):
        """Find the lane line from an image and draw it.

        Parameters:
            img (np.array): a binary warped image

        Returns:
            out_img (np.array): a RGB image that have lane line drawn on that.
        """

        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(img)
        
        # print("leftx=",leftx, "lefty=",lefty, "rightx=",rightx, "righty=",righty, "out_img=",out_img)
        if len(lefty) > 20:
            self.left_fit = np.polyfit(lefty, leftx, 2)
        else:
            self.left_fit = None
            return out_img, None, None, None
        if len(righty) > 20:
            self.right_fit = np.polyfit(righty, rightx, 2)
        else:
            self.right_fit = None
            return out_img, None, None, None

        # Generate x and y values for plotting
        maxy = img.shape[0] - 1
        miny = img.shape[0] // 3
        if len(lefty):
            maxy = max(maxy, np.max(lefty))
            miny = min(miny, np.min(lefty))

        if len(righty):
            maxy = max(maxy, np.max(righty))
            miny = min(miny, np.min(righty))

        ploty = np.linspace(miny, maxy, img.shape[0])

        left_fitx = self.left_fit[0]*ploty**2 + self.left_fit[1]*ploty + self.left_fit[2]
        right_fitx = self.right_fit[0]*ploty**2 + self.right_fit[1]*ploty + self.right_fit[2]

        # Visualization
        for i, y in enumerate(ploty):
            l = int(left_fitx[i])
            r = int(right_fitx[i])
            cx = int(l+((r-l)/2))
            
            # cx= (rightx - leftx) / 2
            # cy = (righty - lefty) / 2

            # y[0] 
            
            y = int(y)
            # print(cx, y)
            # cv2.line(out_img, (cxl, y), (r, y), (255, 0, 0))
            # cv2.circle(out_img, (l, y), 3, (0, 0, 255), thickness=30)
            # cv2.circle(out_img, (r, y), 8, (0, 255, 255), thickness=30)
            cv2.line(out_img, (l, y), (r, y), (0, 255, 0))
            cv2.circle(out_img, (cx, y), 8, (0, 0, 255), thickness=1)
            cv2.circle(out_img, (l, y), 8, (255, 0, 0), thickness=1)
            cv2.circle(out_img, (r, y), 8, (255, 0, 0), thickness=1)
            cv2.circle(out_img, (cx, 410), 8, (255, 255, 255), thickness=5)
            cv2.circle(out_img, (1, 410), 8, (255, 255, 255), thickness=5)
            
            
            # cv2.circle(out_img, (272, 410), 8, (255, 255, 0), thickness=)
            
            # if y == 410:
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            #     print(cx, y)
            target_point_x = cx-39
            diff = target_point_x - 320
            # car is left -> positive
            
            target_point_y = 410
            # cv2.line(out_img, (l, 200), (r, 200), (255, 255, 255))
            # cv2.line(out_img, (l/2, y), (r/2, y), (0, 255, 0))
            # print("y = ", y)
            print(diff)
            print()
            
        print("==============================================")
        # lR, rR, pos = self.measure_curvature()
        # print("pos=",pos)
        # pos = cx - 281
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        # print("pos=", pos)
        return out_img, target_point_x, target_point_y, diff

    # def plot(self, out_img):
    #     # print("plot")
    #     np.set_printoptions(precision=6, suppress=True)
    #     # print("plot")
    #     lR, rR, pos = self.measure_curvature()
    #     # print("lR", lR)
    #     # print("rR", rR)
    #     # print("pos", pos)

    #     if lR is None or rR is None or pos is None:
    #         return out_img
    #     value = None
    #     if abs(self.left_fit[0]) > abs(self.right_fit[0]):
    #         value = self.left_fit[0]
    #     else:
    #         value = self.right_fit[0]

    #     if abs(value) <= 0.00015:
    #         self.dir.append('F')
    #     elif value < 0:
    #         self.dir.append('L')
    #     else:
    #         self.dir.append('R')
        
    #     if len(self.dir) > 10:
    #         self.dir.pop(0)

    #     W = 400
    #     H = 500
    #     widget = np.copy(out_img[:H, :W])
    #     widget //= 2
    #     widget[0,:] = [0, 0, 255]
    #     widget[-1,:] = [0, 0, 255]
    #     widget[:,0] = [0, 0, 255]
    #     widget[:,-1] = [0, 0, 255]
    #     out_img[:H, :W] = widget

    #     direction = max(set(self.dir), key = self.dir.count)
    #     msg = "Keep Straight Ahead"
    #     curvature_msg = "Curvature = {:.0f} m".format(min(lR, rR))
    #     if direction == 'L':
    #         y, x = self.left_curve_img[:,:,3].nonzero()
    #         out_img[y, x-100+W//2] = self.left_curve_img[y, x, :3]
    #         msg = "Left Curve Ahead"
    #     if direction == 'R':
    #         y, x = self.right_curve_img[:,:,3].nonzero()
    #         out_img[y, x-100+W//2] = self.right_curve_img[y, x, :3]
    #         msg = "Right Curve Ahead"
    #     if direction == 'F':
    #         y, x = self.keep_straight_img[:,:,3].nonzero()
    #         out_img[y, x-100+W//2] = self.keep_straight_img[y, x, :3]

    #     cv2.putText(out_img, msg, org=(10, 240), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
    #     if direction in 'LR':
    #         cv2.putText(out_img, curvature_msg, org=(10, 280), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)

    #     cv2.putText(
    #         out_img,
    #         "Good Lane Keeping",
    #         org=(10, 400),
    #         fontFace=cv2.FONT_HERSHEY_SIMPLEX,
    #         fontScale=1.2,
    #         color=(0, 255, 0),
    #         thickness=2)

    #     cv2.putText(
    #         out_img,
    #         "Vehicle is {:.2f} m away from center".format(pos),
    #         org=(10, 450),
    #         fontFace=cv2.FONT_HERSHEY_SIMPLEX,
    #         fontScale=0.66,
    #         color=(255, 255, 255),
    #         thickness=2)

    #     return out_img

    def measure_curvature(self):
        """Measure curvature and calculate vehicle position from center.
        """
        if self.left_fit is None or self.right_fit is None:
            return None, None, None

        ym_per_pix = 30/480  # meters per pixel in y dimension
        xm_per_pix = 3.7/640  # meters per pixel in x dimension

        left_fit_cr = np.polyfit(self.nonzeroy * ym_per_pix, self.nonzerox * xm_per_pix, 2)
        right_fit_cr = np.polyfit(self.nonzeroy * ym_per_pix, self.nonzerox * xm_per_pix, 2)
        
        y_eval = np.max(self.nonzeroy)

        # Calculate radii of curvature in meters
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

        # Calculate the position of the vehicle with respect to center
        car_position = (self.nonzerox[np.argmax(self.nonzeroy)] - 320) * xm_per_pix

        return left_curverad, right_curverad, car_position