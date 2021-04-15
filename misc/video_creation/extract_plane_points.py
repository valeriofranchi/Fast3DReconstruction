import cv2 as cv
import argparse
import numpy as np
import imutils
from imutils import perspective
#from wheelDetection.fpsTracker import FPS
from imutils.video import WebcamVideoStream
from scipy.spatial import distance as dist
import math

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-r", "--recording", help="input 0 or higher number if recording", default=-1)

args = vars(ap.parse_args())

class extract_planes:
    ## CREATE OUR OBJECTS FROM OTHER CLASSES
    #camera object
    camera = None
    # output object for recording
    local_path_to_video = None
    def __init__(self,path_to_video=None):
        self.local_path_to_video = path_to_video

    ## change source to args["video"] when using video feed
    # load the video
    if local_path_to_video is None:
        try:
            camera = WebcamVideoStream(src=2).start()
            # if you can't find a USB webcam on linux type 'ls /dev/video*' with and without the camera plugged in to find the port number..
        except:
            print("Failure")
    else:
        camera = WebcamVideoStream(src=local_path_to_video).start()
        ## AttributeError: 'NoneType' object has no attribute 'shape'  , this occures when the path to the video is incorrect

    (h,w,c)=np.array(camera.read()).shape
    black_image = np.ones((h, w), dtype='uint8')

    ''' temp image to fork behavior '''
    plane_preview = None
    ''' array that might be selected'''
    temp_coordinate_array = []

    ''' SUPER IMPORTANT HYPERPAREMTER'''
    number_planes_used = 2

    plan_selection_complete = False
    break_condition = False

    array_of_points = []
    array_of_planes = []
    cv.namedWindow("Select 4 point outline of a plane")


    ''' Method to select points to enclose space on a plane. Activates on left click '''
    def select_points(self,event,  x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.array_of_points.append((x, y))

        if event == cv.EVENT_MBUTTONUP:
            self.array_of_planes.append(self.temp_coordinate_array)
            ''' reset values'''
            self.temp_coordinate_array = []
            self.array_of_points = []
            self.plane_preview = None
            self.black_image = np.ones((self.h, self.w), dtype='uint8')
            print("plane saved")

    ''' Method to check if there are 4 points selected. Displays bounding box '''
    # Todo, maybe more than 4 points is a good idea. Also do we need to order these points
    def check_number_points(self,originalFrame):

        if self.plane_preview is not None:
            return self.plane_preview
        elif len(self.array_of_points) == 4:
            contour_pts = [np.array(self.array_of_points, dtype=np.int32)]
            output = cv.fillPoly(self.black_image, contour_pts, color=255)

            for idx_row, row in enumerate(output):
                for idx_column, column in enumerate(row):
                    if column == 255:
                        self.temp_coordinate_array.append([idx_row,idx_column])

            output = originalFrame
            for pixel in self.temp_coordinate_array:
                output[pixel[0],pixel[1]] = (0,255,0)

            self.plane_preview = output
            return output

        else:
            return originalFrame

    ''' If the user clicks on q the main loop ends and 2/3 arrays representing image patches are returned '''
    def confirm_plane_selection(self, originalFrame):
        print("left click to confirm selection of planes")
        print(" ")
        print(" click middle mouse button to restart")

        for shape in self.array_of_planes:
            for pixel in shape:
                originalFrame[pixel[0],pixel[1]] = (0,255,0)


        while True:
            cv.imshow("CONFIRM PLANE SELECTION", originalFrame)
            key = cv.waitKey(1) & 0xFF
            if key == ord("q") or key == ord("Q"):
                break

            cv.setMouseCallback("CONFIRM PLANE SELECTION", self.mouse_selection_to_confirm)
            if self.break_condition == True:
                break
        # reset break condition
        self.break_condition = False
        cv.destroyWindow("CONFIRM PLANE SELECTION")

    ''' Handles mouse event in confirm_plane_selection '''
    def mouse_selection_to_confirm(self,event,  x, y, flags, param):

        if event == cv.EVENT_LBUTTONDOWN:
            self.plan_selection_complete = True
            self.break_condition = True
        if event == cv.EVENT_MBUTTONUP:
            # restart selection
            self.array_of_planes = []
            self.break_condition = True
            print("restart selection")




    ''' Method to run the video of a static scan in a loop. In the loop the point selection'''
    def run_video(self):
        #todo: make video run in a loop
        while self.plan_selection_complete is False:
            # grab the current frame
            originalFrame = self.camera.read()
            # output_img = originalFrame.copy()
            # check the frame isn't empty. If so code should stop running
            if originalFrame is None:
                print("[INFO] The code has finished running")
                return

            if len(self.array_of_planes) == self.number_planes_used:
                self.confirm_plane_selection(originalFrame)

            # draw the points in current selection
            for point in self.array_of_points:
                cv.circle(originalFrame, point, 3, (0, 0, 255), -1)

            # check if we have 4 points
            originalFrame=self.check_number_points(originalFrame)


            cv.imshow("Select 4 point outline of a plane", originalFrame)
            key = cv.waitKey(1) & 0xFF
            cv.setMouseCallback("Select 4 point outline of a plane", self.select_points, param=originalFrame)

            # if the 'q' key is pressed, stop the loop
            if key == ord("q") or key == ord("Q"):
                break
        self.camera.stop()
        cv.destroyAllWindows()

        print("success")

        # returns a list of points
        return self.array_of_planes


# extract_plane_pts = extract_planes()
# result_array = extract_plane_pts.run_video()

# print(len(result_array))
# print(len(result_array[1]))


