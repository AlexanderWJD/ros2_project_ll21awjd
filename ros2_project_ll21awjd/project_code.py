import threading
import sys, time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import Image

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from math import sin, cos

from rclpy.exceptions import ROSInterruptException
import signal

from rclpy.executors import MultiThreadedExecutor


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.br = CvBridge()
        self.subscription  = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.sensitivity = 30
        
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')                 # initialise the Node>GoToPose object by giving it a method called .action_client which is type ActionClient() from rclpy 

        #self.subscription
        
    def image_callback(self, data):
        print("woo")
    
        img = self.br.imgmsg_to_cv2(data, "bgr8")             #ad Convert ros img format to cv2 compatible

        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                #ad Convert from BGR to HSV so the Hue is a single parameter
        hsv_green_lower = np.array([60 - self.sensitivity, 80, 80])   #ad The colour green isn't a single value but its centered around 60deg on the colour wheel
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])   #ad we've defined a range of colours at 60 +-30 to be green
                                                                        #ad Opencv colour wheel is wonky with range 0-180
        hsv_blue_lower = np.array([120 - self.sensitivity, 80, 80])   
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        hsv_red_lower_0 = np.array([0, 80, 80])
        hsv_red_upper_0 = np.array([0 + self.sensitivity, 255, 255])  
        hsv_red_lower_180 = np.array([180 - self.sensitivity, 80, 80])
        hsv_red_upper_180 = np.array([180, 255, 255])
        
        red_mask_0 = cv2.inRange(hsv_image, hsv_red_lower_0, hsv_red_upper_0) #ad Pixel mask for filterout non-red pixels
        red_mask_180 = cv2.inRange(hsv_image, hsv_red_lower_180, hsv_red_upper_180) #ad Pixel mask for filterout non-red pixels
        
        red_mask = cv2.bitwise_or(red_mask_0, red_mask_180) #ad Use bitwise OR to combine the 2 red masks
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper) #ad Pixel mask for filterout non-green pixels
        blue_mask  = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper) #ad Pixel mask for filterout non-blue pixels

        
        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rb_mask = cv2.bitwise_or(red_mask, blue_mask)
        bg_mask = cv2.bitwise_or(blue_mask, green_mask)
        rgb_mask = cv2.bitwise_or(rg_mask,blue_mask)        #ad Combine all three masks 

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        #bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.
        green_filtered_img = cv2.bitwise_and(img, img, mask=green_mask)
        blue_filtered_img = cv2.bitwise_and(img, img, mask=blue_mask)
        red_filtered_img = cv2.bitwise_and(img, img, mask=red_mask)
        rgb_filtered_img = cv2.bitwise_and(img, img, mask=rgb_mask)

        #ad Determine image area... using cv2 'contours' method then counting the number pixels within those contours
        #ad Find contours:
        contours, _ = cv2.findContours(rgb_mask,mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        #ad Find biggest contour area
        c = max(contours, key=cv2.contourArea, default=None)
        #print("max contours :", c)
        print(contours)
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('camera_Feed_with_mask',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed_with_mask', rgb_filtered_img)
        cv2.resizeWindow('camera_Feed_with_mask',320,240)
        
        cv2.namedWindow('contours',cv2.WINDOW_NORMAL)
        cv2.drawContours(rgb_filtered_img, contours, -1, (0,255,0), 3)
        cv2.resizeWindow('contours',320,240)
        cv2.waitKey(3)
        
        filtered_img = cv2.bitwise_and(img, img, mask=rgb_mask)
        cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_feed', filtered_img)
        cv2.resizeWindow('camera_feed', 320, 240)
        cv2.waitKey(3)
        
        
        
        return
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.
        

        #--------------------------------------------------Navigation callbacks for the nav action client for this node----------------------------
    
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation (instead of Roll(x) Pitch(y) Yaw(z) (Euler angles) ROS nav Stack requires Quarternions - (w,xi,yj,zk) where w is scalar, rest is vector) )
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)                                             # yaw is given as an argument MyGoToPoseObject(x,y,yaw) in the form of a rotation in (presumably) radians around the z axis then converted 
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.

        # Access the current pose
        #current_pose = feedback_msg.feedback.current_pose
        #position = current_pose.pose.position
        #orientation = current_pose.pose.orientation

        ## Access other feedback fields
        #navigation_time = feedback_msg.feedback.navigation_time
        #distance_remaining = feedback_msg.feedback.distance_remaining

        # Print or process the feedback data
        # self.get_logger().info(f'Current Pose: [x: {position.x}, y: {position.y}, z: {position.z}]')
        #self.get_logger().info(f'Distance Remaining: {distance_remaining}')
        
    def move_around(self):
        poses = [[-1.49, -5.0, 0.0], [-5.57, -2.55, 4.71], [-6.72, -8.17, 3.14], [3.61, -8.63, -0.1]]
        for pose in poses:
            self.send_goal(pose[0], pose[1], pose[2])

# -----------------------------------------------------------------------------------



# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    
    def signal_handler(sig, frame):
        rclpy.shutdown()
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    controller = Controller()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    thread.start()
    
    controller.move_around()


    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    
    
    #-------------------------------------------------------------

        



# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
