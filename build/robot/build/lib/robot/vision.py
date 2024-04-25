import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist # Twist is for linear+angular velocities
from cv_bridge import CvBridge # onvert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from collections import deque
 
class Vision(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('Vision')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera/color/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    
    # Create a publisher to publish the desired 
    # linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /lab05/cmd_vel topic. 
    # Using the diff_drive plugin enables the robot model to read this
    # topic and execute the motion.
    self.cmdvel_publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
    # Define and initialize motion control properties
    self.v = 0.0  # linear velocity
    self.w = 0.0  # angular velocity
    
    #Calculate center of image frame
    self.height, self.width = 480, 1920 #Assuming image size of 640x480
    self.centerX = int(self.width/2)
    
    
    # declare the parameters
    self.declare_parameters(
        namespace='',
        parameters=[
            ('maxV', rclpy.Parameter.Type.DOUBLE),
            ('deadBand', rclpy.Parameter.Type.INTEGER),
            ('maxA', rclpy.Parameter.Type.DOUBLE),
            ('grayUThresh', rclpy.Parameter.Type.INTEGER),
            ('grayLThresh', rclpy.Parameter.Type.INTEGER),
            ('queueSize', rclpy.Parameter.Type.INTEGER)
        ]
    )
    # Read current parameter values into local class variables
    # (one get_parameter call for each parameter)
    self.v = 8.0
    self.deadBand = 80
    self.maxA = 0.07
    self.grayUThresh = 45
    self.grayLThresh = 0
    self.maxLength = 6
    
    
        #Initialize a queue for centroid moving average
    self.qCX = deque(maxlen = self.maxLength)
    self.qCY = deque(maxlen = self.maxLength)
    
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    print("received frame")
    # Display the message on the console
    self.get_logger().info('Processing camera frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    height, width = current_frame.shape[:2]

    
    # Do image processing here
    # *** TO DO *****
    
    #convert image to grayscale
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3,3),0)
    #hist = cv2.calcHist([gray],[0],None, [256],[0,256])
    #cv2.imshow("hist",hist)
    #apply image segmentation to find road segments
    masked_img = cv2.inRange(gray, self.grayLThresh, self.grayUThresh)    
    
    #apply canny edge detection
    #edges = cv2.Canny(masked_img,50, 150)
    masked_img = cv2.dilate(masked_img, None, iterations=1)
    masked_img = cv2.erode(masked_img, None, iterations=1)
    
    #find contours from the edges
    contours, hierarchy = cv2.findContours(masked_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(current_frame, contours, -1, (0,255,0),2)
    #filter small contours, calc centroids
    count = 0;sumX = 0;sumY = 0
    for c in contours:
       if cv2.contourArea(c) > 10000:
          count += 1
          M = cv2.moments(c)
          cX = int(M["m10"]/M["m00"])
          cY = int(M["m01"]/M["m00"])
          sumX += cX
          sumY += cY
          


    if count > 0:
       grandCX = int(float(sumX)/float(count))
       grandCY = int(float(sumY)/float(count))
    else:# no contours; therefore no track stop
       grandCX = self.centerX
       grandCY = int(self.height/2)
       self.w = 0.0
       self.v = 0.0
    
    #update moving average queue with the centroid
    self.qCX.appendleft(grandCX)
    self.qCY.appendleft(grandCY)
       
       # Calculate moving average of centroids
    maCX = int(np.average(self.qCX))
    maCY = int(np.average(self.qCY))
    cv2.circle(current_frame, (maCX, maCY), 20, (255,0,0),-1)
    cv2.putText(current_frame,str(maCX - self.centerX),(maCX+20, maCY),cv2.FONT_HERSHEY_SIMPLEX,2,(209,80,0,255),3)
       
       # Calculate desired robot motion based on centroid position
    if (maCX - self.centerX) > self.deadBand:
       self.w = float(-self.maxA)
    elif (maCX - self.centerX) < -self.deadBand:
       self.w = float(self.maxA)
    else:
       self.w = 0.0
       
       # publish cmd_vel message to change the robot's motion
    twist_msg = Twist()
    twist_msg.linear.x = self.v
    twist_msg.angular.z = self.w
    self.cmdvel_publisher_.publish(twist_msg)
       
    cv2.imshow("camera", current_frame)
    #cv2.imshow("masked", masked_img)
       #cv2.imshow("cont", contours)
       #cv2.imshow("edges", edges)
    cv2.waitKey(1)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  vision = Vision()
  
  # Spin the node so the callback function is called.
  rclpy.spin(vision)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  vision.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

