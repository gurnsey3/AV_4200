############## IMPORT LIBRARIES #################
# Python math library

#from tf2_ros import transformations
#import tf2_geometry_msgs


# Scientific computing library
import numpy as np 
 
# ROS client library for Python
import rclpy 

from rclpy.duration import Duration
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist msg type is used to represent linear and angular velocities
from geometry_msgs.msg import Twist     

# Pose messages
from geometry_msgs.msg import Pose 

# Vector3 messages
from geometry_msgs.msg import Vector3 

# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry
                     
# LaserScan messages to sense distance to obstacles (i.e. walls)      
from sensor_msgs.msg import LaserScan    
  
# float64 array msg (for LaserScan range data)
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 
 
import RPi.GPIO as GPIO
from time import sleep
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import ChassisSpeeds
import wpimath.units 
import math




class Controller(Node):
  """
  This is the Controller class we will use. 
  The class is a subclass of the Node class for ROS2.
  This class will control the robot motion
  """

  def __init__(self):
    """
    This is the class's constructor, used to initialize the node
    """
    # Initiate the parent Node class's constructor and give it a name
    super().__init__('Controller')
 
    ########### SETUP NODE'S SUBSCRIBERS & PUBLISHERS ############

    # Create a subscriber to messages of type
    # nav_msgs/Odometry (position and orientation of the robot)
    # The maximum number of queued messages is 10.
    # We will define the class method (function) odom_callback
    # to receive these messages.  
    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/robot/odom',
                           self.odom_callback,
                           10)
 
    # Create a subscriber to messages of type 
    # geometry_msgs/Twist.msg, to listen for velocity commands
    # The maximum number of queued messages is 10.
    # We will define the class method (function) velocity_callback
    # to receive these messages.
    self.velocity_subscriber = self.create_subscription(
                               Twist,
                               '/robot/cmd_vel',
                               self.velocity_callback,
                               10)
 
    
    # Create a publisher to publish the desired 
    # linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /lab02/cmd_vel topic. 
    # Using the diff_drive plugin enables the robot model to read this
    # topic and execute the motion.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/robot/cmd_vel', 
                      10)

    self.publisher_ = self.create_publisher(Odometry, 'odometry', 10)
    #create timer driven odometry function
    self.timer = self.create_timer(0.2, self.odometry_timer_callback)


    
    ################### ROBOT CONTROL PARAMETERS ##################

    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.05
 
    # Current position and orientation of the robot and velocities in the global 
    # reference frame. We'll initialize to zero, but values will
    # subsequently be set from odometry readings.
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0
    self.current_v = 0.0
    self.current_yaw_rate = 0.0
    self.control_v =0.0
    self.control_yaw_rate = 0.0


    self.x=0.0
    self.y=0.0
    self.theta=0.0
    self.v=0.0
    self.w=0.0
    self.lastTime=self.get_clock().now()


    

    # Set turning speeds (to the left) in rad/s 
    # Determined by trial and error.
    self.turning_speed_wf_fast = 3.0  # Fast turn
    self.turning_speed_wf_slow = 0.05 # Slow turn
 
    # Set min obstacle distance (before change state) - in meters
    self.min_obstacle_dist = 0.2

    # Set position (meters) and orientation (radians) thresholds for 
    # determining when have reached objective
    self.min_position_error=0.01
    self.min_orientation_error=0.01



    self.linearmem = 0
    self.angularmem = 0

    #kinematics = DifferentialDriveKinematics(Units.inchesToMeters(10.5))

    self.rpin = 33				# PWM pin connected to right motor
    self.lpin = 32                               # PWM pin connected to left motor
    GPIO.setwarnings(False)			#disable warnings
    GPIO.setmode(GPIO.BOARD)		#set pin numbering system
    GPIO.setup(self.lpin,GPIO.OUT)
    GPIO.setup(self.rpin,GPIO.OUT)
    self.r_pwm = GPIO.PWM(self.rpin,50)		#create PWM instance with frequency
    self.l_pwm = GPIO.PWM(self.lpin,50)		#create PWM instance with frequency
    #pi_pwm.start(0)
    #sleep(2)	
    #pi_pwm.ChangeDutyCycle(25) #provide duty cycle in the range 0-100			
    #sleep(50)
    #pi_pwm.ChangeDutyCycle(0) #provide duty cycle in the range 0-100

    self.r_pwm.start(7)		#start PWM of required Duty Cycle 
    self.l_pwm.start(7)		#start PWM of required Duty Cycle



    # Control state
    # Finite states for control modes
    #  "idle": Robot is awaiting an objective
    #  "start_turn": robot is doing starting orientation towards objective
    #  "travel": Robot is moving in straight line to objective
    #  "end_turn": robot is doing ending orientation at objective
    #  "blocked": Robot is blocked by an obstacle and has stopped
    self.control_state = "idle"

  ###########  SUBSCRIPTION CALLBACK & HELPER FUNCTIONS ########

  def odom_callback(self, msg):
    """
    This callback function receives odometry msg information (/labo02/odom)
    containing the position and orientation
    of the robot in the global reference frame. 
    The position is x, y, z.
    The orientation is a x,y,z,w quaternion, which are converted 
    to euler angles by calling the euler_from_quaternion function 
    defined below. 
    """                    
    roll, pitch, yaw = self.euler_from_quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w)
 
    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y
    self.current_yaw = yaw
    
    # Call robot_control function to start processing 
    # (change in pose may result in change in state)
    #self.robot_control()
  

  def quaternion_from_euler(self, ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
  def euler_from_quaternion(self, x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians
 
  def velocity_callback(self, msg):
    """
    This callback function listens to the velocity commands (/lab02/cmd_vel). 
    In particular, we only need the x velocity and angular velocity 
    around the z axis (yaw rate)both in the robot's reference frame
    [v,yaw_rate]
    [meters/second, radians/second]
    """
    # Current Forward velocity in the robot's reference frame
    self.current_v = msg.linear.x
 
    # Current Angular velocity around the robot's z axis
    self.current_yaw_rate = msg.angular.z
    
    #convert to left and right wheel speeds
    
    #chassisSpeeds = ChassisSpeeds(linear,0,angular)
    #wheelSpeeds = self.kinematics.toWheelSpeeds(chassisSpeeds)

    #calculate what left and right wheel speeds should be
    left = (self.current_v+self.current_yaw_rate*5.25)/(4*3.14)#wheelSpeeds.left*4*3.14
    right = (self.current_v-self.current_yaw_rate*5.25)/(4*3.14)#wheelSpeeds.right*4*3.14
    #print(left)
    #print(right)
    #Left Motor
    if left >= 5.25: #Saturated forward
        lpwm = 10.4
    elif left > 1.5: # Forward PWM range
        lpwm = -((-19.998+math.sqrt(-4.0968*left+22.4080776))/(2.0484))
    elif left <= -5.25: #Satured Backward
        lpwm = 4.4
    elif left < -1.25: # Reverse PWM range
        lpwm = -((-11933+math.sqrt(-5144000*left+28436313))/(2572))
    else: # Stop condition
        lpwm = 7.1
        
    if right >= 5.25: # Saturated forward
        rpwm = 4.4
    elif right > 1.25: # Forward PWM range
        rpwm = -((-11933+math.sqrt(-5144000*left+28436313))/(2572))
    elif right <= -5.25: # Saturated reverse
        rpwm = 10.4
    elif right < -1.5: #reverse PWM range
        rpwm = -((-19.998+math.sqrt(-4.0968*left+22.4080776))/(2.0484))
    else: #stop condition
        rpwm = 7.1

    #print(rpwm)
    #print(lpwm)
    self.r_pwm.ChangeDutyCycle(rpwm)
    self.l_pwm.ChangeDutyCycle(lpwm)


  def odometry_timer_callback(self):

    # Get Info from hardware
    curT=self.get_clock().now()
    duration = curT - self.lastTime
    print(duration)
    deltaT=.1#duration.to_sec()
    self.lastTime=curT


    # Calculate v and w - DEPENDS ON YOUR ROBOT
    self.v = self.current_v
    self.w = self.current_yaw_rate
       
    #Calculate pose update
    self.x = self.x + self.v*deltaT*np.cos(self.theta)
    self.y = self.y + self.v*deltaT*np.sin(self.theta)            
    self.theta = self.theta + self.w*deltaT
        
    #Publish tf transform
    odom_quat = self.quaternion_from_euler(0, 0, self.theta)
    self.publisher_.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            curT,
            "base_link",
            "odom"
        )
        
    #publish odometry message
    msg = Odometry()
    msg.header.stamp = curT.to_msg()
    msg.pose.pose.position.x = self.x
    msg.pose.pose.position.y = self.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = odom_quat[0]
    msg.pose.pose.orientation.y = odom_quat[1]
    msg.pose.pose.orientation.z = odom_quat[2]
    msg.pose.pose.orientation.w = odom_quat[3]
    self.publisher_.publish(msg)



########### MAIN FUNCTION ###############
# The main function is called automatically when the 
# Controller node is launched. It is th entry point 
# for the node

def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node (initializes an instance of the Controller class)
    controller = Controller()
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    self.r_pwm.stop()
    self.l_pwm.stop()
    self.GPIO.cleanup()
    controller.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()






