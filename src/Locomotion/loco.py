import RPi.GPIO as GPIO
from time import sleep
#from wpimath.kinematics import DifferentialDriveKinematics
#from wpimath.kinematics import ChassisSpeeds
#from wpimath.units import inchesToMeters
import math

linear = 0
angular = .1
linearmem = 0
angularmem = 0

#kinematics = DifferentialDriveKinematics(Units.inchesToMeters(10.5))

rpin = 33				# PWM pin connected to right motor
lpin = 32                               # PWM pin connected to left motor
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(lpin,GPIO.OUT)
GPIO.setup(rpin,GPIO.OUT)
r_pwm = GPIO.PWM(rpin,50)		#create PWM instance with frequency
l_pwm = GPIO.PWM(lpin,50)		#create PWM instance with frequency
#pi_pwm.start(0)
#sleep(2)	
#pi_pwm.ChangeDutyCycle(25) #provide duty cycle in the range 0-100			
#sleep(50)
#pi_pwm.ChangeDutyCycle(0) #provide duty cycle in the range 0-100

r_pwm.start(7)		#start PWM of required Duty Cycle 
l_pwm.start(7)		#start PWM of required Duty Cycle
while True:
    #for duty in range(74,106,2):#30,110,5
    #    pi_pwm.ChangeDutyCycle(duty/10) #provide duty cycle in the range 0-100
    #    print(duty/10)
    #    sleep(5)
    #    pi_pwm.ChangeDutyCycle(7) 
    #    sleep(1)
    #sleep(5)
    
    
    #convert to left and right wheel speeds
    
    #chassisSpeeds = ChassisSpeeds(linear,0,angular)
    #wheelSpeeds = self.kinematics.toWheelSpeeds(chassisSpeeds)
    if (linear != linearmem) or (angularmem != angular): # compare new velo to old, only run code if different
        linearmem = linear
        angularmem = angular
        #calculate what left and right wheel speeds should be
        left = (linear+angular*5.25)*4*3.14#wheelSpeeds.left*4*3.14
        right = (linear-angular*5.25)*4*3.14#wheelSpeeds.right*4*3.14
        print(left)
        print(right)
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
            
        if right >= 5.25: # Satured foward
            rpwm = 4.4
        elif right > 1.25: # Forward PWM range
            rpwm = -((-11933+math.sqrt(-5144000*left+28436313))/(2572))
        elif right <= -5.25: # Saturated reverse
            rpwm = 10.4
        elif right < -1.5: #reverse PWM range
            rpwm = -((-19.998+math.sqrt(-4.0968*left+22.4080776))/(2.0484))
        else: #stop condition
            rpwm = 7.1

        print(rpwm)
        print(lpwm)
        r_pwm.ChangeDutyCycle(rpwm)
        l_pwm.ChangeDutyCycle(lpwm)



pi_pwm.stop()
GPIO.cleanup()
