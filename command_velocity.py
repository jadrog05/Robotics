import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
    """Driving my robot
    """

    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("...", Twist) # Creating a publisher with whatever name...
        
    # A function to send velocities until the node is killed
    def send_velocities(self):
        r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        while not rospy.is_shutdown(): # Runnin until killed
            rospy.loginfo("Sending commands")
            twist_msg = Twist() # Creating a new message to send to the robot

            # ... put something relevant into your message

            self.pub.publish(twist_msg) # Sending the message via our publisher
            r.sleep() # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities() # Calling the function
    rospy.spin()
    
    
    
    #!/usr/bin/env python
#roslaunch uol_turtlebot_simulator maze1.launch
import numpy

import cv2
import cv_bridge
import rospy

#green until not seen

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#follow yellow.
#if yellow not found, avoid banging into walls
#turn when faced with red DONE
#if at green, stop DONE

#if yellow found, follow, else object detect
class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laser_callback) #scan
        self.twist = Twist()
        #a,b,c
        self.turning = False
    def laser_callback(self,msg):
        #get laser messages
        #remove 'NaNs'
        clean = [i for i in msg.ranges if str(i) != 'nan']
        #divide field of perception by 3 (left,front,side)
        division = (len(clean)/3)
        a = clean[0:division]
        a = sum(clean[0:division])/division
        b = sum(clean[division:(division*2)])/division
        c = sum(clean[division*2:])/division

        #print scan values
        print(a)
        print(b)
        print(c)

        print('--------')

        #if corner
        #turn the direction of larger gap
        if self.turning == True:
            #keep turning until a = 0.8
            if (a  > 0.8) & (b > 0.8) & (c > 0.8):
                self.turning = False
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
        else:

            if (a < 0.4) & (c < 0.4):
                if (a < c):
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 1
                    print('turning away left- cornered')
                    #rospy.sleep(1.)
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -1
                    print('turning away right- cornered')
                    #rospy.sleep(1.)
                self.turning = True
            #if close to wall
            #turn the direction of larger gap
            elif (b < 0.45):
                self.twist.linear.x = 0.0
                if (a < c):
                    self.twist.angular.z = 1
                    print('turning away left- facing wall')
                    #rospy.sleep(1.)
                else:
                    self.twist.angular.z = -1
                    print('turning away right- facing wall')
                    #rospy.sleep(1.)
                self.turning = True
            #if one side too close, turn the other way
            elif (a < 0.4) & (c > 0.4):
                self.twist.linear.x = 0.0
                self.twist.angular.z = 1
                print('turning away left')
                self.turning = True
                #rospy.sleep(1.)

            elif (a > 0.4) & (c < 0.4):
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1
                print('turning away right')
                #rospy.sleep(1.)
                self.turning = True

            #else move forward
            else:
                print('moving forward')
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
            #rospy.sleep(1.)
        
    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #if green stop
        lower_green = numpy.array([0, 250, 0])
        upper_green = numpy.array([250, 255, 255])
        mask = cv2.inRange(image, lower_green, upper_green)
        h, w, d = image.shape
        search_top = 3*h/4
        # print("search_top =", search_top)
        search_bot = 3*h/4 + 10
        # print("search_bot=", search_bot)
        #circle
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])#circle x
            cy = int(M['m01']/M['m00'])#circley
            cv2.circle(image, (cx, cy), 20, (75, 150, 75), -1)
            err = cx - w/2
            #print(cx)
            self.twist.angular.z = 0
            self.twist.linear.x = 0
            print("Goal Reached!")

        #if red, turn
        lower_red = numpy.array([0, 0, 250])
        upper_red = numpy.array([250, 255, 255])

        #if red turn around
        mask = cv2.inRange(image, lower_red, upper_red)
        h, w, d = image.shape
        search_top = 3*h/4
        # print("search_top =", search_top)
        search_bot = 3*h/4 + 10
        # print("search_bot=", search_bot)
        #circle
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])#circle x
            cy = int(M['m01']/M['m00'])#circley
            cv2.circle(image, (cx, cy), 20, (75, 150, 75), -1)
            err = cx - w/2
            #print(cx)
            self.twist.angular.z = float(err) / 100 # turns here
            print("Danger spotted!")
            # print self.twist.angular.z

        self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("window", image)
        cv2.waitKey(3)


# cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
pub = rospy.Publisher('/cmd_vel',Twist)

rospy.spin()
