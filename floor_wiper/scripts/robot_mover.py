#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
import threading

class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover', anonymous=True, log_level=rospy.DEBUG)
        
        
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.dirt_positions = [] 
        self.predicted_positions = []  
        self.current_target = None  
        self.battery_level = 100.0  
        self.charging_station_position = {'x': 2, 'y': 0}  


        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cleaned_pub = rospy.Publisher('/cleaned_location', Point, queue_size=10)
        rospy.Subscriber('/dirt_location', Point, self.dirt_callback)
        rospy.Subscriber('/predicted_dirt_location', Point, self.predicted_dirt_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        
        self.move_thread = threading.Thread(target=self.move_to_position)
        self.move_thread.start()

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Update the robot's current position
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        self.current_position['theta'] = yaw
    def dirt_callback(self, msg):
        self.dirt_positions.append({'x': msg.x, 'y': msg.y})
        rospy.logdebug(f"New dirt position added: X: {msg.x}, Y: {msg.y}")

    def predicted_dirt_callback(self, msg):
        self.predicted_positions.append({'x': msg.x, 'y': msg.y})
        rospy.logdebug(f"New predicted dirt position added: X: {msg.x}, Y: {msg.y}")

    def move_to_position(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.current_target:
                self.current_target = self.get_next_target()
            if self.current_target:
                self.navigate_to_target()
            rate.sleep()

    def get_next_target(self):
        if self.battery_level < 20:
            rospy.logdebug("Battery low, switching to charging station as target.")
            return self.charging_station_position
        if self.dirt_positions:
            rospy.logdebug("Selecting next dirt position as target.")
            return self.dirt_positions.pop(0)
        if self.predicted_positions:
            rospy.logdebug("Selecting next predicted position as target.")
            return self.predicted_positions.pop(0)
        return None

    def navigate_to_target(self):
        dx = self.current_target['x'] - self.current_position['x']
        dy = self.current_target['y'] - self.current_position['y']
        distance = sqrt(dx**2 + dy**2)
        angle_to_target = atan2(dy, dx)
        angle_error = angle_to_target - self.current_position['theta']

        while angle_error > pi:
            angle_error -= 2 * pi
        while angle_error < -pi:
            angle_error += 2 * pi

        if distance < 0.1:
            self.handle_target_reached()
        else:
            self.send_movement_commands(distance, angle_error)

    def handle_target_reached(self):
        if self.current_target == self.charging_station_position:
            rospy.loginfo("Reached charging station. Recharging...")
            self.battery_level = 100
        elif self.dirt_positions and not self.current_target:
            rospy.loginfo("Going to the next predicted Dirt spot")
        else:
            rospy.loginfo("Reached dirt spot. Cleaning...")
            self.publish_cleaned_location(self.current_target['x'], self.current_target['y'])
        self.current_target = None

    def publish_cleaned_location(self, x, y):
        cleaned_location = Point(x=x, y=y, z=0)
        self.cleaned_pub.publish(cleaned_location)
        rospy.logdebug(f"cleaned location: X: {x}, Y: {y}")

    def send_movement_commands(self, distance, angle_error):
        vel_msg = Twist()
        vel_msg.linear.x = min(3* distance, 3)
        vel_msg.angular.z = 5.0 * angle_error
        self.velocity_publisher.publish(vel_msg)
        self.battery_level -= 0.1*distance

if __name__ == '__main__':
    try:
        robot_mover = RobotMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
