#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 10 #200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # These are the map base waypoints
        self.base_waypoints = []
         
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        waypoints = self.calculate_final_waypoints(msg)
        self.publish(waypoints)
        rospy.loginfo('WaypointUpdater::pose_cb() - Final Waypoints published')

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store map waypoints in a class member
        # Assumption: base waypoints are published only once in the beginning of the simulation, so this callback runs only once.
        self.base_waypoints = waypoints.waypoints
        rospy.loginfo('WaypointUpdater::waypoints_cb() - Base Waypoints loaded')

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

        
    def find_closest_waypoint(self, msg):
        """
        params: msg, PoseStamped
        return: closest index
        """
        # parse the msg to get ego location
        ego_x = msg.pose.position.x
        ego_y = msg.pose.position.y 
        ort = msg.pose.orientation
        ego_yaw = tf.transformations.euler_from_quaternion([ort.x, ort.y, ort.z, ort.w])[-1]

        # loop til we find the closest
        min_idx = -1
        min_dist = 999999 
        for i, wp in enumerate(self.base_waypoints):
            dx = ego_x - wp.pose.pose.position.x 
            dy = ego_y - wp.pose.pose.position.y
            
            # local coordinates
            x_local = dy*math.sin(ego_yaw) + dx*math.cos(ego_yaw)
            y_local = dy*math.cos(ego_yaw) - dx*math.sin(ego_yaw)

            # wheter the car is behind wp
            is_behind = x_local > 0 and abs(math.atan(x_local/y_local) <= math.pi/4 
            if is_behind:
                dist = math.sqrt(x_local**2 + y_local**2)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = idx
        # logging
        if (min_idx == -1):
            rospy.logerr("WaypointUpdater::find_closest_waypoint() - Closest waypoint not found")
        return min_idx
    
    def calculate_final_waypoints(self, msg):
        """
        params: msg, PostStamped    
        """
        # TODO: Design the actual logic for this function after implementing traffic and obstacle detection.
        closest_idx = self.find_closest_waypoint(msg)
        # TODO: closest_idx + LOOKAHEAD_WPS could go out of limit. Guard against it and handle that case!
        waypoints = self.base_waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        return waypoints
        
    def publish(self, waypoints):
        """
        Publish waypoints into /final_waypoints topic
        params: waypoints, List of final waypoints
        """ 
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
