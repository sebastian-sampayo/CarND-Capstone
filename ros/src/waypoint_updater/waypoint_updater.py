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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        
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
            dx = wp.pose.pose.position.x - ego_x
            dy = wp.pose.pose.position.y - ego_y
            
            # local coordinates of the waypoint wrt the ego
            x_local = dy*math.sin(ego_yaw) + dx*math.cos(ego_yaw)
            y_local = dy*math.cos(ego_yaw) - dx*math.sin(ego_yaw)

            # wheter the wp is ahead of the ego
            is_ahead = x_local > 0. and abs(math.atan2(y_local, x_local)) <= math.pi/4.0
            if is_ahead:
                dist = math.sqrt(x_local**2 + y_local**2)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i

        # logging
        wp_min = self.base_waypoints[min_idx].pose.pose.position 
        log_msg = ("WaypointUpdater::find_closest_waypoint()\n"  
                   "Closest waypoint: %d\n"
                   "ego     x: %f, ego     y: %f\n"
                   "closest x: %f, closest y: %f\n")%(
                    min_idx,
                    ego_x, ego_y, 
                    wp_min.x, wp_min.y)

        rospy.loginfo(log_msg) 
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
