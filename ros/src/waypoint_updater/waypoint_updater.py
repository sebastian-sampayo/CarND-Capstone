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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        while not rospy.is_shutdown():
            waypoints = self.get_final_waypoints(msg)
            self.publish(waypoints)
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        while not self.base_waypoints:
            self.base_waypoints = waypoints.waypoints 
        pass

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

    def find_closest_idx(self, msg):
        return 0
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
            is_behind = x_local > 0 and abs(math.atan(x_local/y_local)) <= math.pi/4 
            if is_behind:
                dist = math.sqrt(x_local**2 + y_local**2)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = idx

        # logging
        if (min_idx == -1):
            rospy.logerr("WaypointUpdater::find_closest_waypoint() - Closest waypoint not found")
        return min_idx



    def get_final_waypoints(self, msg):
        closest_idx = self.find_closest_idx(msg)
        return self.base_waypoints[closest_idx : closest_idx+LOOKAHEAD_WPS]


    def publish(self, wps):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = wps
        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
