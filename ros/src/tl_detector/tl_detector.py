#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = []
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        
    # ----------- New code -----------
    # Find the closest waypoint to the current position
    # @param[in] waypoints: List of input waypoints from where to search
    # @param[in] pose: Current position, message of type PoseStamped.pose
    # @param[in] ahead: Bool, if True gets the closest waypoint *ahead* of the current position. Else, the actual closest waypoint.
    # @param[out] idx: Index of the closest waypoint in the list waypoints
    # Minor TODO: <Cleanup> Move this helper function to a library, because it is also used by the WaypointUpdater. Avoid duplicated code.
    def get_closest_index(self, waypoints, pose, ahead):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        idx = 0
        min_idx = -1
        min_dist = 10000
        for wp in waypoints:
            # TODO: If the wp is not ahead of the ego, skip it (continue). To accomplish this, I guess we should look at the pose.orientation
            # if (ahead):
                # is_wp_behind = ?
                # if (is_wp_behind):
                #    continue
            dist = dl(wp.pose.pose.position, pose.position)
            if (dist < min_dist):
                min_dist = dist
                min_idx = idx
            idx += 1
        return min_idx
    # ---------------------------------

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # ----------- New code -----------
        return self.get_closest_index(self.waypoints, pose, False)
        # ---------------------------------
        
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            # find the closest visible traffic light (if one exists)
            # ----------- New code -----------
            rospy.loginfo('TLDetector::process_traffic_lights() - car_position: %d', car_position)
            # Get the index in the lights array
            light_idx = self.get_closest_index(self.lights, self.waypoints[car_position].pose.pose, True) if car_position >= 0 else -1
            rospy.loginfo('TLDetector::process_traffic_lights() - light_idx: %d', light_idx)
            # Get the index of the traffic light in the waypoints array
            light_wp = self.get_closest_index(self.waypoints, self.lights[light_idx].pose.pose, True) if light_idx >= 0 else -1
            rospy.loginfo('TLDetector::process_traffic_lights() - light_wp: %d', light_wp)
            # Have we found a traffic light?
            light = (light_wp != -1)
            # ---------------------------------

        state = TrafficLight.UNKNOWN
        if light:
            state = self.get_light_state(light)

        return light_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
