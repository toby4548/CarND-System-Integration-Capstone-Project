#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.saved_base_waypoints = None
	self.current_pose = None
        self.num_waypoints = None
	self.speed = 4.0	
		
        rospy.spin()

    def pose_cb(self, msg):
	'''
	Message Type :geometry_msgs/PoseStamped
	PoseStamped
	    header
	    pose
	'''
        # TODO: Implement
        # save current pose from msg to class
	self.current_pose = msg

        #check waypoints availability
        if self.saved_base_waypoints is None:
            return

	# seach for the closest point
        closest_waypoint, closest_index = self.find_closest_waypoint_with_index()

        #Setup test publish
	lane = Lane()
        
	for i in range(LOOKAHEAD_WPS):
	    new_waypoint = Waypoint()
            new_waypoint.pose.pose.position = self.saved_base_waypoints.waypoints[(closest_index+i) % self.num_waypoints].pose.pose.position
	    new_waypoint.twist.twist.linear.x = self.speed
	    lane.waypoints.append(new_waypoint)


	self.final_waypoints_pub.publish(lane) 
    
    def waypoints_cb(self, waypoints):
        '''
	###Lane on /base_waypoints only publish once###
	Message Type: styx_msgs/Lane
	Lane 
	header
	waypoints[]
	    pose
	    twist
			
	'''
	# TODO: Implement
	self.saved_base_waypoints = waypoints
        self.num_waypoints = len(waypoints.waypoints)

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
        '''
	return the distance of the track
	input: waypoints, index of waypoint1 and waypoint2
	output: distance in between
	'''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def check_position_distance(self, p1, p2):
        '''
        return distance between two position msg
        '''
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(p1,p2)

    def check_waypoint_in_front(self, waypoint_index):
        '''
        in a small range, check if the waypoint is in front of the car of not
        input: waypoint_index, current position
        output: bool (True if in front)
        '''
        waypoint = self.saved_base_waypoints.waypoints[waypoint_index]
        next_waypoint = self.saved_base_waypoints.waypoints[(waypoint_index+1)%self.num_waypoints]
        vec1 = [(waypoint.pose.pose.position.x - self.current_pose.pose.position.x),
                (waypoint.pose.pose.position.y - self.current_pose.pose.position.y)]
        vec2 = [(next_waypoint.pose.pose.position.x - self.current_pose.pose.position.x),
                (next_waypoint.pose.pose.position.y - self.current_pose.pose.position.y)]
        
        return (vec1[0]*vec2[0]+vec1[1]+vec2[1])>0

    def find_closest_waypoint_with_index(self):
        '''
        search the closest waypoint to current position
        input: waypoints and current position
        output: index of closest waypoint
        '''
        closest_index = None
        closest_distance = 1e10
        
        for index, waypoint in enumerate(self.saved_base_waypoints.waypoints):
            distance_tmp = self.check_position_distance(waypoint.pose.pose.position, self.current_pose.pose.position)
            if distance_tmp < closest_distance:
                closest_distance = distance_tmp
                closest_index = index
        if ~self.check_waypoint_in_front(closest_index):
            closest_index = (closest_index+1) % self.num_waypoints
        return self.saved_base_waypoints.waypoints[closest_index], closest_index
        



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
