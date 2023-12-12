#!/usr/bin/env python3.8
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospy
import numpy as np
from numpy.linalg import norm
from geometry_msgs.msg import PoseWithCovarianceStamped
#import shapely.geometry
#import cfg
#import init_map

class navigation():
    def __init__(self):
        # Subscriber to listen to current robot position
        rospy.Subscriber('jackal1/odometry/local_filtered', Odometry, self.callback_move_base_feedback) # jackal_velocity_controller odometry/filtered
        # Publisher to publish move_base goals
        self.move_base = actionlib.SimpleActionClient('jackal1/move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        self.offset_pos = [0,0,0]
        self.offset_ori = [0,0,0,0]
        self.offset_set = False

    def pub_goal_pos(self, x, y): #todo use normal pub
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "jackal1/odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base.send_goal(goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.check_goal_reached(x, y):
            rate.sleep()
        self.move_base.cancel_goal()

    def callback_move_base_feedback(self, data):
        try:
            self.pos = data.pose.pose.position
            self.ori = data.pose.pose.orientation
        except:
            pass

    def check_goal_reached(self, x_goal, y_goal):
        try:
            dist = norm(np.array([x_goal, y_goal]) - np.array([self.pos.x, self.pos.y]))
            if dist < 0.3:
                return True
            else:
                return False
        except:
            return False

    def navigate_point_list(self, area_poly):
        for point in area_poly:
            self.pub_goal_pos(point[0], point[1])
            

    def calculate_mapping_path(self, area_poly_pos):
        distance = 1
        path = list()
        x_min = area_poly_pos[0][0]
        x_max = area_poly_pos[0][1]
        y_min = area_poly_pos[0][0]
        y_max = area_poly_pos[0][1]

        for point in area_poly_pos:
            if x_min > point[0]:
                x_min = point[0]
            elif x_max < point[0]:
                x_max = point[0]
            if y_min > point[1]:
                y_min = point[1]
            elif y_max < point[1]:
                y_max = point[1]

        area_poly = shapely.geometry.Polygon(area_poly_pos)

        i = 0
        toggle = True
        while x_min + i <= x_max:  # (pl)-------line------(pr)
            pr = [x_min + i, y_min]
            pl = [x_min + i, y_max]
            pr, pl = self.cut_line_to_poly(pr, pl, area_poly)
            if toggle:
                path.append(pr)
                path.append(pl)
                toggle = False
            else:
                path.append(pl)
                path.append(pr)
                toggle = True
            i += 1

        return path

    def cut_line_to_poly(self, p1, p2, area_poly):
        l = shapely.geometry.LineString([p1, p2])
        ls = l.intersection(area_poly)
        p1 = np.array(ls.coords)[0]
        p2 = np.array(ls.coords)[-1]

        return p1, p2

    def pub_amcl_pose_estimate(self):
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        rate = rospy.Rate(10)
        i = 10
        while not rospy.is_shutdown() and i > 0:
            data = PoseWithCovarianceStamped()
            data.header.frame_id = "map"
            data.pose.pose.position.x = init_map.INIT_POS_X
            data.pose.pose.position.y = init_map.INIT_POS_Y
            data.pose.pose.orientation.z = init_map.INIT_POS_Z
            data.pose.pose.orientation.w = init_map.INIT_POS_W
            pub.publish(data)
            rate.sleep()
            i += -1

    def write_pos_in_cfg(self, area):
        f = open(cfg.PACKAGE_DIR + "/scripts/init_map.py", 'r+')
        f.write("INIT_POS_X =" + str(self.pos.x) + "\n")
        f.write("INIT_POS_Y =" + str(self.pos.y) + "\n")
        f.write("INIT_POS_Z =" + str(self.ori.z) + "\n")
        f.write("INIT_POS_W =" + str(self.ori.w) + "\n")
        f.write("AREA =" + str(area) + "\n")
        f.close()
        
if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    navigate = navigation()
    while 1:
        point_list = [(1,1),(3,2),(2,3)]
        navigate.navigate_point_list(point_list)