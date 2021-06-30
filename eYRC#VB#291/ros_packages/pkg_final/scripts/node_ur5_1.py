#! /usr/bin/env python

# ROS Node - Ur5_1

import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper

import rospkg
from std_srvs.srv import Empty
import yaml
import time
import datetime

from pkg_task6.msg import msgOrderDispatchedAction
from pkg_task6.msg import msgOrderDispatchedResult

import rospy

class Ur5_1:
    """This class connects to Ur5_1 model to send it goals to pick packages from shelf and place it on conveyor belt."""

    # Constructor
    def __init__(self):

        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        self.attach_detach_box = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        rospy.loginfo("Package Path: {}".format(self._file_path))
        rospy.loginfo('\033[94m' + " >>> Ur5_1 init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """This function moves the Ur5_2 arm based on the joint angles provided.
        :param: arg_joint_angles - Joint angles list
        """

        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose

        if flag_plan:
            pass

        else:
            pass

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """This function calls the set_joint_angles function some no. of attempts until it returns True.
        :param: arg_list_joint_angles - Joint angles list, arg_max_attempts - No. of attempts
        """

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """This function open the trajectory file and executes it.
        :param: arg_file_path - file path, arg_file_name - file name
        """

        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)

        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        """This function tries to play the stored trajectory for some number of attempts.
        :param: arg_file_path - file path, arg_file_name - file name, arg_max_attempts - No. of attempts
        """

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    def clear_octomap(self):
        """This function subsribe to clear_octomap service."""

        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)

        return clear_octomap_service_proxy()

    def plan_path_packages(self, package, home_pose):
        """This function activates the vacuum gripper to attach the package to Ur5_1 and to put it on the conveyor belt.
        :param: priority
        """

        self.moveit_hard_play_planned_path_from_file(self._file_path, package, 5)
        param_config_my = rospy.get_param('status')
        while param_config_my['condition'] == "True":
            param_config_my = rospy.get_param('status')

        rospy.set_param('/status/condition', "True")
        rospy.sleep(0.5)
        self.attach_detach_box(True)
        self.moveit_hard_play_planned_path_from_file(self._file_path, home_pose, 5)
        self.attach_detach_box(False)
        rospy.set_param('/status/condition', "False")

        return True



class Ur5_1_Server:
    """This class is a server for action_ur5_1_controller it accepts the goals from ur5_master to control Ur5_1 arm."""

    # Constructor
    def __init__(self):

        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ur5_1_controller',
                                          msgOrderDispatchedAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        # Start the Action Server
        self._as.start()

        self.ur5 = Ur5_1()
        self.home_joint = [0.13682462346610347, -2.4185447511955145, -0.9820108411366579, -1.3116525343785987, 1.570733087543557, 0.1364974244094581]
        self.ur5.hard_set_joint_angles(self.home_joint, 5)

        rospy.loginfo("Started ROS-IoT Bridge Action Server 2.")

    def get_time_str(self):
        """This function gets the current time in string format.
        :return: Current time in string format
        """

        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def on_goal(self, goal_handle):
        """This function is executed when a new goal is recieved.
        :param: goal_handle - Goal information
        """

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        goal_handle.set_accepted()
        result = msgOrderDispatchedResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        row = goal.row
        col = goal.col
        priority = goal.priority
        order_data = goal.order_data

        status = self.ur5.plan_path_packages('zero_to_packagen'+row+col+'.yaml', 'packagen'+row+col+'_to_zero.yaml')

        if status:
            result.flag_success = True
            result.order_id = order_data.order_id
            result.city = order_data.city
            result.item = order_data.item
            result.priority = priority
            result.dispatched_qty = order_data.quantity
            result.cost = order_data.cost
            result.dispatched_status = 'Yes'
            result.dispatched_time = self.get_time_str()
            rospy.loginfo("Publishing to Google Sheet local Successful.")

        else:
            result.flag_success = False
            rospy.logerr("Publishing failed to Google Sheet local")

        rospy.loginfo("Send goal result to client")

        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)

        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """This function will be called when Goal Cancel request is send to the Action Server."""

        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


#Main function
def main():

    rospy.init_node('node_ur5_1', anonymous=True)
    rospy.sleep(5)
    ur5_1_control = Ur5_1_Server()
    rospy.spin()


if __name__ == '__main__':
    main()
