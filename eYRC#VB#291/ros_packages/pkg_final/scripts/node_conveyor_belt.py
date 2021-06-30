#! /usr/bin/env python

# ROS Node - Conveyor Belt


from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task6.msg import PackageExist

import rospy

class Conveyor:
    """The sole purpose of this class is to control the conveyor belt."""

    # Constructor
    def __init__(self):

        rospy.init_node('node_conveyor_belt', anonymous=True)

        self._conveyor_speed = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        self._package_exist = False
        self.var_handle_pub = rospy.Publisher('eyrc/vb/package_exist', PackageExist, queue_size=10)
        self.send_obj = PackageExist()
        self.send_obj.package_exist = True

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    #Callback function
    def conveyor_call_back(self, msg):
        """This is a callback function for logical_camera_2 subscription it repeatedly gets data from eyrc/vb/logical_camera_2 topic.
        :param: msg - All the models and their position in vicinity of logical_camera_2)
        """

        self.send_obj.package_exist = self._package_exist
        self.var_handle_pub.publish(self.send_obj)
        count_packages = 0
        for model in msg.models:
            if model.type != 'ur5':
                count_packages += 1

        if count_packages == 0:
            self._conveyor_speed(100)
            self._package_exist = False
            self.flag = 0

        elif self._package_exist:
            self._conveyor_speed(0)
            if self.flag == 0:
                self.flag = 1

        else:
            self.flag = 0
            for model in msg.models:
                if model.type != 'ur5':
                    curr = abs(model.pose.position.y)
                    if curr <= 0.1:
                        self._conveyor_speed(0)
                        self._package_exist = True
                    else:
                        self._conveyor_speed(100)

    #Subscribe to logical camera
    def logical_camera_start(self):
        """This function subscribe to /eyrc/vb/logical_camera_2 topic."""

        rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, self.conveyor_call_back, queue_size=10)


#Main function
def main():

    rospy.sleep(5)
    ur5 = Conveyor()
    ur5.logical_camera_start()

    rospy.spin()


if __name__ == '__main__':
    main()
