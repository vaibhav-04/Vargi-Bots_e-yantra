#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import threading
import json
import requests
import actionlib
import rospy

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_ros_iot_bridge.msg import msgGoogleSpreadsheetAction
from pkg_ros_iot_bridge.msg import msgGoogleSpreadsheetResult

from pkg_ros_iot_bridge.msg import msgMqttSub
from pyiot import iot


class IotRosBridgeActionServer1:
    """Server1 handles the goals for mqtt subscription and pushes incoming data to google sheet."""

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_incoming_order_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']

        # Initialize ROS Topic Publication
        self._handle_ros_pub = rospy.Publisher(self._config_incoming_order_topic, msgMqttSub, queue_size=10)

        ret = iot.mqtt_subscribe_thread_start(self.incoming_order_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server 1.")

    # This is a callback function for MQTT Subscriptions
    def incoming_order_callback(self, client, userdata, message):
        """Incoming orders callback function.
        This function also pushes the incoming orders data on google spreadsheet.
        :param: message from mqtt
        """

        payload = str(message.payload.decode("utf-8"))
        dict_obj = json.loads(payload)

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_incoming_order = msgMqttSub()
        msg_incoming_order.order_id = str(dict_obj.get('order_id'))
        msg_incoming_order.order_time = str(dict_obj.get('order_time'))
        msg_incoming_order.item = str(dict_obj.get('item'))
        msg_incoming_order.quantity = str(dict_obj.get('qty'))
        msg_incoming_order.city = str(dict_obj.get('city'))
        msg_incoming_order.order_time = str(dict_obj.get('order_time'))
        msg_incoming_order.lon = str(dict_obj.get('lon'))
        msg_incoming_order.lat = str(dict_obj.get('lat'))
        cost = 150
        priority = "LP"
        if msg_incoming_order.item == "Medicine":
            cost = 450
            priority = "HP"
        elif msg_incoming_order.item == "Food":
            cost = 300
            priority = "MP"
        msg_incoming_order.cost = str(cost)

        self._handle_ros_pub.publish(msg_incoming_order)
        # Push to local google sheet
        update_sheet_local = self.push_to_google_sheet(self._config_google_apps_spread_sheet_id, {"id": "IncomingOrders",\
                             "Team Id": "VB#0291",\
                             "Unique Id": "dbMBsyVV",\
                             "Order ID": msg_incoming_order.order_id,\
                             "Order Date and Time": msg_incoming_order.order_time,\
                             "Item": msg_incoming_order.item,\
                             "Priority": priority,\
                             "Order Quantity": msg_incoming_order.quantity,\
                             "City": msg_incoming_order.city,\
                             "Longitude": msg_incoming_order.lon,\
                             "Latitude": msg_incoming_order.lat,\
                             "Cost": cost})

        if update_sheet_local == "success":
            rospy.loginfo("Publishing to Google Sheet local Successful.")
        else:
            rospy.logerr("Publishing failed to Google Sheet local")

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        """This function is called when Action Server receives Goals.
        :param: goal_handle
        """

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        """This function is called to start separate threads to process Goals."""

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.incoming_order_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")

        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will push the goal to google sheet
    def push_to_google_sheet(self, web_app_id, content):
        """This function will push the goals to google sheet.
        :param: web_app_id, content  as variadic arguments
        """

        url = "https://script.google.com/macros/s/"+web_app_id+"/exec"
        response = requests.get(url, params=content)
        return response.content

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """This function will be called when Goal Cancel request is send to the Action Server."""

        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()



class IotRosBridgeActionServer2:
    """Server2 handles the goals for pushing data to various google spreadsheet."""

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot_spreadsheet',
                                          msgGoogleSpreadsheetAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']


        # Start the Action Server
        self._as.start()
        rospy.loginfo("Started ROS-IoT Bridge Action Server 2.")

    def on_goal(self, goal_handle):
        """process the goals coming from action clients.
        Here separate threads are not created to handle the goals.
        :param: goal handle"""

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        goal_handle.set_accepted()
        result = msgGoogleSpreadsheetResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        rospy.logwarn("HTTPS PUB Goal ID: " + str(goal_id.id))

        if goal.sheet_id == "Inventory":
            update_sheet_local = self.push_to_google_sheet(self._config_google_apps_spread_sheet_id, {"id": "Inventory",\
                     "Team Id": "VB#0291",\
                     "Unique Id": "dbMBsyVV",\
                     "SKU": goal.data[0],\
                     "Item": goal.data[1],\
                     "Priority": goal.data[2],\
                     "Storage Number": goal.data[3],\
                     "Cost": goal.data[4],\
                     "Quantity": goal.data[5]})

        elif goal.sheet_id == "OrdersDispatched":
            update_sheet_local = self.push_to_google_sheet(self._config_google_apps_spread_sheet_id, {"id": "OrdersDispatched",\
                     "Team Id": "VB#0291",\
                     "Unique Id": "dbMBsyVV",\
                     "Order ID": goal.data[0],\
                     "City": goal.data[1],\
                     "Item": goal.data[2],\
                     "Priority": goal.data[3],\
                     "Dispatch Quantity": goal.data[4],\
                     "Cost": goal.data[5],\
                     "Dispatch Status": goal.data[6],\
                     "Dispatch Date and Time": goal.data[7]})

        elif goal.sheet_id == "OrdersShipped":
            update_sheet_local = self.push_to_google_sheet(self._config_google_apps_spread_sheet_id, {"id": "OrdersShipped",\
                     "Team Id": "VB#0291",\
                     "Unique Id": "dbMBsyVV",\
                     "Order ID": goal.data[0],\
                     "City": goal.data[1],\
                     "Item": goal.data[2],\
                     "Priority": goal.data[3],\
                     "Shipped Quantity": goal.data[4],\
                     "Cost": goal.data[5],\
                     "Shipped Status": goal.data[6],\
                     "Shipped Date and Time": goal.data[7],\
                     "Estimated Time of Delivery": goal.data[8]})

        else:
            rospy.loginfo("Invalid Sheet Id")

        if update_sheet_local == "success":
            result.flag_success = True
            rospy.loginfo("Publishing to Google Sheet local Successful.")
        else:
            result.flag_success = False
            rospy.logerr("Publishing failed to Google Sheet local")

        rospy.loginfo("Send goal result to client")

        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will push the goal to google sheet
    def push_to_google_sheet(self, web_app_id, content):
        """This function will push the goal to google sheet.
        :param:web_app_id, content as variadic argument
        """

        url = "https://script.google.com/macros/s/"+web_app_id+"/exec"
        response = requests.get(url, params=content)
        return response.content

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """This function will be called when Goal Cancel request is send to the Action Server."""

        rospy.loginfo("Received cancel request.")
        # goal_id = goal_handle.get_goal_id()

# Main
def main():
    rospy.init_node('node_ros_iot_action_server')

    action_server1 = IotRosBridgeActionServer1()
    action_server2 = IotRosBridgeActionServer2()

    rospy.spin()

if __name__ == '__main__':
    main()
