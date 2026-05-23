#!/usr/bin/env python3
"""Load and start steering joint controllers via controller_manager services."""
import rospy
from controller_manager_msgs.srv import LoadController, ListControllers, SwitchController, SwitchControllerRequest

rospy.init_node("load_steering_controllers")

# Wait for controller_manager services
rospy.wait_for_service("/controller_manager/load_controller")
rospy.wait_for_service("/controller_manager/switch_controller")
rospy.wait_for_service("/controller_manager/list_controllers")

load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
list_ctrl = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)

# Load controllers
for name in ["joint_state_controller", "steering_joint_position_controller"]:
    try:
        resp = load(name)
        rospy.loginfo("Loaded %s: %s", name, "ok" if resp.ok else "fail: " + resp.error_string)
    except rospy.ServiceException as e:
        rospy.logwarn("Load %s failed: %s", name, e)

# Start controllers
req = SwitchControllerRequest()
req.start_controllers = ["joint_state_controller", "steering_joint_position_controller"]
req.stop_controllers = []
req.strictness = SwitchControllerRequest.BEST_EFFORT
try:
    resp = switch(req)
    rospy.loginfo("Started controllers: ok=%s", resp.ok)
except rospy.ServiceException as e:
    rospy.logwarn("Switch failed: %s", e)

# Verify
ctrl = list_ctrl()
for c in ctrl.controller:
    rospy.loginfo("  %s: state=%s type=%s", c.name, c.state, c.type)
