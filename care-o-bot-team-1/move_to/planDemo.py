#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

import roslib
roslib.load_manifest('cob_script_server')
roslib.load_manifest('move_to')
import rospy

from simple_script_server import script
from cob_msgs.msg import *
from cob_srvs.srv import *
from test_planning.srv import *
from sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *
import actionlib
import sys

class TaskDemoPlanScript(script):
		
	def Initialize(self):

		# initialize components (not needed for simulation)
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.set_operation_mode("arm", "position")
		self.sss.init("sdh")
		self.sss.init("base")
		
#		self.thumb_sub = rospy.Subscriber("/sdh_thumb_2_bumper/state",ContactsState,self.callback)
		
		# move to initial positions
		
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		
	def Run(self): 

		rospy.wait_for_service('/cob_arm_planning/plan_arm_path')
		callPlan = rospy.ServiceProxy('/cob_arm_planning/plan_arm_path', ComputeArmPlan)


		goal = ComputeArmPlanRequest()
		goal.goal.position = rospy.get_param("/script_server/arm/folded")[0]
		print str(goal.goal.position)
		traj = callPlan(goal)

		print "Got plan"

		client = actionlib.SimpleActionClient("/arm_controller/joint_trajectory_action", JointTrajectoryAction);
		client.wait_for_server()
		client_goal = JointTrajectoryGoal()
		client_goal.trajectory = traj.path
		print str(traj.path)
		#print client_goal
		client.send_goal(client_goal)

		print "Request path"
		#block until reaching goal
		client.wait_for_result()
		print "Done"

if __name__ == "__main__":
	SCRIPT = TaskDemoPlanScript()
	SCRIPT.Start()
