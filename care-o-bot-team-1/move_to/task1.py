#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script


class Task1Script(script):
		
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
#		self.sss.wait_for_input()
		#self.sss.move("base","home")
		
	def Run(self): 
	
                self.sss.move("arm", "folded")
                self.sss.move("base", "home")
                self.sss.move("base", "cob1initial0")
                self.sss.move("base", "cob1initial")
                self.sss.move("arm", "cob1initial")
		self.sss.move("sdh", "cylopen")
                self.sss.move("base", "cob1pregrasp")
                self.sss.move("base", "cob1grasp")
		self.sss.move("sdh", "cylclosed")
                self.sss.move("base", "cob1lbrdelivery")
                self.sss.wait_for_input()
		self.sss.move("sdh", "cylopen")
		self.sss.move("sdh", "cylclosed")
                self.sss.move("arm", "folded")
                self.sss.move("base", "home")

if __name__ == "__main__":
	SCRIPT = Task1Script()
	SCRIPT.Start()
