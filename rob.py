#!/usr/bin/env python


import rospy
import ros
import actionlib
import std_msgs
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math
import sys

def lim(q):
	return max(math.radians(-110), min(math.radians(110), q)); 

def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
	"""
	
	d1 = 10.0; # cm (height of 2nd joint)
	a1 = 0.0; # (distance along "y-axis" to 2nd joint)
	a2 = 20.0; # (distance between 2nd and 3rd joints)
	d4 = 20.0; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	xc = xyz[0];
	yc = xyz[1];
	zc = xyz[2];

	q1 = lim(math.atan2(yc, xc));

	r2 = math.pow((xc - a1 * math.cos(q1)),2) + math.pow((yc - a1 * math.sin(q1)),2);
	s = (zc - d1);
	D = (r2 + s*s - a2*a2 - d4*d4)/(2 * a2 * d4);
	
	Udr = 1 - D*D
	
	q3 = (lim(math.atan2(-math.sqrt(Udr), D)));
	q2 = lim(math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(q3), a2 + d4 * math.cos(q3)));
	q4 = 0;
		
	print q1
	print q2
	print q3
	print q4
	
	return q1,q2,q3,q4

class crustcrawlerNode:
	server_name = '/arm_controller/follow_joint_trajectory'
	name = 'itrob1'
	node = name + '_rob'
	topic = '/' + name
	Joints = 5			#Number of joints on Robot to control
	offset_Table = [0, 0, 0]	#Table for offset in cam
	xyz_Table = [0, 0, 0]		#Table for raw XYZ data
	calibrated_Table = [0, 0, 0]	#Table for calibrated data
	default_Pos = [0, 0, 0, 0,-1] 	#-1 for open gripper
	q_Vals = [0, 0, 0, 0, 0]	#Rads for motor angles

	def __init__(self):
		self.client = actionlib.SimpleActionClient(self.server_name, FollowJointTrajectoryAction)
	
	def createTraj(self, point):
		self.joints_position = []
		self.names =["joint1", "joint2", "joint3", "joint4", "gripper"]
		dur = rospy.Duration(1)
		pointTraj = JointTrajectoryPoint(positions=point, velocities=[0.5]*self.Joints, time_from_start=dur)
		self.joints_position.append(pointTraj)
		self.TrajPoint = JointTrajectory(joint_names=self.names, points=self.joints_position)
		self.TrajGoal = FollowJointTrajectoryGoal(trajectory=self.TrajPoint, goal_time_tolerance=rospy.Duration(3))
	
	def moveToPoint(self):
		self.setInverseKinematics(self.calibrated_Table)
		self.createTraj(point=self.q_Vals)
		self.sendCommand()

	def moveToDefault(self):
		self.createTraj(point=self.default_Pos)
		self.sendCommand()

	def closeGripper(self):				#Close gripper
		self.createTraj(point=[self.q_Vals[0], 
					self.q_Vals[1], 
					self.q_Vals[2], 
					self.q_Vals[3], 1])
		self.sendCommand()

	def openGripper(self):				#Open gripper
		self.createTraj(point=[self.q_Vals[0], 
					self.q_Vals[1], 
					self.q_Vals[2],
					self.q_Vals[3], -1])
		self.sendCommand()
	
	def setInverseKinematics(self, table):
		self.q_Vals = invkin(table)
	
	def returnToDefaultPoint(self):
		self.createTraj(point=self.default_Pos)
		self.sendCommand()

	def setOffsetTable(self, table):
		self.offset_Table = table

	def setCalibration(self):
		self.calibrated_Table = [self.xyz_Table[0]-self.offset_Table[0],
					self.xyz_Table[1]-self.offset_Table[1],
					self.xyz_Table[2]-self.offset_Table[2]]

	def setXYZTable(self, table):
		self.xyz_Table = table
	
	def sendCommand(self):
		self.client.wait_for_server()
		self.client.send_goal(self.TrajGoal)		
		self.client.wait_for_result()
		print self.client.get_result()
	
	def homePos(self):
		self.moveToDefault()

	def moveRobot(self, data):
		self.setXYZTable([data.data[0], data.data[1], 0])
		self.setCalibration()
		self.moveToPoint()
		self.closeGripper()
		self.setXYZTable([data.data[2], data.data[3], 0])
		self.setCalibration()
		self.moveToPoint()
		self.openGripper()
		self.homePos()

	def calibrateRobot(self, data):
		self.setOffsetTable(data.data)

	def main(self, argv):
		rospy.init_node(self.node)
		self.subToMove = ros.subscriber(self.topic + "/rob/move", std_msgs.msg.Float32MultiArray, self.moveRobot)
		self.subToHome = ros.subscriber(self.topic + "/rob/home", std_msgs.msg.Empty, self.homePos)
		self.subToZero = ros.subscriber(self.topic + "/rob/zero", std_msgs.msg.Float32MultiArray, self.calibrateRobot)
		self.xyz_Table = [0, 0, 0]
		self.offset_Table = [0, 0, 0]
		rospy.spin()



if __name__ == "__main__":
	try:
		run = crustcrawlerNode()
		run.main(sys.argv)
	except rospy.ROSInterruptException:
		pass

