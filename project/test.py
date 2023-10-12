#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg  import Odometry 	#try to use odometry but fail
from geometry_msgs.msg import PoseWithCovarianceStamped
import Queue

master_pos = {'x': 3.3, 'y' : -0.856}	# master position
standby_pos = {'x': 0.64, 'y' : 2.08}	# when no request has been send the robot will wait at this position
positions = {"fridge":{'x': -4.95, 'y' : 1.21},"tea table":{'x': 2.07, 'y' : -3.37},"bed":{'x': -3.65, 'y' : -3.91}}	#positions

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

# bring stuff from position pos to position mas 
# if the flag value f is 0, means that whole mission isn't over, there are other stuff to deal with
# when f=0, won't go back to standby position sta

def bring(pos,mas,sta,f):
    #go to pos to fetch the object
    pos1 = pos
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    rospy.loginfo("Bringing from (%s, %s) pose", pos1['x'], pos1['y'])
    success = navigator.goto(pos1, quaternion)
    if success:
        rospy.loginfo("Reached the target place")
    else:
        rospy.loginfo("The base failed to reach")  
    rospy.sleep(1)	# Sleep to give the last log messages time to be sent
    # return to master
    pos2 = mas
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    rospy.loginfo("Bringing from (%s, %s) pose", pos2['x'], pos2['y'])
    success = navigator.goto(pos2, quaternion)
    if success:
        rospy.loginfo("Bring back the stuff")
    else:
        rospy.loginfo("The base failed to reach")
    rospy.sleep(1)
    # if the mission is finish, go back to standby spot and wait for next call
    if f == 1:
	pos3 = sta
   	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    	rospy.loginfo("Going to (%s, %s) pose", pos3['x'], pos3['y'])
    	success = navigator.goto(pos3, quaternion)
    	if success:
            rospy.loginfo("Go back to standby spot, please call me if you have any need.")
   	else:
            rospy.loginfo("The base failed to reach")  
        rospy.sleep(1)

# put stuff from position mas to position pos 
# if the flag value f is 0, means that whole mission isn't over, there are other stuff to deal with
# when f=0, won't go back to standby position sta

def put(mas,pos,sta,f):
    #go to master to get the object
    pos1 = {'x': 2.5, 'y' : -0.856}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    rospy.loginfo("Puting at (%s, %s) pose", pos1['x'], pos1['y'])
    success = navigator.goto(pos1, quaternion)
    if success:
        rospy.loginfo("Get the object")
    else:
        rospy.loginfo("The base failed to reach")  
    rospy.sleep(1)	# Sleep to give the last log messages time to be sent
    #go to target position to put it down
    pos2 = pos
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    rospy.loginfo("Puting at (%s, %s) pose", pos2['x'], pos2['y'])
    success = navigator.goto(pos2, quaternion)
    if success:
        rospy.loginfo("I put it down")
    else:
        rospy.loginfo("The base failed to reach")
    rospy.sleep(1)
    #if the mission is finish, go back to standby spot and wait for next call
    if f == 1:
	pos3 = sta
   	quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    	rospy.loginfo("Going to (%s, %s) pose", pos3['x'], pos3['y'])
    	success = navigator.goto(pos3, quaternion)
    	if success:
            rospy.loginfo("Go back to standby spot, please call me if you have any need.")
   	else:
            rospy.loginfo("The base failed to reach")  
        rospy.sleep(1)

if __name__ == '__main__':
    try:
	# initializing
	global master_pos
	global standby_pos	
	rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        
	# tried to use odometry and frame transformation
	'''
	var=rospy.wait_for_message('/odom',Odometry,timeout=5)#
	print(var.pose.pose.position.x)#
 	print(var.pose.pose.position.y)#
	'''
	
	q = Queue.Queue()	# to do list queue
	print("have a good day, my master!")	
	while 1:	#outside loop to decide whether to shut down
	# continously do this unless master input y to exit the program
	    while 1:	# inner loop get requests from master
	    	kb = raw_input("what else can I do for you?")
		# ask for task, and add mission to queue
		# if it finds word of bring in the input, add "bring" and target position to queue q
	    	if kb.find("bring")>=0 :
		    #print(kb.find("bring"))
		    if kb.find("fridge")>0 :
		    	q.put(["bring","fridge"])
		    elif kb.find("tea table")>0 :
		        q.put(["bring","tea table"])
	 	    elif kb.find("bed")>0 :
		    	q.put(["bring","bed"])		
		    else:
		    	print("sorry, my creater didn't permit me to bring stuff from there.")
		# if it finds word of put in the input, add "put" and target position to queue q
	    	elif kb.find("put")>=0 :
		    if kb.find("fridge")>0 :
		    	q.put(["put","fridge"])
		    elif kb.find("tea table")>0 :
		    	q.put(["put","tea table"])
	 	    elif kb.find("bed")>0 :
		    	q.put(["put","bed"])		
		    else:
		    	print("sorry, my creater didn't permit me to put stuff at there.")
		# if it finds no or didn't find "bring" or "put", break the inner loop
	    	elif kb.find("no")>=0 or kb.find("No")>=0 :
		    print("please wait for a little while, I will do as you say.")
		    break
	    	else :
		    print("I'm sorry, I don't understand.")
		    break

	    # finish requests in queue until it's empty
	    while not q.empty():
		l = q.qsize()
		print(l," thing to do")
    		m = q.get()
		if m[0] == "bring" :
		    if l>1 :
		    	bring(positions[m[1]],master_pos,standby_pos,0)
		    else :
			bring(positions[m[1]],master_pos,standby_pos,1)
		elif m[0] == "put" :
		    if l>1 :
			put(master_pos,positions[m[1]],standby_pos,0)
		    else :
		    	put(master_pos,positions[m[1]],standby_pos,1)
	    # ask if master want to shut down, if not, return and ask for requests again
	    kb_ = raw_input("do you need me to leave? [y/n]")
	    if kb_.find('y')>=0 or kb_.find('Y')>=0 :	# shut down
		print("goodbye, my master!")
	        exit(0)

	    
	    '''
	    bring(pos["bed"],master_pos,standby_pos,0)
	    put(master_pos,pos["tea table"],standby_pos,0)
	    bring(pos["fridge"],master_pos,standby_pos,1)
	    '''

	
	
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

