#!/usr/bin/env python

#this code applicate navigation
#it can move continuously
#first,second,third dict have destination information 
import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

first = {'x':2.986258, 'y':-0.317417, 'z':0.690547, 'w':0.723288}
second = {'x':3.685679, 'y':0.647017, 'z':-0.891777, 'w':0.452476}
third = {'x':0.056933, 'y':-0.009652, 'z':-0.005608, 'w':0.999984}

def movebase_client(x):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    if x==1:
        goal.target_pose.pose.position.x = first['x']
        goal.target_pose.pose.position.y = first['y']
        goal.target_pose.pose.orientation.z = first['z']
        goal.target_pose.pose.orientation.w = first['w']
    elif x==2:
        goal.target_pose.pose.position.x = second['x']
        goal.target_pose.pose.position.y = second['y']
        goal.target_pose.pose.orientation.z = second['z']
        goal.target_pose.pose.orientation.w = second['w']   
    else:
        goal.target_pose.pose.position.x = third['x']
        goal.target_pose.pose.position.y = third['y']
        goal.target_pose.pose.orientation.z = third['z']
        goal.target_pose.pose.orientation.w = third['w']
        
        

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        movebase_client(1)
        rospy.loginfo("first Goal execution done!")
        rospy.sleep(3)
        movebase_client(2)
        rospy.loginfo("second Goal execution done!")
        rospy.sleep(3)
        movebase_client(3)
        rospy.loginfo("third Goal execution done!")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
