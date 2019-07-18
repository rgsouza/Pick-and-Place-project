#!/usr/bin/env python
#
# Last modification: 20 Dec. 2018
# Author: Rayanne Souza and Franklin Fernandez


import moveit_commander
from geometry_msgs.msg import Pose, PointStamped, TransformStamped
from std_msgs.msg import Float64, Header
from arbotix_msgs.msg import Analog
from tf2_msgs.msg import TFMessage 
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_from_euler
import time, rospy, image_geometry, numpy
import tf2_ros
import tf2_geometry_msgs
import tf,  math


# Global variables

global model

# Initial parameters

gripper = None
cxm = None
cym = None
cam_info = None

# Initialize camera

model = image_geometry.PinholeCameraModel()

# Initial position

pose_target = Pose()


# Callbacks and util functions
def callback_camera(data):
 global model
 model.fromCameraInfo(data)

def callback_force(msg):
 global force
 force = msg.value
 
def compute_position(p, q):
 z = -0.08
 var_lambda =(z - p.point.z)/(q.point.z-p.point.z)
 x = p.point.x+var_lambda*(q.point.x-p.point.x)
 y = p.point.y+var_lambda*(q.point.y-p.point.y)
 pos = [x,y,z]
 return pos

def callback_goal(bounds):
 global cxm
 global cym
 print bounds
 cxm = int((bounds[1] + bounds[3])*0.5) 
 cym = int((bounds[0] + bounds[2])*0.5)
 print cxm, cym


def callback_gripper(msg):
 global gripper 
 
 gripper = msg.transforms
 if(len(gripper) == 5):
  print(gripper)
# print(gripper)

def go_to_pinitial(move_group):
 pose_initial = Pose()
 pose_initial.position.x = 0.062
 pose_initial.position.y = 0.003
 pose_initial.position.z = 0.274
 pose_initial.orientation.x = 0.001
 pose_initial.orientation.y = -0.067
 pose_initial.orientation.z = 0.020
 pose_initial.orientation.w = 0.997
 move_group.set_pose_target(pose_initial)

 move_group.go(wait=True)
 time.sleep(0.5)

def go_to_pfinal(move_group):
 pose_initial = Pose()
 pose_initial.position.x = 0.300
 pose_initial.position.y = -0.069
 pose_initial.position.z = 0.110
 pose_initial.orientation.x = 0.023
 pose_initial.orientation.y = 0.207
 pose_initial.orientation.z = -0.110
 pose_initial.orientation.w = 0.972
 move_group.set_pose_target(pose_initial)

 move_group.go(wait=True)
 time.sleep(0.5)

def get_object():
 global pub
# Force control routine
 a = 0.5
 delta = 0.1
 while(force < 100) and not rospy.is_shutdown():
  if(a < 1.2):
   pub.publish(a)
   a = a + delta
   time.sleep(0.2)
  else:
   break


# -----------------Main function-------------------------|

def go_target():

 global cxm
 global cym
 global pub

#Initialize node
 rospy.init_node('pick_place', anonymous=True)
 rospy.Subscriber("/arbotix/force", Analog, callback_force)
 rospy.Subscriber("/gpg/camera_info", CameraInfo, callback_camera)
 pub = rospy.Publisher("/gripper_joint/command", Float64, queue_size=5) 

 tfBuffer = tf2_ros.Buffer()
 listener = tf2_ros.TransformListener(tfBuffer)
 direction = PointStamped()
 direction.header.stamp = rospy.Time()
 direction.header.frame_id = "camera_link"

 robot = moveit_commander.RobotCommander()
 group = moveit_commander.MoveGroupCommander("arm")
 group.set_goal_tolerance(0.01)
 group.set_planner_id('RRTstarkConfigDefault')
 group.set_planning_time(0.5)
 
 r = rospy.Rate(1)  

 time.sleep(1)
 pub.publish(0)
 time.sleep(1)
 
 
 while not rospy.is_shutdown():
# Go to initial position
   go_to_pinitial(group)

#Wait for user input
   obj = raw_input("Write the object that you want to pick up or write 'quit' to exit: ")
   if obj == 'quit':
    break
   print obj
#Read center of mass
   f= open("/home/nvidia/catkin_ws/src/project/src/object.fcf","w")
   f.write(obj)
   f.close()
   f = open("data.fcf",'r')
   args = f.readline() 
   args = args.split(',')
   args = numpy.array(args, dtype=numpy.float64)
#Calculate object position
   cxm = int((args[1] + args[3])*0.5) 
   cym = int(args[2])
   position = None
   line = model.projectPixelTo3dRay((cxm,cym))
   
   if cxm == 0 and cym == 0:
    print "zero"
    pass

   direction.point.x = line[0]
   direction.point.y = line[1]
   direction.point.z = line[2]

   try:
    q = tfBuffer.transform(direction, "base_link")
    direction.point.x = 0
    direction.point.y = 0
    direction.point.z = 0
    p = tfBuffer.transform(direction, "base_link")
    
    position = compute_position(p,q)
    pose_target.position.x = position[0] + 0.01
    pose_target.position.y = position[1] 
    if obj == 'bottle':
     pose_target.position.z = position[2] + 0.065
    else:   
     pose_target.position.z = position[2] + 0.03
    if pose_target.position.z < -0.052:
      pose_target.position.z = -0.052

    angle = math.atan2(pose_target.position.y,pose_target.position.x)    
    quart = tf.transformations.quaternion_from_euler(0, 1.5707, angle)
    
    pose_target.orientation.x = quart[0]
    pose_target.orientation.y = quart[1]
    pose_target.orientation.z = quart[2]
    pose_target.orientation.w = quart[3]

   except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
     print e 
     r.sleep()

    
   
# Send the coordinates to Rviz
   broadcaster = tf2_ros.TransformBroadcaster()
   t = TransformStamped()
   t.header.stamp = rospy.Time.now()
   t.header.frame_id = "base_link"
   t.child_frame_id = "object"
   t.transform.translation.x = position[0] 
   t.transform.translation.y = position[1]
   t.transform.translation.z = position[2]
    
   t.transform.rotation.x = 0
   t.transform.rotation.y = 0
   t.transform.rotation.z = 0
   t.transform.rotation.w = 1
   broadcaster.sendTransform(t)

# Publish the object position    
   
   print "pose_target"
   print pose_target  
   group.set_pose_target(pose_target)
# Pick up the object and come back to initial position
   group.go(wait=True)
   time.sleep(1)
   get_object()
   go_to_pinitial(group)
   go_to_pfinal(group)
   pub.publish(0)
   time.sleep(0.5)
   go_to_pinitial(group)

# Main loop
if __name__ == '__main__':
 try:
  go_target()
 except rospy.ROSInterruptException: pass
