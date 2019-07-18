#!/usr/bin/env python
#
# Last modification: 20 Dec. 2018
# Author: Rayanne Souza

import rospy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose

rospy.wait_for_service('/apply_planning_scene')
client = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)

def plane():

 wall_object = CollisionObject()
 wall_object.operation = wall_object.ADD
 wall_object.id = "wall_box"
 wall_object.header.frame_id = "base_link"

 wall_pose = Pose()
 wall_pose.orientation.w = 1
 wall_pose.position.z = 0.15
 wall_pose.position.x = -0.18
 wall_pose.position.y = 0

 wall = SolidPrimitive()
 wall.type = SolidPrimitive.BOX
 wall.dimensions = [0.01, 0.27, 0.3]

 wall_object.primitives = [wall]
 wall_object.primitive_poses.append(wall_pose)

# ---------------------------------- 
 wall2_object = CollisionObject()
 wall2_object.operation = wall_object.ADD
 wall2_object.id = "wall2_box"
 wall2_object.header.frame_id = "base_link"

 wall2_pose = Pose()
 wall2_pose.orientation.w = 1
 wall2_pose.position.z = 0.15
 wall2_pose.position.x = -0.05
 wall2_pose.position.y = -0.15

 wall2 = SolidPrimitive()
 wall2.type = SolidPrimitive.BOX
 wall2.dimensions = [0.24, 0.01, 0.3]

 wall2_object.primitives = [wall2]
 wall2_object.primitive_poses.append(wall2_pose)

# ---------------------------------- 
 wall3_object = CollisionObject()
 wall3_object.operation = wall_object.ADD
 wall3_object.id = "wall3_box"
 wall3_object.header.frame_id = "base_link"

 wall3_pose = Pose()
 wall3_pose.orientation.w = 1
 wall3_pose.position.z = 0.15
 wall3_pose.position.x = -0.05
 wall3_pose.position.y = 0.15

 wall3 = SolidPrimitive()
 wall3.type = SolidPrimitive.BOX
 wall3.dimensions = [0.24, 0.01, 0.3]

 wall3_object.primitives = [wall3]
 wall3_object.primitive_poses.append(wall3_pose)

# ---------------------------------- 


 base1_object = CollisionObject()
 base1_object.operation = base1_object.ADD
 base1_object.id = "base1_box"
 base1_object.header.frame_id = "base_link"

 base1_pose = Pose()
 base1_pose.orientation.w = 1
 base1_pose.position.y = 0.0
 base1_pose.position.x = -0.07
 base1_pose.position.z = 0.03

 base1 = SolidPrimitive()
 base1.type = SolidPrimitive.BOX
 base1.dimensions = [0.22, 0.27, 0.01]
 
 base1_object.primitives = [base1]
 base1_object.primitive_poses.append(base1_pose)

# ---------------------------------- 

 base2_object = CollisionObject()
 base2_object.operation = base1_object.ADD
 base2_object.id = "base2_box"
 base2_object.header.frame_id = "base_link"

 base2_pose = Pose()
 base2_pose.orientation.w = 1
 base2_pose.position.y = 0.0
 base2_pose.position.x = -0.07
 base2_pose.position.z = 0.35

 base2 = SolidPrimitive()
 base2.type = SolidPrimitive.BOX
 base2.dimensions = [0.20, 0.27, 0.01]
 
 base2_object.primitives = [base2]
 base2_object.primitive_poses.append(base2_pose)


 planning_scene = PlanningScene()
 planning_scene.is_diff = True
 planning_scene.world.collision_objects.append(wall_object)
 planning_scene.world.collision_objects.append(wall2_object)
 planning_scene.world.collision_objects.append(base1_object)
 planning_scene.world.collision_objects.append(base2_object)
 client(planning_scene)


if __name__ == '__main__':
 try:
  plane()
 except rospy.ROSInterruptException: pass
