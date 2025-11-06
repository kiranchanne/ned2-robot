import sympy as sp
from pyniryo import *
import numpy as np
import spatialmath.base as base




theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')




DH_params = [
  [0.0,      sp.pi/2,  0.183,  theta1],
  [0.221,       0.0,   0.0,   theta2+sp.pi/2],
  [0.0282,    sp.pi/2,   0.0, theta3],
  [0.0,   -sp.pi/2,   0.235,   theta4-sp.pi/2],
  [0.0,       sp.pi/2,   0.0, -theta5],
  [0.035,           0.0,   0.06, theta6+sp.pi/2]
]




# Function to compute Ai-1i
def DH_matrix(a, alpha, d, theta):
  return sp.Matrix([
      [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
      [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
      [0,              sp.sin(alpha),                sp.cos(alpha),               d],
      [0,              0,                            0,                           1]
  ])




# T0_6 symbolically (overall transformation)
T = sp.eye(4)
for a, alpha, d, theta in DH_params:
  T = T * DH_matrix(a, alpha, d, theta)




start_pos = [-0.35444782111674256, -1.0246265081988333, -0.2871126754419009, 0.2976849264396071, 1.1089754560515255, -0.20699475277476864]
stop_pos  = [0.34107286712000606, -0.6837637052843407, -0.5340488393310666, 0.2670053106818946, 0.9954608777479881, -0.17171319465339874]




subs_start = {theta1: start_pos[0], theta2: start_pos[1], theta3: start_pos[2],
            theta4: start_pos[3], theta5: start_pos[4], theta6: start_pos[5]}




subs_stop  = {theta1: stop_pos[0], theta2: stop_pos[1], theta3: stop_pos[2],
            theta4: stop_pos[3], theta5: stop_pos[4], theta6: stop_pos[5]}




T_start = np.array(T.evalf(subs=subs_start)).astype(np.float64)
T_stop  = np.array(T.evalf(subs=subs_stop)).astype(np.float64)




print("Start Position Transformation Matrix:")
sp.pprint(sp.Matrix(T_start))




print("\nStop Position Transformation Matrix:")
sp.pprint(sp.Matrix(T_stop))




start_position = np.concatenate((T_start[0:3,3], base.tr2rpy(T_start, unit='rad')))
stop_position = np.concatenate((T_stop[0:3,3], base.tr2rpy(T_stop, unit='rad')))


print("\nPick Pose (from FK):")
print("x,y,z,roll,pitch,yaw =", start_position)
print("\nPlace Pose (from FK):")
print("x,y,z,roll,pitch,yaw =", stop_position)




robot_ip_address, workspace_name = "ned3.ee.ca", "Ned Station"
def process(robot):
"""
USED TO ACHIEVE THETA VALUES MANUALLY
pick_joints = [-0.35444782111674256, -1.0246265081988333, -0.2871126754419009, 0.2976849264396071, 1.1089754560515255, -0.20699475277476864]
place_joints = [0.34107286712000606, -0.6837637052843407, -0.5340488393310666, 0.2670053106818946, 0.9954608777479881, -0.17171319465339874]




robot.move_to_home_pose() #returns to home
input("Move to start position")
start = robot.get_joints()
print(start)




input("Move to stop position")
stop = robot.get_joints()
print(stop)




#Pick & Place




#PICK
robot.open_gripper()
robot.move_joints(pick_joints)
robot.close_gripper()
measured_pick = robot.get_pose()
print("Measured pick pose:", measured_pick)




#PLACE
robot.move_joints(place_joints)
robot.open_gripper()
measured_place = robot.get_pose()
print("Measured place pose:", measured_place)




robot.move_to_home_pose()
"""




#Pick & Place
robot.open_gripper()
pose_pick = PoseObject(*start_position)
pose_place = PoseObject(*stop_position)
robot.move_pose(pose_pick)
robot.close_gripper()
robot.move_pose(pose_place)
robot.open_gripper()




robot.move_to_home_pose()




if __name__ == '__main__':
robot = NiryoRobot(robot_ip_address)
robot.update_tool()
robot.calibrate_auto()
process(robot)
robot.go_to_sleep()
robot.close_connection()


from pyniryo import *
import sympy as sp
import numpy as np
def DH_matrix(a, d, alpha, theta):
  return sp.Matrix([
      [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
      [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
      [0,              np.sin(alpha),                np.cos(alpha),               d],[0,              0,                            0,                         1]  ])


robot_ip_address, workspace_name = "ned3.ee.ca", "Ned Station"
def process(robot):
   a = np.array([0, 0.221, 0.0282, 0, 0, 0.035])
   alpha = np.array([np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, 0])
   d = np.array([0.183, 0, 0, 0.235, 0, 0.06])
   robot.move_to_home_pose()
   q = [-0.2, 0.3, 0.2, 0.3, -0.6, 0]
   for i in range(15):
       q[0] += 0.05
       robot.move_joints(q)
       print(robot.get_pose())
if __name__ == '__main__':
  robot = NiryoRobot(robot_ip_address)
  robot.update_tool()
  robot.calibrate_auto()
process(robot)
robot.go_to_sleep()
robot.close_connection()
