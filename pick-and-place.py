robot = NiryoRobot("ned3.ee.ca")
robot.update_tool()
robot.calibrate_auto()
robot.close_gripper()


home = [[0.258, 0.00903, 0.966, 0.1242],
           [-0.00258, 0.999, -0.00866, 0],
           [-0.966, -0.000258, 0.258, 0.1481],
           [0, 0, 0, 1]]
home_matrix = np.array(home)
pose_home = to_pose(home_matrix)


pos_2 = [[0.0329, -0.1017, 0.9943, 0.3719],
            [0.0592, 0.9933, 0.09968, -0.0958],
            [-0.998, 0.0556, 0.03869, 0.0858],
            [0, 0, 0, 1]]
pos_2_matrix=np.array(pos_2)


pos_1 = [[0.0480, 0.289, 0.956, 0.3752],
    [0.119, 0.949, -0.292, 0.0824],
    [-0.992, 0.128, 0.011, 0.0901],
    [0, 0, 0, 1]]
pos_1_matrix = np.array(pos_1)


pos_4 = [[-0.0338, 0.2623, 0.9643, 0.2068],
    [0.0947, 0.9614, -0.258, 0.088],
    [-0.9949, 0.0826, -0.0574, 0.0898],
    [0, 0, 0, 1]]
pos_4_matrix = np.array(pos_4)


pos_3 = [[-0.0633, -0.2090, 0.9759, 0.2031],
     [0.0589, 0.9749, 0.2130, -0.1057],
     [-0.9959, 0.0769, -0.0481, 0.0914],
     [0, 0, 0, 1]]
pos_3_matrix = np.array(pos_3)


# from home to position 2
pos2 = to_pose(pos_2_matrix)
# from position 2 to 1
pos1 = to_pose(pos_1_matrix)
#from position 1 to 4
pos4 = to_pose(pos_4_matrix)
# from position 4 to 3
pos3 = to_pose(pos_3_matrix)


# Normal path
robot.move_pose(pose_home)
robot.move_pose(pos2)
final_pos = robot.get_pose()
print('Pos 2:', final_pos)


robot.move_pose(pos1)
final_pos = robot.get_pose()
print('Pos 1:', final_pos)


robot.move_pose(pos4)
final_pos = robot.get_pose()
print('Pos 4:', final_pos)


robot.move_pose(pos3)
final_pos = robot.get_pose()
print('Pos 3:', final_pos)


Rx90 = Rx_h(90)
pos4_roll90 = pos_4_matrix @ Rx90
pos3_roll90 = pos_3_matrix @ Rx90
rot_pos4 = to_pose(pos4_roll90)
rot_pos3 = to_pose(pos3_roll90)


robot.move_pose(pose_home)
robot.move_pose(pos2)
final_pos = robot.get_pose()
print('Pos 2:', final_pos)


robot.move_pose(pos1)
final_pos = robot.get_pose()
print('Pos 1:', final_pos)


robot.move_pose(pos4)
final_pos = robot.get_pose()
print('Pos 4:', final_pos)


robot.move_pose(rot_pos4)
final_pos = robot.get_pose()
print('Rotated Pos 4:', final_pos)


robot.move_pose(rot_pos3)
final_pos = robot.get_pose()
print('Rotated Pos 3:', final_pos)


robot.close_connection()



robot_ip_address, workspace_name = "ned3.ee.ca", "Ned Station"


def process(robot):

  pose = [0.1312,0.2084,0.1011,-2.602, 1.435, -0.801]


  robot.move_pose(pose)
  final_pos = robot.get_pose()
  print('Pick up:', final_pos)


  p = robot.get_pose()
  robot.open_gripper()
  robot.close_gripper()


  p2 = [0.03 , -0.03, -0.25]


  anglex = 90


  tr2 = tr_maker(angle=anglex, position=p2, units='deg', axis='x')


  tr3 = tr_maker(angle= -90, units='deg', axis='y') # rotate gripper


  tr = tr2 @ tr3


  org = p2tr(p)


  finalpose1 = org @ tr


  finalpose = tr2p(finalpose1)


  print(finalpose)


  robot.move_pose(finalpose)
  final_pos = robot.get_pose()
  print('Drop off:', final_pos)


  robot.release_with_tool()


  # reverse
  robot.close_gripper()
  p_start_inv = robot.get_pose()
  final_pos = robot.get_pose()
  print('Pick up:', final_pos)


  tr_inve = np.linalg.inv(tr)
  back = finalpose1 @ tr_inve
  end = tr2p(back)
  robot.move_pose(end)
  final_pos = robot.get_pose()
  print('Pick up:', final_pos)
  robot.release_with_tool()

def p2tr(pose):


  pose_list = pose.to_list()  # to_list() returns a list [x, y, z, roll, pitch, yaw] corresponding to the pose's parameters


  p = pose_list[0:3]


  rpy = pose_list[3:6]


  pose_r = base.rpy2tr(*rpy)


  pose_p = base.transl(p)


  pose_tr = pose_p @ pose_r


  return pose_tr




def tr2p(tr):


  send_p = tr[0:3, 3]


  send_r = base.tr2rpy(tr)


  packet = [*send_p, *send_r]


  pose = PoseObject(*packet)


  return pose




def tr_maker(angle=None, position=None, units='rad', axis=None):


  if position is None:
      position = [0, 0, 0]


  if units != 'rad' and units != 'deg':
      raise ValueError("Units must be 'rad' or 'deg'")


  tr_p = base.transl(*position)


  if axis is None and angle is not None:


      raise ValueError("Axis argument is 'None'")


  elif angle is None:


      tr_r = spatialmath.SE3._identity()


  elif axis == 'x':


      tr_r = base.trotx(angle, unit=units)


  elif axis == 'y':


      tr_r = base.troty(angle, unit=units)


  elif axis == 'z':


      tr_r = base.trotz(angle, unit=units)


  else:


      raise ValueError("Axis must be 'x', 'y', or 'z'")


  tr = tr_p @ tr_r


  return tr

if __name__ == '__main__':
  # Connect to robot

  robot = NiryoRobot(robot_ip_address)

  # Changing tool

  robot.update_tool()

  # Calibrate robot if robot needs calibration
  robot.calibrate_auto()
  robot.release_with_tool()
  robot.move_to_home_pose()
  # Launching main process
  process(robot)
  # print(robot.get_pose())
  # Ending

  robot.go_to_sleep()
  #Releasing connection
  robot.close_connection()