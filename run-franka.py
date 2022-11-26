#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand, RobotState
import numpy as np
from copy import deepcopy

vals = []
vels = []
joint_positions = []
joint_velocites = []
vel_rate = 1 #velocity rate, 1, 2, 4
names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
write_lmt = 1 #0 for single letter, 1 for LMT
#neutral_pose = [-0.017792060227770554, -0.7601235411041661, 0.019782607023391807, -2.342050140544315, 0.029840531355804868, 1.5411935298621688, 0.7534486589746342]

def callback(msg):
    
    global vals , vels
    temp_vals = []
    temp_vels = []
    for n in names:
        idx = msg.name.index(n)
        temp_vals.append(msg.position[idx])
        temp_vels.append(msg.velocity[idx])

    vals = deepcopy(temp_vals)
    vels = deepcopy(temp_vels)
    


def read_joint_position():
    """
        read the txt file
    """
    #rospy.init_node("test_node")
    #rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/joint_states', JointState, callback)
    data = np.loadtxt("/home/student6/Downloads/franka_joint_state_hLMT",str)
    #print(vals)
    
    for i in range(len(data)):
        data0 = data[i].split(",")
        for j in range(len(data0)):
            data0[j] = float(data0[j])
            if (j == 3 or j == 0 or j == 2 or j == 4 or j == 5):
               data0[j] = -data0[j]
            #if (j == 5):
               #data0[j] = -data0[j]
            data0[j] = math.radians(data0[j])
        joint_positions.append(data0) 
    #print(joint_positions)


        


if __name__ == '__main__':
    
    read_joint_position()

   
    rospy.init_node("test_node")

    rospy.wait_for_service('/controller_manager/list_controllers')

    rospy.loginfo("Starting node...")
    rospy.sleep(3)

    pub = rospy.Publisher('/franka_ros_interface/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)

    # Subscribe to robot joint state
    rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/joint_states', JointState, callback)

    # Subscribe to robot state (Refer JointState.msg to find all available data. 
    # Note: All msg fields are not populated when using the simulated environment)
    #rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/robot_state', RobotState, state_callback)
    

    rate = rospy.Rate(1000)


    while not rospy.is_shutdown() and len(vals) != 7:
        continue

    rospy.loginfo("Sending robot to starting pose")

    rospy.sleep(2.0)

    #print(joint_positions[0])
    #print(vals)
    #for q in range(len(vals)):
     # joint_velocites.append((joint_positions[0][q]-vals[q])*2)

    #print(joint_velocites)

    # Create JointCommand message to publish commands
    pubmsg = JointCommand()
    pubmsg.names = names # names of joints (has to be 7 and in the same order as the command fields (positions, velocities, efforts))
    iteration = 0
    position_idx = 0
    start_flag = 0
    nrate = 1000
    rise_arm = 0
    while not rospy.is_shutdown():
      if (position_idx == len(joint_positions)):
        if (iteration > 10):
          print("finish")
        iteration = 1
        pubmsg.velocity = [0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001]
      if (write_lmt == 1):
        if (position_idx == 67 or position_idx == 182):
          if (rise_arm == 0):
            start_flag = 2
      if (start_flag == 1):
        nrate = 1000 / vel_rate
      else:
        nrate = 1000
      
      if (iteration % nrate == 0):
        if (start_flag == 0):
          print("Going to starting point...",position_idx+1)
          tt = 0
          for q in range(len(vals)):
            v = ((joint_positions[position_idx][q]-vals[q])*0.5)
            if (abs(v) < 0.001):
              tt = tt + 1
            joint_velocites.append(v)
          if (tt == 7):
            start_flag = 1
            position_idx = position_idx + 1
            rise_arm = 0
            pubmsg.velocity = [0.000001,0.000001,0.000001,0.000001,0.000001,0.000001,0.000001]
            joint_velocites = []
            
          else:
            pubmsg.velocity = joint_velocites
            joint_velocites = []
        elif (start_flag == 2):
          pubmsg.velocity = [0.000001,0.000001,0.000001,0.1,0.000001,0.000001,0.000001]
          start_flag = 0
          rise_arm = 1
        else:
          print("Drawing letter...",position_idx+1)
          for q in range(len(vals)):
            joint_velocites.append((joint_positions[position_idx][q]-vals[q])*vel_rate)
          #pubmsg.velocity = [0.000001,0.000001,0.000001,0.000001,0.00001,0.000001,0.000001]
          pubmsg.velocity = joint_velocites
          position_idx = position_idx + 1
          joint_velocites = []

        # JointCommand msg has other fields (velocities, efforts) for
                               # when controlling in other control mode
        # pubmsg.effort = [0.,0.,0.,0.,0.,0.,0.]
      pubmsg.mode = pubmsg.VELOCITY_MODE # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)
      #print(vals)
      pub.publish(pubmsg)
      iteration = iteration + 1

      rate.sleep()
      
      
        