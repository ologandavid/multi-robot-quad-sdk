#! /usr/bin/env python


from quad_pybullet.pybullet_master_node import pybullet_sim_node


node_name = "pybullet_sim"
robot_file = "/home/haoluo/catkin_ws_opt/src/quad-sdk/quad_simulator/spirit_description/urdf/spirit.urdf"
load_sdf = False
# robot_file = "/home/haoluo/catkin_ws_opt/src/quad-sdk/quad_simulator/spirit_description/sdf_mesh/spirit.sdf"
# load_sdf = True

step_rate = 500
pybullet_pub_name = "/robot_1/state/ground_truth_pybullet"
# pybullet_pub_name = "/robot_1/state/ground_truth"
clock_pub_name = "/double_clock"
# world_urdf = "/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/gazebo_scripts/worlds/flat/flat.world"
# world_urdf = "/home/haoluo/catkin_ws_opt/src/quad-sdk/quad_pybullet/plane.urdf"
world_urdf = "/home/haoluo/catkin_ws_opt/src/quad-sdk/quad_pybullet/plane.urdf"
# pybullet_sub_name = "/robot_1/control/joint_commands_pybullet"
pybullet_sub_name = "/robot_1/control/joint_command"

if __name__ == '__main__':
    new_node = pybullet_sim_node(node_name,step_rate,robot_file,world_urdf=world_urdf, \
        pub_name = pybullet_pub_name,sub_name = pybullet_sub_name,clock_topic = clock_pub_name\
        , load_sdf=load_sdf)
    # sensor_node = new_node.sensor_node
    new_node.run()
