# What the spine writes to its actuator drivers each tick: the throttle DAC, the brake
# Jrk G2, the reverse relay and the steering Jrk G2 (spine-core's Outputs).
#
# Only in simulation: on the car these go over I2C and wires, never to ROS. Published by
# spine_sim on /vehicle/sim/spine_outputs at 100 Hz in Gazebo's spine profile
# (sim.launch.py spine:=true), where sim_adapter's actuator model turns them into what
# Gazebo's car does.

std_msgs/Header header
float32 throttle     # 0..1 of full throttle
float32 brake        # 0..1 of full brake
bool reverse         # the drive direction: true = backwards
float32 steering     # rad, the steering target (the virtual centre wheel), + left
