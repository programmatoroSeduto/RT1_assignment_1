#!/usr/bin/env python

# Informations for services
tolerance = 0.1
twist_gain = 1.0

# Informations for the controller
input_topic = "base_pose_ground_truth"
output_topic = "cmd_vel"

# Names of the services
srv_name_check_target = "rg_check_target"
srv_name_get_target = "rg_get_target"
srv_name_get_velocity = "rg_get_vel"