from moveit_configs_utils.launches import generate_move_group_launch

from arm_moveit_config_utils.launch_utils import build_moveit_config

def generate_launch_description():
    moveit_config = build_moveit_config()
    return generate_move_group_launch(moveit_config)
