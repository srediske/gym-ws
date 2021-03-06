#import logging
from gym.envs.registration import register

#logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------
# Copter envs
register(
    id='Hover-v0',
    entry_point='gym_ros.envs.copter:CopterEnv',
)

register(
    id='new_Hover-v0',
    entry_point='gym_ros.envs.copter:new_CopterEnv',
)

register(
    id='dronekit_Hover-v0',
    entry_point='gym_ros.envs.copter:dronekit_CopterEnv',
)
