from setuptools import setup, find_packages
import sys
import os.path

# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym_ws'))

setup(name='gym_ws',
      version='0.0.1',
      packages=find_packages(),
      install_requires=['gym>=0.2.3'],
      description='The OpenAI Gym for robotics: A toolkit for developing and'
                  ' compare your reinforcement learning agents in simulated environments with Gazebo/ROS.',
      url='',
      author='',
      package_data={'gym_ws': ['envs/param/*.param']},
      )
