from setuptools import setup
from glob import glob
setup(name='rover_sim_lab', version='0.1.0', packages=['rover_sim_lab'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rover_sim_lab']),
                  ('share/rover_sim_lab', ['package.xml']),
                  ('share/rover_sim_lab/launch', glob('launch/*.py')),
                  ('share/rover_sim_lab/rviz', glob('rviz/*')),
                  ('share/rover_sim_lab/practice', glob('../../practice/*.yaml')),
                  ('share/rover_sim_lab/homework', glob('../../homework/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='ROS 2 course', maintainer_email='course@example.com',
      description='Gazebo Harmonic simulation laboratory', license='Apache-2.0',
      entry_points={'console_scripts': ['lab = rover_sim_lab.cli:main']})
