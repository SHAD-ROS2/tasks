from setuptools import setup

setup(name='rover_fixture', version='0.1.0', packages=['rover_fixture'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rover_fixture']),
                  ('share/rover_fixture', ['package.xml'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='ROS Course', maintainer_email='course@example.com',
      description='Deterministic L03 sensor stand', license='Apache-2.0',
      entry_points={'console_scripts': ['fixture = rover_fixture.node:main']})
