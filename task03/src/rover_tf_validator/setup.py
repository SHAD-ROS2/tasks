from setuptools import find_packages, setup

setup(
    name='rover_tf_validator', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/rover_tf_validator']),
                ('share/rover_tf_validator', ['package.xml'])],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Robotics course', maintainer_email='course@example.org',
    description='Timestamp-aware transform validator: L03 homework scaffold.', license='MIT',
    entry_points={'console_scripts': [
        'validator_node = rover_tf_validator.node:main',
        'acceptance = rover_tf_validator.acceptance:main',
        'practice_lookup = rover_tf_validator.practice_lookup:main',
        'practice_exercise = rover_tf_validator.practice_exercise:main',
    ]},
)
