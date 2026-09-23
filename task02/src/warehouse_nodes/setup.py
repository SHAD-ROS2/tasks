from setuptools import find_packages, setup


package_name = "warehouse_nodes"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHAD ROS 2 course",
    maintainer_email="ros2-course@example.invalid",
    description="Six small ROS 2 nodes for the warehouse integration exercise.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "range_sensor = warehouse_nodes.range_sensor:main",
            "obstacle_filter = warehouse_nodes.obstacle_filter:main",
            "mission_planner = warehouse_nodes.mission_planner:main",
            "drive_controller = warehouse_nodes.drive_controller:main",
            "safety_monitor = warehouse_nodes.safety_monitor:main",
            "operator_panel = warehouse_nodes.operator_panel:main",
        ],
    },
)
