import os
from glob import glob

from setuptools import find_packages, setup


package_name = "warehouse_bringup"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHAD ROS 2 course",
    maintainer_email="ros2-course@example.invalid",
    description="Launch files and parameters for the warehouse stack.",
    license="MIT",
    extras_require={"test": ["pytest"]},
)
