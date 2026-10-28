from glob import glob
from setuptools import find_packages, setup

package_name = "inspection_cart"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/inspection_cart"]),
        ("share/inspection_cart", ["package.xml"]),
        ("share/inspection_cart/launch", glob("launch/*.launch.py")),
        ("share/inspection_cart/urdf", glob("urdf/*.urdf")),
        ("share/inspection_cart/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS course",
    maintainer_email="course@example.invalid",
    description="L06 visual model and reproducible axis incident",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "synthetic_states = inspection_cart.synthetic_states:main",
            "provenance = inspection_cart.provenance:main",
        ]
    },
)
