from setuptools import setup
import os
from glob import glob

package_name = "mpc_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py"))),
        (os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Navid",
    maintainer_email="navid@todo.todo",
    description="2WD Yahboom hardware bring-up for distributed MPC formation control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver_node = mpc_robot_bringup.driver_node:main",
            "calibrate_node = mpc_robot_bringup.calibrate_node:main",
        ],
    },
)
