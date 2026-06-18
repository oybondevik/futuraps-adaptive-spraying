from setuptools import setup
import os
from glob import glob

package_name = "futuraps_platform_pid"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO",
    description="PID controller for platform plant-following.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pid_controller = futuraps_platform_pid.pid_controller_node:main",
            "plot_pid_debug = futuraps_platform_pid.plot_pid_debug:main",
        ],
    },
)
