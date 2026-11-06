from setuptools import find_packages, setup
from glob import glob
import os

PACKAGE = "rcauto"
setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
        (os.path.join("share", PACKAGE, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RCauto Research",
    maintainer_email="rcauto@example.invalid",
    description="Fail-safe ROS 2 autonomous RC car stack",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "vision = rcauto.vision_node:main",
        "autonomy = rcauto.autonomy_node:main",
        "actuator = rcauto.actuator_node:main",
        "hailo_bridge = rcauto.hailo_bridge_node:main",
        "simulator = rcauto.simulator_node:main",
    ]},
)
