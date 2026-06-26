from glob import glob
from setuptools import setup

package_name = "indoory_xlerobot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyzmq", "msgpack"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@localhost",
    description="ROS 2 bridge for indoory_isaac_sim ZMQ clients.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "isaac_sim_zmq_bridge = indoory_xlerobot_bridge.isaac_sim_zmq_bridge:main",
        ],
    },
)
