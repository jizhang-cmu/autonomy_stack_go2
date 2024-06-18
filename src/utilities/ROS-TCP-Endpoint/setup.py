import os

from setuptools import setup

package_name = "ros_tcp_endpoint"
share_dir = os.path.join("share", package_name)

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share_dir, ["package.xml"]),
        (os.path.join(share_dir, "launch"), ["launch/endpoint.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Unity Robotics",
    maintainer_email="unity-robotics@unity3d.com",
    description="ROS TCP Endpoint Unity Integration (ROS2 version)",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "default_server_endpoint = ros_tcp_endpoint.default_server_endpoint:main"
        ]
    },
)
