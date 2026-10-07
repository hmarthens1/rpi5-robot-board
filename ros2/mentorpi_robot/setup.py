from glob import glob
from setuptools import setup

package_name = "mentorpi_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hmarthens1",
    maintainer_email="hmarthens@gmail.com",
    description="Fleet interface for a Hiwonder MentorPi T1",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_node = mentorpi_robot.robot_node:main",
            "ms200_node = mentorpi_robot.ms200_node:main",
            "camera_bridge = mentorpi_robot.camera_bridge:main",
        ],
    },
)
