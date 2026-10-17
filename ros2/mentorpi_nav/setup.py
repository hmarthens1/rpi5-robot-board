from glob import glob
from setuptools import setup

package_name = "mentorpi_nav"

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
    description="Autonomous exploration for robot04 (Hector SLAM + Nav2 + frontier explorer)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "explorer = mentorpi_nav.explorer:main",
            "cmd_vel_adapter = mentorpi_nav.cmd_vel_adapter:main",
        ],
    },
)
