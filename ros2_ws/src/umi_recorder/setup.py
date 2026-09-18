from setuptools import find_packages, setup

package_name = "umi_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/umi_recorder.launch.py"]),
        ("share/" + package_name + "/templates", ["templates/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aho3",
    maintainer_email="aho3@ncsu.edu",
    description="Web UI to record synchronized UMI gripper demonstrations into a ROS 2 bag",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "recorder_node = umi_recorder.recorder_node:main",
        ],
    },
)
