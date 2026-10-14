from setuptools import find_packages, setup

package_name = "exo_pose_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aho3",
    maintainer_email="aho3@ncsu.edu",
    description="MediaPipe Pose on an exocentric camera's live feed for human torso tracking",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_node = exo_pose_bridge.pose_node:main",
        ],
    },
)
