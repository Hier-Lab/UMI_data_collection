from setuptools import find_packages, setup

package_name = "rgbd_odometry"

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
    description="Post-process a recorded umi_recorder episode bag: compute per-camera RGBD "
    "visual odometry with Open3D and write it back into the bag as a visual_odom topic",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "compute_odometry = rgbd_odometry.compute_odometry:main",
        ],
    },
)
