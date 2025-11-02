from setuptools import find_packages, setup

package_name = "waypoint_tasking"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="varsenii",
    maintainer_email="varsenyi@gmail.com",
    description="Waypoint tasking",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_tasking = waypoint_tasking.waypoint_tasking:main",
            "twist_converter = waypoint_tasking.navigation.twist_converter:main",
            "scan_tf_debuger = waypoint_tasking.scan_tf_debugger:main",
            "tfs_stamp_checker = waypoint_tasking.tfs_stamp_checker:main",
            "tf_hz_checker = waypoint_tasking.tf_hz_checker:main",
            "pose_controller = waypoint_tasking.navigation.pose_controller:main"
        ],
    },
)
