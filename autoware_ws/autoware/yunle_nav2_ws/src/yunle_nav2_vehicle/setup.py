from glob import glob
from setuptools import find_packages, setup


package_name = "yunle_nav2_vehicle"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="tang",
    maintainer_email="tang@example.com",
    description="Nav2 command and odometry adapters for the Yunle vehicle.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "twist_to_control = yunle_nav2_vehicle.twist_to_control:main",
            "vehicle_odometry = yunle_nav2_vehicle.vehicle_odometry:main",
            "map_odom_lock = yunle_nav2_vehicle.map_odom_lock:main",
        ],
    },
)
