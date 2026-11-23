from glob import glob
from setuptools import setup

package_name = 'indoor_navigation'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'ackermann_cmd_converter = indoor_navigation.ackermann_cmd_converter:main',
        'route_navigator = indoor_navigation.route_navigator:main',
        'hybrid_mode_manager = indoor_navigation.hybrid_mode_manager:main',
    ]},
)
