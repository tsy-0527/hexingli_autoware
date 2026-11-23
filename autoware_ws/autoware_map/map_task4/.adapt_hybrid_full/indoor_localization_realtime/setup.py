from glob import glob
from setuptools import setup

package_name = 'indoor_localization'
nodes = ['mid360_preprocessor', 'degeneracy_covariance_relay', 'initial_pose_bridge',
         'velocity_report_converter', 'localization_health_monitor']
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [f'{n} = {package_name}.{n}:main' for n in nodes]},
)
