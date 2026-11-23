from glob import glob
from setuptools import setup

package_name = 'fj_experiment'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fault_injector = fj_experiment.fault_injector:main',
            'existence_filter = fj_experiment.existence_filter:main',
            'runner = fj_experiment.runner:main',
            'calibrate = fj_experiment.calibrate:main',
            'evaluate = fj_experiment.evaluate:main',
        ],
    },
)
