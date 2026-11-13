from setuptools import find_packages, setup

package_name = 'cnc_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timo',
    maintainer_email='timo.schaffner@stud.hslu.ch',
    description='Manager Node for Talking to a CNC Machine over ROS',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manager_node = cnc_manager.manager_node:main'
        ],
    },
)
