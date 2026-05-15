from setuptools import find_packages, setup

package_name = 'autonomous_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/launch', ['launch/drive.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dchavan2192',
    maintainer_email='dchavan2192@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_node = autonomous_car.serial_node:main',
            'teleop_node = autonomous_car.teleop_node:main',
            'camera_node = autonomous_car.camera_node:main',
            'servo_node = autonomous_car.servo_node:main',
            'safety_node = autonomous_car.safety_node:main',
        ],
    },
)
