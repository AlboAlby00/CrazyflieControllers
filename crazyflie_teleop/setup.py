from setuptools import setup

package_name = 'crazyflie_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alboalby00',
    maintainer_email='ge87cuf@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_attitude = crazyflie_teleop.cmd_vel_to_attitude:main',
            'joystick_to_attitude = crazyflie_teleop.joystick_to_attitude:main'
        ],
    },
)
