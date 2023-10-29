from setuptools import setup

package_name = 'crazyflie_teleop'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/joystick.launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
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
