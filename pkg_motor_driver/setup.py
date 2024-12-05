from setuptools import find_packages, setup

package_name = 'pkg_motor_driver'

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
    maintainer='dafodilrat',
    maintainer_email='dafodilrat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = pkg_motor_driver.imu_publisher:main' 
            'robot_pid = pkg_motor_driver.PID_motor:main'   
            # motor control is the script name pkg_motor_driver is the pkg nanme
            # main is the main function that calls all the ros2 nodes.
        ],
    },
)
