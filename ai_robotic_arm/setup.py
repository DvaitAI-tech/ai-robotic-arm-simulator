from setuptools import find_packages, setup

package_name = 'ai_robotic_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nk',
    maintainer_email='mkumar7404508129@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_simulator = ai_robotic_arm.arm_simulator:main',
            'ai_controller = ai_robotic_arm.ai_controller:main',
            'arm_command_subscriber = ai_robotic_arm.arm_command_subscriber:main',
            'arm_status_publisher = ai_robotic_arm.arm_status_publisher:main',
            'arm_controller = ai_robotic_arm.arm_controller_node:main',
            'ai_autopilot = ai_robotic_arm.ai_autopilot:main',

        ],
    },
)
