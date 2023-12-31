from setuptools import setup

package_name = 'rl_methods'

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
    maintainer='ollie',
    maintainer_email='ollie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colour_detection_node = rl_methods.colour_detection_node:main',
            'action_publisher = rl_methods.action_publisher:main',
            'lidar_processor_node = rl_methods.lidar_processor_node:main',
            'chaser_basic_navigation_node = rl_methods.chaser_basic_navigation_node:main',
        ],
    },
)
