from setuptools import find_packages, setup

package_name = 'multi_net_bot'

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
    maintainer='indra',
    maintainer_email='indra@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_manager = multi_net_bot.swarm_manager:main',
            'robot_node = my_swarm_package.robot_node:main',
        ],
    },
)

