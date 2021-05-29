from setuptools import setup

package_name = 'turtlesim_udemy_project'

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
    maintainer='emanuele',
    maintainer_email='emanuele@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = turtlesim_udemy_project.turtle_controller:main',
            'turtle_spawner = turtlesim_udemy_project.turtle_spawner:main'
        ],
    },
)
