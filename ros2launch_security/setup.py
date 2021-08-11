from setuptools import find_packages
from setuptools import setup

package_name = 'ros2launch_security'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2launch', 'launch_ros', 'setuptools'],
    zip_safe=True,
    author='Joe Example',
    author_email='joe@example.com',
    maintainer='Joe Example',
    maintainer_email='joe@example.com',
    url='https://github.com/ros2/launch/tree/master/ros2launch_security',
    download_url='https://github.com/ros2/ros2launch_security/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The security extensions for ros2 launch',
    long_description=(
        'The package provides an extension adding security '
        'capabilities to the ROS 2 launch command line tool.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2launch.option': [
            'security = ros2launch_security.option.security:SecurityOption',
        ],
        'launch_ros.node_action': [
            'security = ros2launch_security.node_action.security:SecurityNodeActionExtension',
        ],
    }
)
