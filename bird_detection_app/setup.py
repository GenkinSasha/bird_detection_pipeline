from setuptools import setup

package_name = 'bird_detection_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='AlexG',
    maintainer_email='genkin.sasha@gmail.com',
    description='Bird detection application using DeepStream and ROS 2',
    license='MIT',
    extras_require={
        'dev': ['pytest'],  # Use this for test-related dependencies
    },
    entry_points={
        'console_scripts': [
            'bird_publisher = ros_interface.bird_publisher:main',
        ],
    },
)
