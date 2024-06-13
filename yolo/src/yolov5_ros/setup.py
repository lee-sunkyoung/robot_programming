from setuptools import setup

package_name = 'yolov5_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'torch'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOv5 with ROS2 for real-time object detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_node = yolov5_ros.yolov5_node:main',
            'yolov5_subscriber = yolov5_ros.yolov5_subscriber:main'
        ],
    },
)




