from setuptools import setup

package_name = 'ros_ocr'

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
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS2 node for OCR and classification using EasyOCR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ocr_classify_node = ros_ocr.ocr_classify_node:main',
            'image_publisher = ros_ocr.image_publisher:main',
        ],
    },
)
