from setuptools import setup

package_name = 'edge_detector'

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
    maintainer='Ashmit Rajaure',
    maintainer_email='ashmit.rajaure@gmail.com',
    description='A node that receives the raw image from turtlebot3 and processes it.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor = edge_detector.edge_detector:main'
        ],
    },
)
