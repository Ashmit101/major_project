from setuptools import setup

package_name = 'video_publisher'

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
    maintainer='Ashmit',
    maintainer_email='ashmit.rajaure@gmail.com',
    description='Video frame publisher from a video file',
    license='Apache License 2,0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = video_publisher.video_publisher:main'
        ],
    },
)
