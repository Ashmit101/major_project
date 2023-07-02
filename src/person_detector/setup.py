from setuptools import setup

package_name = 'person_detector'

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
    maintainer='Tribhuwan',
    maintainer_email='tribhuwan@gmail.com',
    description='Package to detect people in a video frame',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = person_detector.person_detector:main'
        ],
    },
)
