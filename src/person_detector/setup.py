from setuptools import setup
import os

package_name = 'person_detector'

yolov3_dir = os.path.join(os.path.dirname(__file__), 'yolov3')
yolov3_files = []
for root, dirs, files in os.walk(yolov3_dir):
    for file in files:
        yolov3_files.append(os.path.join(root, file))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # package_data={package_name: ['yolov3/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/yolov3', yolov3_files)
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
