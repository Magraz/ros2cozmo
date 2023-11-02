from setuptools import find_packages, setup

package_name = 'ros2cozmo'

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
    maintainer='magraz',
    maintainer_email='agrazvallejo@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringup = ros2cozmo.bringup:main',
            'face_detect = ros2cozmo.face_detect:main',
        ],
    },
)
