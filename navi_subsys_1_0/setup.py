from setuptools import setup

package_name = 'navi_subsys_1_0'

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
    maintainer='jonas mahler',
    maintainer_email='mahler@dst-org.de',
    description='Receive and preprocess environment and internal sensor data',
    license=' GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'write_string= navi_subsys_1_0.write_string:main',
            'get_string = navi_subsys_1_0.get_string:main',
            'get_image = navi_subsys_1_0.get_image:main',
            'write_lidar_static = navi_subsys_1_0.write_lidar_static:main',
            'get_lidar = navi_subsys_1_0.get_lidar:main',
            'get_imu = navi_subsys_1_0.get_imu:main',
            'get_radar = navi_subsys_1_0.get_radar:main',
        ],
    },
)
