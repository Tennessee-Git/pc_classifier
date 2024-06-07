from setuptools import find_packages, setup

package_name = 'pc_c'

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
    maintainer='p-awr',
    maintainer_email='p-awr@gmail.com',
    description='Point cloud classifier',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display3d = pc_c.Display3DNode:main',
            'display2d = pc_c.Display2DNode:main',
            'classifier = pc_c.ClassifierNode:main',
            'lidar = pc_c.LidarReaderNode:main'
        ],
    },
)
