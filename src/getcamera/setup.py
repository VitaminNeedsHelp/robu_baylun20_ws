from setuptools import find_packages, setup

package_name = 'getcamera'

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
    maintainer='Baylun20',
    maintainer_email='baylun20@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_srv = getcamera.cameraServiceNode:main',
            'camera_clt = getcamera.cameraClientNode:main'
        ],
    },
)
