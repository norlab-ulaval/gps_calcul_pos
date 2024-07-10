from setuptools import find_packages, setup

package_name = 'gps_calcul_pos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/gps_compute.yaml']),
        ('share/' + package_name + '/launch', ['launch/gps_compute.launch.py']),
    ],
    install_requires=['setuptools', 'pyproj', 'numpy'],
    zip_safe=True,
    maintainer='dom',
    maintainer_email='nitrof2000@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_compute = gps_calcul_pos.gps_compute:main',
        ],
    },
)
