from setuptools import find_packages, setup

package_name = 'cv_franka_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/integrate.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Graham Clifford',
    maintainer_email='gclifford@u.northwestern.edu',
    description='TODO: Package description',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_franka_bridge = cv_franka_bridge.cv_franka_bridge:main'
        ],
    },
)
