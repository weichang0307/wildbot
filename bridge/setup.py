from setuptools import find_packages, setup

package_name = 'bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bridge.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name, ['wildbot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='weichang0307@gmail.com',
    description='Bridge crossing node',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'run = nodes.main:main',
            'bridge_node = nodes.bridge_node:main',
        ],
    },
)
