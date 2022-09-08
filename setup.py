from setuptools import setup

package_name = 'ft_rig'

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
    maintainer='ft',
    maintainer_email='uzoukwuc@tcd.ie',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_rig_node = ft_rig.ft_rig_node:main',
            'command_motors = ft_rig.command_motors',
            'print_pinion_distance = ft_rig.print_pinion_distance'
        ],
    },
)
