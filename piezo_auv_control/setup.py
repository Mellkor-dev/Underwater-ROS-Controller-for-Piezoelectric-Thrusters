from setuptools import find_packages, setup

package_name = 'piezo_auv_control'

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
    maintainer='soumyadeep',
    maintainer_email='chatterjee.soumyadeep15@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pja_controller = piezo_auv_control.pja_controller:main',
            'depth_hold_pid = piezo_auv_control.depth_hold_pid:main',
            'piezo_sim_node = piezo_auv_control.piezo_sim_node:main'
        ],
    },
)
