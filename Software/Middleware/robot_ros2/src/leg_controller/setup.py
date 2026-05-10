from setuptools import find_packages, setup

package_name = 'leg_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gazebo_ros', 'gazebo_plugins', 'simulation', 'controller_manager'],
    zip_safe=True,
    maintainer='lvdaengineer',
    maintainer_email='longdaicangu0002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [   
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node_leg_controller = leg_controller.nodeLegController:main',
            'node_web_gui = leg_controller.nodeWebGui:main',
        ],
    },
)
