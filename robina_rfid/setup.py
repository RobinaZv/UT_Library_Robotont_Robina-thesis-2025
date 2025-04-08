from setuptools import find_packages, setup

package_name = 'robina_rfid'

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
    maintainer='peko',
    maintainer_email='robina.zvirgzdina@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'rfid_reader = robina_rfid.rfid_reader:main',
        	'big_node = robina_rfid.big_node:main', 
        	'tag_location_processor = robina_rfid.tag_location_processor:main', 
        	'marker_publisher = robina_rfid.marker_publisher:main', 
        	'step_publisher = robina_rfid.step_publisher:main',
        	'height_changer_node = robina_rfid.height_changer_node:main',
		'point_follower = robina_rfid.point_follower:main',
        ],
    },
)
