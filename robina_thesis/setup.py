from setuptools import setup

package_name = 'robina_thesis'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Include the main package directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robina Zvirgzdina',
    maintainer_email='your_email@example.com',  # Replace with your email
    description='Robina Theses ROS2 package with laser filter node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_filter_node = robina_thesis.laser_filter_node:main',  # Fix path to the node
        ],
    },
)
