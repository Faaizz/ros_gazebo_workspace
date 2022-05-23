from setuptools import setup

package_name = 'my_pkg'

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
    maintainer='faaizz',
    maintainer_email='fr33ziey@gmail.com',
    description='ROS 2 Turotials from docs.ros.org',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_pkg.my_node:main',
            'talker = my_pkg.publisher_member_function:main',
            'listener = my_pkg.subscriber_member_function:main',
            'add_service = my_pkg.service_member_function:main',
            'add_client = my_pkg.client_member_function:main',
        ],
    },
)