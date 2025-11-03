from setuptools import setup

package_name = 'virtual_gps'

setup(
    name=package_name,
    version='0.0.1',
    packages=['virtual_gps'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='talha',
    maintainer_email='bilikcitalha@gmail.com',
    description='Virtual GPS publisher and subscriber',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_publisher = virtual_gps.gps_publisher:main',
            'gps_subscriber = virtual_gps.gps_subscriber:main',
        ],
    },
)
