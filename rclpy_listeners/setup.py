from setuptools import setup

package_name = 'rclpy_listeners'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        'instrumentation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Henkel',
    maintainer_email='christian.henkel2@de.bosch.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'measure = instrumentation.measure:main'
        ],
    },
)
