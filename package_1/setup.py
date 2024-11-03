from setuptools import find_packages, setup

package_name = 'package_1'

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
    maintainer='Danilo Giacomin Schneider',
    maintainer_email='danilo_gsch@hotmail.com',
    description='Este pacote publica mensagens a cada 1 (um) segundo com informações sobre a quantidade total de memória, o uso de memória RAM em Gigabyte e o percentual do uso.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'memory_monitor_publisher = package_1.memory_monitor_publisher:main',
        ],
    },
)
