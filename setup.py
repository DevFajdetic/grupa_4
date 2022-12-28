from setuptools import setup

package_name = 'grupa_4'

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
    maintainer='grupa_4',
    maintainer_email='grupa_4@todo.todo',
    description='Paket za projekt iz mobilne robitike',
    license='Apache license 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
