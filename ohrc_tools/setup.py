from setuptools import find_packages, setup

package_name = 'ohrc_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/reset_keyboard_caller.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shunki',
    maintainer_email='s.itadera@aist.go.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reset_keyboard_caller = ohrc_tools.reset_keyboard_caller:main',
        ],
    },
)
