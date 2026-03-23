from setuptools import find_packages, setup

package_name = 'hex_cmd_crl'

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
    maintainer='hexfellow',
    maintainer_email='13769010+thetaigon@user.noreply.gitee.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_cmd_crl = hex_cmd_crl.joint_cmd_crl:main',
            'cmd_vel_crl = hex_cmd_crl.cmd_vel_crl:main'
        ],
    },
)
