from glob import glob

from setuptools import find_packages, setup

package_name = 'rail_control_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config / launch 를 share 로 복사해야 설치 후에도 읽힙니다.
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='남현지',
    maintainer_email='rokey@example.com',
    description='의료 레일 관제 디지털 트윈 — 관제 코어',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_core = rail_control_core.control_core_node:main',
            'sim = rail_control_core.engine:main',
        ],
    },
)
