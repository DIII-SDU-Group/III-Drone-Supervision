import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'iii_drone_supervision'
description="Package containing the III system daemon, launch-driven runtime manager, and managed node wrappers."
maintainer="Frederik Falk Nyboe"
maintainer_email="ffn@sdu.dk"
license="proprietary"
version="2.2"

setup(
    name=package_name,
    version=version,
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'node_management_config'), glob('node_management_config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    entry_points={
        'console_scripts': [
            "managed_node_wrapper = iii_drone_supervision.managed_node_wrapper:main",
            "system_daemon = iii_drone_supervision.system_daemon:main",
        ],
    },
)
