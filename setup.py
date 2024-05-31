from setuptools import find_packages, setup
import os

package_name = 'iii_drone_supervision'
description="Package containing the III-Drone-Supervisor node responsible for system bringup and node management through the ROS2 lifecycle API, as well as the ManagedNodeWrapper generic ROS2 node for wrapping any process as a managed node."
maintainer="Frederik Falk Nyboe"
maintainer_email="ffn@sdu.dk"
license="proprietary"
version="2.2"

setup(
    name=package_name,
    version=version,
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    entry_points={
        'console_scripts': [
            "managed_node_wrapper = iii_drone_supervision.managed_node_wrapper:main",
            "supervisor = iii_drone_supervision.supervisor_node:main"
        ],
    },
)
