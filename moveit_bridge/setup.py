from setuptools import setup

package_name = "moveit_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MHC",
    maintainer_email="carvalho.matheush@gmail.com",
    description="Ponte HTTP entre a interface web do digital twin e o move_group.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = moveit_bridge.bridge_node:main",
        ],
    },
)
