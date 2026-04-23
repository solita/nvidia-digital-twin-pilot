from setuptools import find_packages, setup

package_name = "vehicle_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "vehicle_controller/config/nav2_params.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "vehicle_controller/launch/multi_forklift.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Forklift vehicle controller",
    license="MIT",
    entry_points={
        "console_scripts": [
            "forklift_controller_node = vehicle_controller.forklift_controller_node:main",
        ],
    },
)
