from setuptools import find_packages, setup

package_name = "orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenMind",
    maintainer_email="hello@openmind.org",
    description="Orchestrator node for OM",
    license="MIT",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/orchestrator"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/orchestrator_launch.py"]),
    ],
    entry_points={
        "console_scripts": [
            "orchestrator_api = orchestrator.core.orchestrator_api:main",
            "orchestrator_cloud = orchestrator.core.orchestrator_cloud:main",
        ],
    },
)
