from setuptools import find_packages, setup

PACKAGE_NAME = "vla_task_coordinator"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RRBTW",
    maintainer_email="mainacc1228@gmail.com",
    description="Python lifecycle coordinator for VLA / scripted pick-and-place TCP commands.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vla_task_coordinator = vla_task_coordinator.coordinator_node:main",
        ],
    },
)
