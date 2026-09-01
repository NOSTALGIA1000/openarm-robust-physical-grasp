from setuptools import setup

package_name = "openarm_grasp_physics"

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
    maintainer="nostalgia",
    maintainer_email="3096954761qq@gmail.com",
    description="Physics-mode grasp state machine and evaluation for OpenArm.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "physics_picker = openarm_grasp_physics.physics_picker:main",
        ],
    },
)
