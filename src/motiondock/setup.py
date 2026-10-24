from setuptools import setup

package_name = "motiondock"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="MotionDock maintainers",
    maintainer_email="maintainer@example.com",
    description="Single-robot MotionDock control desktop application",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "motiondock = motiondock.app:main",
        "motiondock_supervisor = motiondock.supervisor:main",
    ]},
)
