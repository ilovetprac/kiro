from setuptools import find_packages, setup
import os
from glob import glob

package_name = "d435_marker_xy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ilovetprac",
    maintainer_email="ilovetprac@users.noreply.github.com",
    description="D435 depth+IR ArUco marker X,Y (mm) publisher. Marker center = origin.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "marker_xy_node = d435_marker_xy.marker_xy_node:main",
        ],
    },
)
