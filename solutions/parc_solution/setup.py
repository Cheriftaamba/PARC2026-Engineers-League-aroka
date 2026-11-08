import os
from glob import glob

from setuptools import find_packages, setup

package_name = "parc_solution"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="keba",
    maintainer_email="kebafaye98@gmail.com",
    description="PARC Engineers League 2026 - Phase 1 navigation solution.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # The competition requires the executable to be named exactly
            # "task_solution.py" so that `ros2 run parc_solution task_solution.py`
            # works. Setuptools happily accepts a dot in the script name.
            "task_solution.py = parc_solution.task_solution:main",
            "scan_filter.py = parc_solution.scan_filter:main",
            "world_anchor.py = parc_solution.world_anchor:main",
            "evidence_map.py = parc_solution.evidence_map:main",
            "speed_governor.py = parc_solution.speed_governor:main",
        ],
    },
)
