import os
from glob import glob

from setuptools import find_packages, setup

package_name = "silo"

setup(
  name=package_name,
  version="0.0.0",
  packages=find_packages(exclude=["test"]),
  data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (
      os.path.join("share", package_name, "launch"),
      glob(os.path.join("launch", "*launch.[pxy][yma]*")),
    ),
    (
      os.path.join("share", package_name, "config"),
      glob(os.path.join("config", "*.yaml")),
    ),
    (os.path.join("share", "yolov8_msgs", "msg"), glob(os.path.join("msg", "*.msg"))),
    (os.path.join("share", "silo_msgs", "msg"), glob(os.path.join("msg", "*.msg"))),
  ],
  install_requires=["setuptools"],
  zip_safe=True,
  maintainer="apil",
  maintainer_email="078bct017.apil@pcampus.edu.np",
  description="TODO: Package description",
  license="TODO: License declaration",
  tests_require=["pytest"],
  entry_points={
    "console_scripts": [
      "cam_optical2cam_ros_tf = transforms.cam_optical2cam_ros:main",
      "base2cam_optical_tf = transforms.base2cam_optical:main",
      "state_estimation_node = silo.estimate_state:main",
      "silo_matching_node = silo.match_silo:main",
    ],
  },
)
