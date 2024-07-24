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
      "cam_optical2cam_ros_tf = silo.cam_optical2cam_ros_tf:main",
      "base2cam_optical_tf = silo.base2cam_optical_tf:main",
      "state_estimation_node = silo.estimate_state:main",
      "state_estimation_node_HSV = silo.estimate_state_HSV:main",
      "silo_selection_node = silo.select_silo:main",
      "absolute_silo_state_node = silo.absolute_silo_state:main",
      "image_receiver_node = silo.image_receiver:main",
      # Rviz visualizations
      "silos_marker_node = rviz.balls_silo:main",
      "target_node = rviz.target_silo:main",
      # Fake publisher nodes
      "fake_map2base_link_node = fake.map2base_link:main",
      "fake_silo_state_publisher_node = fake.silo_state_map:main",
    ],
  },
)
