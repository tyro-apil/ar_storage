#!/usr/bin/env python3

import os
from typing import List

import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Boxes, Results
from ultralytics.utils.plotting import (
  Annotator,  # ultralytics.yolo.utils.plotting is deprecated
)

MODELS_DIR = "/home/apil/main_ws/src/robot/models"
MODEL_NAME = "picam_mount.pt"


def main():
  model_path = os.path.join(MODELS_DIR, MODEL_NAME)
  model = YOLO(model_path)

  img_path = os.path.join("silo.jpg")
  img = cv2.imread(img_path)
  hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  red_h_low = 0
  red_h_high = 10
  red_s_low = 50
  red_s_high = 255
  red_v_low = 100
  red_v_high = 255

  blue_h_low = 85
  blue_h_high = 110
  blue_s_low = 50
  blue_s_high = 255
  blue_v_low = 100
  blue_v_high = 255

  red_mask = cv2.inRange(
    hsv_img, (red_h_low, red_s_low, red_v_low), (red_h_high, red_s_high, red_v_high)
  )
  blue_mask = cv2.inRange(
    hsv_img,
    (blue_h_low, blue_s_low, blue_v_low),
    (blue_h_high, blue_s_high, blue_v_high),
  )
  cv2.imshow("red_mask", red_mask)
  cv2.imshow("blue_mask", blue_mask)
  cv2.waitKey(0)

  annotated_img = img.copy()

  # BGR to RGB conversion is performed under the hood
  # see: https://github.com/ultralytics/ultralytics/issues/2575
  results: List[Results] = model.predict(source=img, imgsz=(512, 928), device=0)

  for r in results:
    r = r.cpu().numpy()
    # breakpoint()
    annotator = Annotator(r.orig_img)

    boxes: Boxes = r.boxes.cpu().numpy()
    for box in boxes:
      b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
      b = [int(i) for i in b]
      c = box.cls[0]

      ## Get detected silos
      if not model.names[int(c)] == "silo":
        continue

      ## Get individual silo bboxes sliced
      silo_img = r.orig_img[b[1] : b[3], b[0] : b[2]]
      silo_h, silo_w, _ = silo_img.shape

      # breakpoint()

      ## Divide silo into three parts in Y axis
      height_division = (int(0.24 * silo_h), int(0.62 * silo_h), int(silo_h))

      start_1 = (b[0], max(0, b[1] - 50))
      end_1 = (b[2], b[1] + height_division[0])
      cv2.rectangle(annotated_img, start_1, end_1, (0, 0, 255), 2)

      start_2 = (b[0], b[1] + height_division[0])
      end_2 = (b[2], b[1] + height_division[1])
      cv2.rectangle(annotated_img, start_2, end_2, (0, 0, 255), 2)

      start_3 = (b[0], b[1] + height_division[1])
      end_3 = (b[2], b[1] + height_division[2])
      cv2.rectangle(annotated_img, start_3, end_3, (0, 0, 255), 2)

      ## Get HSV values of each pixel in each part

      ## if about 30% pixels have desired HSV values then name that as red or blue ball

      annotator.box_label(b, model.names[int(c)])

    img = annotator.result()
    cv2.imshow("YOLO V8 Detection", img)
    cv2.imshow("Silos", annotated_img)
    cv2.waitKey(0)

  cv2.destroyAllWindows()
  pass


if __name__ == "__main__":
  main()
