#!/usr/bin/env python3

import os
from typing import Dict, List, Tuple

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.engine.results import Boxes, Results
from ultralytics.utils.plotting import (
  Annotator,  # ultralytics.yolo.utils.plotting is deprecated
)

MODELS_DIR = "/home/apil/main_ws/src/robot/models"
MODEL_NAME = "picam_mount.pt"


def get_match_percent(hsv_img: cv2.Mat, roi: Tuple, mask: cv2.Mat) -> float:
  hsv_img_copy = hsv_img.copy()
  mask_copy = mask.copy()
  x1, y1, x2, y2 = roi
  roi_img = hsv_img_copy[y1:y2, x1:x2]
  roi_mask = mask_copy[y1:y2, x1:x2]

  roi_mask = cv2.bitwise_and(roi_img, roi_img, mask=roi_mask)
  roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_HSV2BGR)
  roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_BGR2GRAY)

  match_percent = cv2.countNonZero(roi_mask) / (roi_mask.shape[0] * roi_mask.shape[1])
  return match_percent


def main():
  model_path = os.path.join(MODELS_DIR, MODEL_NAME)
  model = YOLO(model_path)

  img_path = os.path.join("silo.jpg")
  img = cv2.imread(img_path)
  hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  red_h_low = 0
  red_h_high = 10
  red_s_low = 100
  red_s_high = 255
  red_v_low = 80
  red_v_high = 255

  blue_h_low = 85
  blue_h_high = 110
  blue_s_low = 50
  blue_s_high = 255
  blue_v_low = 30
  blue_v_high = 255

  threshold = 0.30

  red_mask = cv2.inRange(
    hsv_img, (red_h_low, red_s_low, red_v_low), (red_h_high, red_s_high, red_v_high)
  )
  blue_mask = cv2.inRange(
    hsv_img,
    (blue_h_low, blue_s_low, blue_v_low),
    (blue_h_high, blue_s_high, blue_v_high),
  )

  kernel = np.ones((5, 5), np.uint8)

  red_mask = cv2.dilate(red_mask, kernel, iterations=3)
  blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

  combined_mask = cv2.bitwise_or(red_mask, blue_mask)
  colored_mask = cv2.bitwise_and(img, img, mask=combined_mask)

  # cv2.imshow("combined_mask", combined_mask)
  # cv2.imshow("colored_mask", colored_mask)
  # cv2.waitKey(0)

  results: List[Results] = model.predict(source=img, imgsz=(512, 928), device=0)

  for r in results:
    r = r.cpu().numpy()
    annotator = Annotator(r.orig_img)

    boxes: Boxes = r.boxes.cpu().numpy()

    silos: List[Dict] = []

    for box in boxes:
      b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
      b = [int(i) for i in b]
      c = box.cls[0]

      ## Get detected silos
      if not model.names[int(c)] == "silo":
        continue

      ## Get individual silo bboxes sliced
      silo_h = b[3] - b[1]
      silo_w = b[2] - b[0]

      ## Divide silo into three parts in Y axis
      y_divisions = [-0.10 * silo_h, 0.20 * silo_h, 0.60 * silo_h, 0.95 * silo_h]
      y_divisions = [int(i) for i in y_divisions]

      roi_1 = (
        b[0],
        b[1] + y_divisions[2],
        b[2],
        b[1] + y_divisions[3],
      )

      roi_2 = (
        b[0],
        b[1] + y_divisions[1],
        b[2],
        b[1] + y_divisions[2],
      )

      roi_3 = (
        b[0],
        max(0, b[1] + y_divisions[0]),
        b[2],
        b[1] + y_divisions[1],
      )

      cv2.rectangle(colored_mask, roi_1[:2], roi_1[2:], (0, 255, 0), 2)
      cv2.rectangle(colored_mask, roi_2[:2], roi_2[2:], (0, 255, 0), 2)
      cv2.rectangle(colored_mask, roi_3[:2], roi_3[2:], (0, 255, 0), 2)

      rois = [roi_1, roi_2, roi_3]

      state = ""
      for roi in rois:
        red_match = get_match_percent(hsv_img, roi, red_mask)
        blue_match = get_match_percent(hsv_img, roi, blue_mask)

        if red_match > threshold and red_match > blue_match:
          state += "R"
        elif blue_match > threshold and blue_match > red_match:
          state += "B"

      cv2.putText(
        colored_mask,
        f"{state}",
        (b[0] + int(0.1 * silo_w), b[3] - int(0.1 * silo_h)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        1,
      )
      silo = {"bbox": b, "state": state}
      silos.append(silo)

      annotator.box_label(b, model.names[int(c)])

    print(silos)
    img = annotator.result()
    cv2.imshow("colored_mask", colored_mask)
    cv2.imshow("YOLO V8 Detection", img)
    cv2.waitKey(0)

  cv2.destroyAllWindows()
  pass


if __name__ == "__main__":
  main()
