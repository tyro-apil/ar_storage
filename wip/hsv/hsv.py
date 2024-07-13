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

IMG_DIR = "/home/apil/stuff/silo_pics/valid_test"
RESULTS_DIR = "/home/apil/stuff/silo_pics/debug"


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

  red1_h_low = 0
  red1_h_high = 10
  red1_s_low = 120
  red1_s_high = 255
  red1_v_low = 50
  red1_v_high = 235

  red2_h_low = 170
  red2_h_high = 180
  red2_s_low = 120
  red2_s_high = 255
  red2_v_low = 60
  red2_v_high = 220

  blue1_h_low = 80
  blue1_h_high = 110
  blue1_s_low = 130
  blue1_s_high = 170
  blue1_v_low = 30
  blue1_v_high = 90

  blue2_h_low = 100
  blue2_h_high = 115
  blue2_s_low = 100
  blue2_s_high = 150
  blue2_v_low = 110
  blue2_v_high = 230

  threshold = 0.33

  # cv2.imshow("combined_mask", combined_mask)
  # cv2.imshow("colored_mask", colored_mask)
  # cv2.waitKey(0)

  results: List[Results] = model.predict(source=IMG_DIR, imgsz=(512, 928), device=0)

  for r in results:
    # breakpoint()
    file_name = os.path.basename(r.path)
    r = r.cpu().numpy()

    img: cv2.Mat = r.orig_img
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(
      hsv_img,
      (red1_h_low, red1_s_low, red1_v_low),
      (red1_h_high, red1_s_high, red1_v_high),
    )
    red_mask_2 = cv2.inRange(
      hsv_img,
      (red2_h_low, red2_s_low, red2_v_low),
      (red2_h_high, red2_s_high, red2_v_high),
    )

    blue_mask_1 = cv2.inRange(
      hsv_img,
      (blue1_h_low, blue1_s_low, blue1_v_low),
      (blue1_h_high, blue1_s_high, blue1_v_high),
    )
    blue_mask_2 = cv2.inRange(
      hsv_img,
      (blue2_h_low, blue2_s_low, blue2_v_low),
      (blue2_h_high, blue2_s_high, blue2_v_high),
    )

    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    blue_mask = cv2.bitwise_or(blue_mask_1, blue_mask_2)

    kernel = np.ones((5, 5), np.uint8)

    red_mask = cv2.dilate(red_mask, kernel, iterations=2)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)

    combined_mask = cv2.bitwise_or(red_mask, blue_mask)
    colored_mask = cv2.bitwise_and(img, img, mask=combined_mask)

    # breakpoint()

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

      # annotator.box_label(b, model.names[int(c)])

    # print(silos)
    # img = annotator.result()
    # cv2.imshow("colored_mask", colored_mask)
    # # cv2.imshow("YOLO V8 Detection", img)
    bgr_resize = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
    mask_resize = cv2.resize(
      colored_mask, (colored_mask.shape[1] // 2, colored_mask.shape[0] // 2)
    )
    combined_img = cv2.hconcat([bgr_resize, mask_resize])
    hsv_resize = cv2.resize(hsv_img, (hsv_img.shape[1] // 2, hsv_img.shape[0] // 2))

    def print_hsv_value(event, x, y, flags, param):
      if event == cv2.EVENT_LBUTTONDOWN:
        if x >= hsv_resize.shape[1] or y >= hsv_resize.shape[0]:
          hsv_value = hsv_resize[y, x - hsv_resize.shape[1]]
        else:
          hsv_value = hsv_resize[y, x]
        print(f"HSV value at ({x}, {y}): {hsv_value}")

    cv2.namedWindow("combined_img")
    cv2.setMouseCallback("combined_img", print_hsv_value)

    cv2.imshow("combined_img", combined_img)
    if cv2.waitKey(0) == ord("q"):
      break

    # debug_path = os.path.join(RESULTS_DIR, file_name)
    # cv2.imwrite(debug_path, colored_mask)

  cv2.destroyAllWindows()
  pass


if __name__ == "__main__":
  main()
