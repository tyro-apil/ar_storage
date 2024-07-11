from typing import Dict, List


def compute_consistent_state(silos_received_state, silos_absolute_state):
  breakpoint()
  consistent_state = silos_absolute_state
  for silo_received, silo_previous in zip(silos_received_state, silos_absolute_state):
    # breakpoint()
    if len(silo_received["state"]) < len(silo_previous["state"]):
      print(
        f"Silo-{silo_received['index']} -> Previous: {silo_previous['state']} balls | Received: {silo_received['state']}"
      )
      continue
    if len(silo_received["state"]) > 3:
      print(
        f"Silo-{silo_received['index']} has more than 3 balls | State: {silo_received['state']}"
      )
      continue
    prev_state_len = len(silo_previous["state"])
    if not silo_received["state"][:prev_state_len] == silo_previous["state"]:
      continue
    consistent_state[silo_received["index"] - 1]["state"] = silo_received["state"]
  return consistent_state


def predict_full_state(partial_state, previous_state: List[Dict], aligned_silo):
  if aligned_silo == 0:
    return None
  print(previous_state)
  aligned_index_relative = get_relative_index_aligned_silo(partial_state)
  predicted_state = previous_state

  print(previous_state)
  for silo in partial_state:
    offset = aligned_index_relative - silo["index"]
    predicted_state[aligned_silo - offset - 1].copy()["state"] = silo["state"]
    print(previous_state)

  print(6969)
  print(previous_state)
  return predicted_state


def get_relative_index_aligned_silo(partial_state):
  return 2
  x_center_image: int = 460
  closest_center_x = 1000
  closest_index = 0
  for silo in partial_state:
    bbox = silo["bbox"]
    center_x = (bbox[0] + bbox[2]) / 2
    if abs(center_x - x_center_image) < abs(x_center_image - closest_center_x):
      closest_center_x = center_x
      closest_index = silo["index"]
  return closest_index


def display_state(state):
  for silo in state:
    print(f"Silo-{silo['index']}: {silo['state']} | ")


def test_compute_consistent_state():
  print("Test compute_consistent_state")
  final_state = compute_consistent_state(silos_received_state, silos_absolute_state)
  display_state(final_state)


def test_predict_full_state():
  print("Test predict_full_state")
  final_state = predict_full_state(silos_partial_state, silos_absolute_state, 3)
  display_state(final_state)


def test_combined():
  breakpoint()
  print("Absolute state previous")
  display_state(silos_absolute_state)
  print("Received state")
  display_state(silos_partial_state)
  print("Integration test")
  absolute_state_copy = silos_absolute_state.copy()
  predicted_state = predict_full_state(silos_partial_state, absolute_state_copy, 3)
  print("Predicted state")
  display_state(predicted_state)
  # breakpoint()
  final_state = compute_consistent_state(predicted_state, silos_absolute_state)
  print("Final state")
  display_state(final_state)


silos_absolute_state = [
  {"index": 1, "state": "", "bbox": [0, 0, 100, 100]},
  {"index": 2, "state": "", "bbox": [0, 0, 100, 100]},
  {"index": 3, "state": "BR", "bbox": [0, 0, 100, 100]},
  {"index": 4, "state": "R", "bbox": [0, 0, 100, 100]},
  {"index": 5, "state": "", "bbox": [0, 0, 100, 100]},
]

silos_received_state = [
  {"index": 1, "state": "", "bbox": [0, 0, 100, 100]},
  {"index": 2, "state": "", "bbox": [0, 0, 100, 100]},
  {"index": 3, "state": "BR", "bbox": [0, 0, 100, 100]},
  {"index": 4, "state": "RB", "bbox": [0, 0, 100, 100]},
  {"index": 5, "state": "", "bbox": [0, 0, 100, 100]},
]

silos_partial_state = [
  {"index": 1, "state": "", "bbox": [0, 0, 100, 100]},
  {"index": 2, "state": "BRB", "bbox": [0, 0, 100, 100]},
  {"index": 3, "state": "", "bbox": [0, 0, 100, 100]},
]


def main():
  # test_compute_consistent_state()
  # test_predict_full_state()
  # test_combined()
  pass


if __name__ == "__main__":
  main()
