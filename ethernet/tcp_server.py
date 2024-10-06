#!/usr/bin/env python3

import os
import socket

import cv2
import numpy as np

PORT = 12345
file_counter = 1


def receive_image():
  global file_counter

  # Set up socket
  server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server_socket.bind(("0.0.0.0", PORT))
  server_socket.listen(1)
  print(f"Listening on port {PORT}")

  while True:
    try:
      client_socket, addr = server_socket.accept()
      print(f"Connection from {addr}")

      while True:
        # Receive image size
        size_bytes = client_socket.recv(4)
        if not size_bytes:
          print("No size data received, closing connection.")
          break

        size = int.from_bytes(size_bytes, byteorder="big")

        # Receive image data
        img_data = b""
        while len(img_data) < size:
          chunk = client_socket.recv(size - len(img_data))
          if not chunk:
            raise RuntimeError("Socket connection broken")
          img_data += chunk

        if len(img_data) != size:
          print(f"Incomplete img_data received: {len(img_data)} out of {size} bytes")
          raise Exception("Incomplete img_data received")

        # Decode image
        np_arr = np.frombuffer(img_data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
          print("Failed to decode frame.")
          continue

        # Display image
        cv2.imshow("Received Image", cv_image)

        # Wait for 'c' key press to capture the image
        key = cv2.waitKey(1)
        if key == ord("c"):
          file_name = os.path.join("images", "v2", f"picam-{file_counter}.jpg")
          cv2.imwrite(file_name, cv_image)
          print(f"Captured image saved as {file_name}")
          file_counter += 1

        elif cv2.waitKey(1) & 0xFF == ord("q"):
          print("Exiting program.")
          client_socket.close()
          server_socket.close()
          cv2.destroyAllWindows()
          return

    except Exception as e:
      print(f"Error receiving image: {str(e)}")

  server_socket.close()


if __name__ == "__main__":
  receive_image()
