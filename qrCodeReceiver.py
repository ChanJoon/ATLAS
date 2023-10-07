import time
import socket
import threading
import cv2
import pyzbar
from pyzbar.pyzbar import decode
from PIL import Image, ImageTk

## MARK: -------- QR Code --------

class QrCodeReceiver:
    def __init__(self):
        self.data = None
        self.connect_server()
        
    def connect_server(self):
        while True:
            try:
                self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_server.bind(("localhost", 1234))
                self.socket_server.listen(1)
                self.connection, self.address = self.socket_server.accept()
                break
            except ConnectionRefusedError:
                print("Connection refused. Retrying in 1 second...")
                time.sleep(1)
            except TimeoutError:
                print("Connection timeout. Retrying in 1 second...")
                time.sleep(1)
            except Exception as e:
                print(f"Error occurred: {str(e)}")
                break

    def receive_data(self):
        while True:
            self.data = self.connection.recv(1024).decode()
            if self.data is not None and self.data != "":
                print("Received data:", self.data)