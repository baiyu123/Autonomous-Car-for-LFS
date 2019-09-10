try:
    from PIL import Image
except ImportError:
    import Image
import pytesseract
import cv2
import numpy as np

# # If you don't have tesseract executable in your PATH, include the following:
# # pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files (x86)\Tesseract-OCR\tesseract.exe'
# # Example tesseract_cmd = r'C:\Program Files (x86)\Tesseract-OCR\tesseract'
#
#
# def filter_with_color(img, lower_bound, upper_bound):
#     # Convert BGR to HSV
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#
#     # Threshold the HSV image to get only blue colors
#     mask = cv2.inRange(hsv, lower_bound, upper_bound)
#     kernel = np.ones((3, 3), np.uint8)
#     dilation = cv2.dilate(mask, kernel, iterations=2)
#     # create new image
#     shape = img.shape
#     blank_image = np.zeros((shape[0], shape[1], 3), np.uint8)
#     blank_image[:, :] = (255, 255, 255)
#     # inverse mask
#     # mask = cv2.bitwise_not(mask)
#     # Bitwise-AND mask and original image
#     res = cv2.bitwise_not(blank_image, blank_image, mask=dilation)
#     return res
#
# lower_white = np.array([0, 0, 150])
# upper_white = np.array([180, 10, 200])
#
# # Simple image to string
# img = cv2.imread('./images/f1.png',3)
#
# num = filter_with_color(img, lower_white, upper_white)
# # print(pytesseract.image_to_string(Image.open('./images/test.png')))
# speed = pytesseract.image_to_string(Image.fromarray(num),lang='eng')
#
# cv2.imshow('gray', num)
# # cv2.imwrite('./images/f5.png', num)
# k = cv2.waitKey(0)
# if k == 27:
#     cv2.destroyAllWindows()

# Import Python's socket module.
import socket
import struct
# Some constants.
INSIM_VERSION = 4
BUFFER_SIZE = 2048

# Packet types.
ISP_ISI = 1
ISP_VER = 2
ISP_TINY = 3
TINY_NONE = 0

# Initialise the socket in TCP mode.
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to LFS.
sock.connect(('localhost', 29999))

# Pack the IS_ISI data into a string.
value = ('BBBBHHBcH16s16s',
                  44,           # Size
                  1,            # Type
                  1,            # ReqI
                  0,            # Zero
                  0,            # UDPPort
                  0,            # Flags
                  0,            # Sp0
                  ' '.encode('utf-8'),          # Prefix
                  0,            # Interval
                  'liveforspeed12345Qq!'.encode('utf-8'),   # Admin
                  'MyProgram'.encode('utf-8')) # IName
isi = struct.pack(*value) # IName

# Send the string to InSim
sock.send(isi)

# We use a string as the buffer.
buffer = ''.encode('utf-8')

while True:
    data = sock.recv(BUFFER_SIZE)
    if data:
        buffer += data
        print(buffer)
        print("indicated size:" + str(buffer[0]))
        print("real size:" + str(len(buffer)))
        # Loop through completed packets.
        while len(buffer) > 0 and len(buffer) >= buffer[0]:

            # Copy the packet from the buffer.
            packet = buffer[:buffer[0]]
            # Check packet type.
            if buffer[1] == ISP_TINY:
                # Unpack the TINY packet and check it for keep-alive signal.
                size, type, reqi, subt = struct.unpack('BBBB', packet)
                if subt == TINY_NONE:
                    sock.send(packet)  # Respond to keep-alive.
            elif buffer[1] == ISP_VER:
                # Unpack the VER packet and check the InSim version.
                size, type, reqi, _, version, product, insimver = struct.unpack('BBBB8s6sH',buffer)
                print("size:"+str(size))
                print("version:"+str(version.decode("utf-8","ignore")))
                if insimver != INSIM_VERSION:
                    break  # Invalid version, break loop.
        buffer = buffer[buffer[0]:]
    else:
        break  # Connection has closed.

# Release the socket.
sock.close()