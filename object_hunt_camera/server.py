# USAGE
# python server.py --prototxt MobileNetSSD_deploy.prototxt --model MobileNetSSD_deploy.caffemodel --montageW 2 --montageH 2

# import the necessary packages
from imutils import build_montages
from datetime import datetime
import time
import numpy as np
import imagezmq
import argparse
import imutils
import cv2
import socket
import sys
import struct
# for catching signals
import signal

def handleSignal(signalNumber, frame): 
    print('Received:', signalNumber)
    print('Leaving Script...')
    sys.exit()
    return

# register the signal to be caught
signal.signal(signal.SIGINT, handleSignal)

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--confidence", type=float, default=0.2,
    help="minimum probability to filter weak detections")
ap.add_argument("-mW", "--montageW", required=True, type=int,
    help="montage frame width")
ap.add_argument("-mH", "--montageH", required=True, type=int,
    help="montage frame height")
args = vars(ap.parse_args())

# print("Test keyboard features:")
# while True:
# 	if keyboard.is_pressed('a'):
# 		print("a was pressed!")
# 		break

# initialize the ImageHub object to receive images from the RPi
imageHub = imagezmq.ImageHub()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the object_hunt_base service is listening
server_address = ('10.0.0.1', 19002)



# List of trained classes of the COCO dataset
CLASSES = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}

# load pretrained MobilenetSSD v2 trained on the COCO dataset
print("[INFO] loading model...")
net = cv2.dnn.readNetFromTensorflow("./models/frozen_inference_graph.pb"
                                    , "./models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt")
# initialize the consider set (class labels we care about and want
# to count), the object count dictionary, and the frame  dictionary
CONSIDER = set(["sports ball"])
objCount = {obj: 0 for obj in CONSIDER}
frameDict = {}

# initialize the dictionary which will contain  information regarding
# when a device was last active, then store the last time the check
# was made was now
lastActive = {}
lastActiveCheck = datetime.now()

# stores the estimated number of Pis, active checking period, and
# calculates the duration seconds to wait before making a check to
# see if a device was active
ESTIMATED_NUM_PIS = 1
ACTIVE_CHECK_PERIOD = 10
ACTIVE_CHECK_SECONDS = ESTIMATED_NUM_PIS * ACTIVE_CHECK_PERIOD

# assign montage width and height so we can view all incoming frames
# in a single "dashboard"
mW = args["montageW"]
mH = args["montageH"]
print("[INFO] detecting: {}...".format(", ".join(obj for obj in
    CONSIDER)))
# Marker for found object
found = False

# start looping over all the frames
while True:
    # receive RPi name and frame from the RPi and acknowledge
    # the receipt
    (rpiName, frame) = imageHub.recv_image()
    imageHub.send_reply(b'OK')

    # if a device is not in the last active dictionary then it means
    # that its a newly connected device
    if rpiName not in lastActive.keys():
        print("[INFO] receiving data from {}...".format(rpiName))

    # record the last active time for the device from which we just
    # received a frame
    lastActive[rpiName] = datetime.now()

    # resize the frame to have a maximum width of 400 pixels, then
    # grab the frame dimensions and construct a blob
    frame = imutils.resize(frame, width=400)
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
        swapRB=True)

    # pass the blob through the network and obtain the detections and
    # predictions
    net.setInput(blob)
    detections = net.forward()

    # reset the object count for each object in the CONSIDER set
    objCount = {obj: 0 for obj in CONSIDER}

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the prediction
        confidence = detections[0, 0, i, 2]
        idx = int(detections[0, 0, i, 1])
        #filter out sports ball
        if idx == 37:

            # filter out weak detections by ensuring the confidence is
            # greater than the minimum confidence
            if confidence > args["confidence"]:
                found = True
                # extract the index of the class label from the
                # detections
                

                # check to see if the predicted class is in the set of
                # classes that need to be considered
                if CLASSES[idx] in CONSIDER:
                    # increment the count of the particular object
                    # detected in the frame
                    objCount[CLASSES[idx]] += 1

                    # compute the (x, y)-coordinates of the bounding box
                    # for the object
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # draw the bounding box around the detected object on
                    # the frame
                    cv2.rectangle(frame, (startX, startY), (endX, endY),
                        (255, 0, 0), 2)

    # draw the sending device name on the frame
    cv2.putText(frame, rpiName, (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # draw the object count on the frame
    label = ", ".join("{}: {}".format(obj, count) for (obj, count) in
        objCount.items())
    cv2.putText(frame, label, (10, h - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255,0), 2)

    # update the new frame in the frame dictionary
    frameDict[rpiName] = frame
    # build a montage, to present different image streams, depending on the number of RPis connected
    montages = build_montages(frameDict.values(), (w, h), (mW, mH))
    # print("montages length: ", len(montages))

    #display the montage(s) on the screen
    for (i, montage) in enumerate(montages):
        cv2.imshow("Autonomous object detection ({})".format(i),
            montage)


    # detect any kepresses, while displaying image
    key = cv2.waitKey(1) & 0xFF

    # if current time *minus* last time when the active device check
    # was made is greater than the threshold set then do a check
    if (datetime.now() - lastActiveCheck).seconds > ACTIVE_CHECK_SECONDS:
        # loop over all previously active devices
        for (rpiName, ts) in list(lastActive.items()):
            # remove the RPi if he has been inactive for the last ten seconds 
            if (datetime.now() - ts).seconds > ACTIVE_CHECK_SECONDS:
                print("[INFO] lost connection to {}".format(rpiName))
                lastActive.pop(rpiName)
                frameDict.pop(rpiName)

        # set timestamp to the last check
        lastActiveCheck = datetime.now()
    
    # if object was found, a message will be sent to the object_hunt_base service to communicate the detection
    # "11" was defined as the object received identifier
    # "13" was defined as the object received identifier
    if found:
        found = False
        print ('[INFO] connecting to %s on port %s' % server_address)
        sock.connect(('10.0.0.1', 19002))

        try:
            print('[INFO] sending the message')
            sock.send(struct.pack('<B', 11))
            print('message sent')
            # wait for acknowledgement
            response = sock.recv(1)
            print('message received')
            # print the response
            print("[INFO] response: ", struct.unpack('<B', response)[0])

        finally:
            print('[INFO] Closing TCP connection')
            sock.close()

        break

while True:
    # show the picutre of detection for presentation purposes
    cv2.imshow("Autonomous object detection ({})".format(i),
            montage)

# clean up of the cv processes
cv2.destroyAllWindows()