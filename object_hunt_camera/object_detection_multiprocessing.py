from imutils.video import VideoStream
from imutils.video import FPS
from multiprocessing import Process
from multiprocessing import Queue
import numpy as np
import argparse
import imutils
import time
import cv2

import socket
import sys
import struct

BUFFER_SIZE = 256
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 19002)
print ('connecting to %s port %s' , server_address)
sock.connect(server_address)

# processing the frame in function for multiprocessing purposes (net.forward() ist the bottleneck)
def classify_frame(net, inputQueue, outputQueue):
    #keep looping
    while True:
        if not inputQueue.empty():
            #grab the frame from the inputQueue, resize it and construct a blob from it
            frame = inputQueue.get()
            blob = cv2.dnn.blobFromImage(frame, size=(300,300), swapRB=True)
            
            #set the frame as input to our deep learning object
            # detector and obtain detections
            net.setInput(blob)
            detections = net.forward()
            
            # write detections to the output queue
            outputQueue.put(detections)
            

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", required=False,
	help="path to Tensorflow frozen graph")
ap.add_argument("-m", "--model", required=False,
	help="path to Tensorflow pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.2,
	help="minimum probability to filter weak detections")
args = vars(ap.parse_args())


# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
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
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromTensorflow("models/ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb", 
                                    "models/ssdlite_mobilenet_v2_coco_2018_05_09/ssdlite_mobilenet_v2_coco_2018_05_09.pbtxt")

# initialize the input queue (frames), output queue (detections)
# initialize the actual list of detections returned by the child process
inputQueue = Queue(maxsize=1)
outputQueue = Queue(maxsize=1)
detections = None

# construct a child process *independent* from our main process of execution
print("[Info] starting process...")
p = Process(target=classify_frame, args=(net, inputQueue, outputQueue))
p.daemon = True
p.start()

# and initialize the FPS counter
print("[INFO] starting video stream...")
vs = VideoStream(src=0, framerate=2, usePiCamera=True, resolution=(300, 300)).start()
time.sleep(5.0)
fps = FPS().start()
found = False

while True:
    # getting image from camera feed
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    #(fH, fW) = frame.shape[:2]
    # handing over the frame to the parallel object detection
    if inputQueue.empty():
        inputQueue.put(frame)
    
    # getting results from the object detection
    if not outputQueue.empty():
        detections = outputQueue.get()

       
    if detections is not None:
        # loop over the detections
        for detection in detections[0,0, :, :]:
            # extract the confidence (i.e., probability) associated with the prediction
            idx = int(detection[1])
            #print("detection: ", detection[1])
            confidence = detection[2]

            if confidence >= args["confidence"]:
                
                print("Class: ", str(CLASSES[idx]),"    Confidence: ", confidence, "    Index: ", idx)
                # filter out weak detections by ensuring the `confidence` is
                # greater than the minimum confidence
                if idx == 44:
                    # extract the index of the class label from the `detections`,
                    # then compute the (x, y)-coordinates of the bounding box for
                    # the object
                    
                    print("Found detection with index: ", idx, " and confidence: ", confidence)
                    # Send data
                    # message = 11
                    print('[INFO ]sending the message')
                    sock.send(struct.pack('<B', 11))
                    # Wait for the response
                    while found == False:
                        time.sleep(0.5)
                        response = sock.recv(1)
                        unpacked_response = struct.unpack('<B', response)
                        print("response: ", unpacked_response[0])
                        # defined response of 13 as acknowledgement from mother process
                        if unpacked_response[0] is 13:
                            print("[INFO] Received an answer")
                            found = True

                    #print("Index of detection: ", idx)
                    # box = detection[3:7] * np.array([fW, fH, fW, fH])
                    # (startX, startY, endX, endY) = box.astype("int")
                    # print("startX: {}; startY, {}\nendX, {}, endY, {}".format(startX,startY,endX,endY))

                    # #display the prediction
                    # label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                    # cv2.rectangle(frame, (startX, startY), (endX, endY),
                    #         COLORS[idx], 2)
                    # y = startY - 15 if startY - 15 > 15 else startY + 15
                    # cv2.putText(frame, label, (startX, y),
                    #         cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[idx], 2)

                    # show the output image
    #cv2.imshow("Frame", frame)
        # stoping the process if the found object got acknowledged
            if found:
                break
    if found:
        break
    # updating the FPS timer
    fps.update()
    
# stoping the FPS counter
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# cleanup of videostream
cv2.destroyAllWindows()
vs.stop()

