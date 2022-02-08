from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2

def draw_text(image, weight, x, y):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (x,y)
    fontScale              = 0.6
    fontColor              = (0,0,255)
    thickness              = 1
    lineType               = 2
    cv2.putText(image,'{}'.format(weight), bottomLeftCornerOfText, font, fontScale,fontColor,thickness,lineType)

def test(image0, hog):
    # load the image and resize it to (1) reduce detection time
	# and (2) improve detection accuracy
	#image = cv2.imread(imagePath)
    image = image0.copy()
    image = imutils.resize(image, width=min(600, image.shape[1]))
    orig = image.copy()
	# detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(8, 8),
		padding=(8, 8), scale=1.05)
	# draw the original bounding boxes
    for rec in range(len(rects)):
        (x, y, w, h) = rects[rec]
        weight = weights[rec] 
        if weight > 0.9:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
            draw_text(orig, weight, x, y)
	# apply non-maxima suppression to the bounding boxes using a
	# fairly large overlap threshold to try to maintain overlapping
	# boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.85)
	# draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
	    cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
    return orig

# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--images", required=True, help="path to images directory")
#args = vars(ap.parse_args())
cap = cv2.VideoCapture("../../../data/dynamic/nshA1_1_images/output.mp4")
out = cv2.VideoWriter('detected.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, (600, 400))
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
while(cap.isOpened()):
    # ret = a boolean return value from getting the frame, frame = the current frame being projected in the video
    ret, frame = cap.read()
    if ret:
        image = test(frame, hog)
        image = cv2.resize(image, (600,400))
        out.write(image)
    else:
        print('bad frame')
        break
    cv2.imshow("After NMS", image)
    cv2.waitKey(5)

cap.release()
out.release()
cv2.destroyAllWindows()