import sys
import cv2
import cv2.cv as cv
import numpy as np
import itertools as it
import pprint
import itertools
import random
import time
import re
import serial
import pickle
import argparse
import brutecontroller as bc

#ideal_positions = ([(163,51),(301,47),(152,428),(305,433)])
ideal_positions = ([(166,34),(325,34),(166,600),(325,600)])
#ideal_positions = [(169.0,96.0),(311.05.0),(240.0, 257.0),(240.0,393.0)]

V_CARD_WIDTH= ideal_positions[1][0] - ideal_positions[0][0]
V_CARD_HEIGHT = ideal_positions[2][1] - ideal_positions[0][1]
V_CARD_CENTER = (ideal_positions[0][0] + (V_CARD_WIDTH/2), ideal_positions[0][1] + (V_CARD_HEIGHT/2))

#real_positions = [(-3.5,4.7,-0.9),(1.6,4.7,-0.9),(0,0,0),(0,-4.5,0.2)]
real_positions = [(-2.3, 4.7, -2.2), (1.5,3., -2.1), (-2.3,-2.8,-1.8),(1.5,-2.8,-1.8)]
REAL_CARD_WIDTH = abs(real_positions[0][0] - real_positions[1][0])#3.6
REAL_CARD_HEIGHT = abs(real_positions[0][1] - real_positions[3][1]) #7.5

V2R = (float(REAL_CARD_WIDTH) / float(V_CARD_WIDTH), float(REAL_CARD_HEIGHT)/float(V_CARD_HEIGHT))

CIRCLE_THICKNESS = 2
CIRCLE_RADIUS =5
GREEN_COLOR=(0,255,0)
PURPLE_COLOR=(255,0,255)

DROP_Z = 0
CURRENT_POINT = {'x':0.0, 'y':0.0, 'z':4.0}

WINDOW_NAME = "R2B2"

CLIPSIZE = 20
IMG_SIZE = (480, 640)

FALSELIST = ("0","False","false","FALSE","f","F","no","No", "NO")
TRUELIST = ("1","True","true","TRUE","t","T", "yes", "Yes", "YES")

button_list = list()
image = None
cam = None
rotation_angle = None
shear_angle = None
perspective_xform = None
orientation = 0
detector = None
additional_detectors = list()
done_cracking = False
correct_pin = None
region = None
increment = 0.1
display = True


ser = None

FIRSTCHAR=ord('a')

SERIALPORT="COM4" #'/dev/tty.usbmodem1411'

XOF1=1.8
YOF1=.8
DR=1.75
LI=0;

global_layout = None


writedelay=.5
buttonids={}
counter=0

"""If True, the input and output to serial are shown on the console"""
SERIALTOCONSOLE=False

def serialsetup():
	global ser
	ser = serial.Serial(SERIALPORT, 57600, timeout=1)
	ser.flushInput()
	ser.flushOutput()
	readuntil(ser,'>')


# potentially useful for looking for characters
def squarish(group, margin):
	if (group != None):
		remove_list = []
		for box in group:
			if ((box[2]/box[3])> margin or(box[3]/box[2]) > margin):
				remove_list.append(box)

		for box in remove_list:
			group.remove(box)

	return group


# gives the axis-aligned bounding rect and its area for a given contour
def rectAndArea(contour):
	rect = cv2.boundingRect(contour)
	return (rect, rect[2] * rect[3])

# approximate the area by dividing by 1000, used to group contours into roughly similar-size groupings
def approx_area((rect, area)):
	MARGIN = 1000
	return area / MARGIN

# whether rect1 and rect2 are "near" one another, defined by some of their edges being close
def nearby(rect1, rect2):
	NEARBY_MARGIN = 30

	dx1 = np.abs(rect1[0] - rect2[0])
	dx2 = np.abs(dx1 - rect1[2])
	dx3 = np.abs(dx1 - rect2[2])
	dx = min((dx1, dx2, dx3))

	dy1 = np.abs(rect1[1] - rect2[1])
	dy2 = np.abs(dy1 - rect1[3])
	dy3 = np.abs(dy1 - rect2[3])
	dy = min((dy1, dy2, dy3))

	return (dx < NEARBY_MARGIN and dy < NEARBY_MARGIN)

# takes a group and removes any members that are not close enough to any other member
def distant_elimination(group):
	remove_list = []
	if (group != None and len(group) > 4):
		# test if all these potential buttons are near one another
		localized = False
		for box in group:
			for other_box in group:
				if (box != other_box):
					if (nearby(box, other_box)):
						localized = True
				if (localized):
					break
			if (not localized):
				remove_list.append(box)

		for box in remove_list:
			group.remove(box)

		return group
	else:
		return None


def dimension_check(group):
	# test to see if height and width are similar, as well as area
	DIMENSION_MARGIN = 10
	MIN_GROUP_SIZE = 2

	similar_dimensions = False
	remove_list = []

	# is there a way to combine the sums into a single function producing two values with a single pass?
	avg_dimensions = (sum(box[2] for box in group) / float(len(group)), sum(box[3] for box in group) / float(len(group)))
	for box in group:
		similar_dimensions = np.abs(avg_dimensions[0] - box[2]) < DIMENSION_MARGIN and np.abs(avg_dimensions[1] - box[3]) < DIMENSION_MARGIN
		if (not similar_dimensions):
			remove_list.append(box)

	for remove_box in remove_list:
		group.remove(remove_box)

	if (len(group) < MIN_GROUP_SIZE):
		return None

	return group

# remove boxes from the group if they are overlapping another box
def overlap_elimination(group, margin):
	if (group != None):
		remove_list = []
		for box in group:
			for other_box in group:
				if ((not other_box in remove_list) and box != other_box):
					if (too_close(box, other_box, margin)):
						remove_list.append(box)
						break

		for box in remove_list:
			group.remove(box)

		return group
	
# tests if two given rectangles are too close to one another and should be considered overlapping
def too_close((x1, y1, w1, h1), (x2, y2, w2, h2), margin):
	dx = np.abs(x1 - x2)
	dy = np.abs(y1 - y2)
	dw = np.abs(w1 - w2)
	dh = np.abs(h1 - h2)
	return (dx + dy + dw + dh) < margin


# find contours using the MSER function
def find_contours_MSER(img, minsize, maxsize, find_characters, margins):
	global display
	display = False
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#
#   DEFAULTS
#     CV_WRAP explicit MSER( int _delta=5, int _min_area=60, int _max_area=14400,
#           double _max_variation=0.25, double _min_diversity=.2,
#           int _max_evolution=200, double _area_threshold=1.01,
#           double _min_margin=0.003, int _edge_blur_size=5 );
	delta = 5
	minArea = minsize
	maxArea = maxsize
	maxVariation = 0.1
	minDiversity = 0.1
	maxEvolution = 200
	areaThreshold = 1.01
	minMarging = 0.003
	edgeBlurSize = 5
	mser = cv2.MSER(delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMarging, edgeBlurSize)

	contours = mser.detect(gray, None)
	buttons, stats = process_contours(contours, minsize, maxsize, img, "gray -> MSER", find_characters, margins)
	display = True
	return buttons

# find contours using the findContours function
def find_contours_FC(img, minsize, maxsize, find_characters, margins):

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	canny = cv2.Canny(gray, 0, 50)

	block_size = 5
	param = 1

	thresh = gray
	erosion_size = 3
	kernel = cv2.getStructuringElement(cv2.MORPH_ERODE, (erosion_size, erosion_size))
	thresh = cv2.erode(thresh, kernel)

	thresh = cv2.adaptiveThreshold(thresh, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, param)

	show(thresh, "thresh")
	show(canny, "canny")

	contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	buttons, stats = process_contours(contours, minsize, maxsize, img, "Canny -> findContours", find_characters, margins)

	return buttons

# processes a list of contours based on whether it should be looking for buttons or characters
def process_contours(contours, minsize, maxsize, img, contour_source, find_characters, (overlap_margin, squarish_margin)):
	# try to find 9+ boxes with almost exactly the same dimensions
	boxes = map(rectAndArea, filter(lambda cnt: cv2.contourArea(cnt) < maxsize and cv2.contourArea(cnt) > minsize, [r.reshape(-1, 1, 2) for r in contours]))

	tempimg = np.copy(img)
	for box in boxes:
		cv2.rectangle(tempimg, (box[0][0], box[0][1]), (box[0][0] + box[0][2], box[0][1] + box[0][3]), (0, 255, 0), 2)

	show(tempimg, "boxes")

	contour_data = boxes

	sorted_contours = sorted(contour_data, key=lambda c: c[1])


	groups = []
	keys = []
	for k, g in it.groupby(sorted_contours, approx_area):
		# just get the rect, don't need area any more
		groups.append(list(contour[0] for contour in g))
		keys.append(k)

	if (not find_characters):
		groups = [dimension_check(group) for group in groups]

	groups = filter(lambda g: g != None, [overlap_elimination(group, overlap_margin) for group in groups])

	if (not find_characters):
		groups = [distant_elimination(group) for group in groups]
	else:
		groups = filter(lambda g: g != None, [squarish(group, squarish_margin) for group in groups])


	groups = filter(lambda g: g != None and len(g) > 0, groups)

	for group in groups:
		if (group != None):
			tempimg = np.copy(img)
			for box in group:
				cv2.rectangle(tempimg, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
			show(tempimg, "Button Group from " + contour_source)


	if (len(groups) > 0 and (find_characters )):
		# flatten list
		buttons = [box for group in groups for box in group]
		average_area = sum(b[2] * b[3] for b in buttons) / len(buttons)
	elif (len(groups) > 0 and not find_characters):
		average_area = sum(sum(b[2] * b[3] for b in g) / float(len(g)) for g in groups) / float(len(groups))

		if (len(groups) > 1):
			buttons = sorted(groups, key=lambda g: np.abs(len(g) - 12))[0]
		else:
			buttons = groups[0]
	else:
		buttons = []
		average_area = 0

	tempimg = np.copy(img)
	for box in buttons:
		cv2.rectangle(tempimg, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
	show(tempimg, "ALL Button Group from " + contour_source)

	return buttons, average_area

# gets the angle of a line
def get_angle(line):
	return np.rad2deg(np.arctan(float(abs(line[1] - line[3])) / max(float(abs(line[0] - line[2])), .0000001)))


'''finds the longest line and returns its angle'''
def get_primary_axis_angle(lines):

	longest_line = sorted(lines, key=lambda line: np.square(line[0] - line[2]) + np.square(line[1] - line[3]))[-1]
	angle = get_angle(longest_line)
	if (longest_line[1] < longest_line[3]):
		angle = -angle

	return (angle, longest_line)

MARGIN = 20

'''finds the longest line close to perpendicular to the primary_angle'''
def get_secondary_axis_angle(lines):
	longest = 0
	longest_line = None
	for line in lines:
		length = np.square(line[0] - line[2]) + np.square(line[1] - line[3])
		angle = get_angle(line)
		if (length > longest and np.abs(angle) < MARGIN):
			longest_line = line
			longest = length

	if (longest_line != None):
		angle = get_angle(longest_line)
	else:
		angle = 0

	if (longest_line[1] > longest_line[3]):
		angle = -angle
	return (angle, longest_line)

# creates a shearing transform to remove the shear that is present in the image
def unshear(img, angle):
	m = np.zeros((2, 3))
	m[0][0] = 1
	m[0][1] = 0
	m[0][2] = 0
	m[1][0] = -np.sin(np.deg2rad(angle))
	m[1][1] = 1
	m[1][2] = 0
	img = cv2.warpAffine(img, m, (len(img[0]), len(img)), img, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
	return img


''' takes an image, finds the primary vertical and horizontal axes of
the object, and attempts to rotate and shear the image to make
these axes exactly vertical and horizontal'''
def derotate_and_deshear(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	
	# ret,thresh = cv2.threshold(gray,150,255,1)

	# Find contours with cv2.RETR_CCOMP
#   contours, hierarchy = cv2.find_contours(sub, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	canny = cv2.Canny(gray, 110, 100)

	show(img,"lines")
	show(get_frame(), "lines")
	MIN_LINE_LENGTH = 30
	lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 80, 0, MIN_LINE_LENGTH, 30)[0]

	for line in lines:
	     cv2.line(img, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), cv2.cv.CV_RGB(0, 255, 0), 4)
	(primary_angle, primary_line) = get_primary_axis_angle(lines)
	if (primary_angle > 0):
		rotation_angle = 90 - primary_angle
	else:
		rotation_angle = -(90 + primary_angle)
	
	cv2.line(img, (int(primary_line[0]), int(primary_line[1])), (int(primary_line[2]), int(primary_line[3])), cv2.cv.CV_RGB(255,0,255), 4)

	show(img, "lines")

	rotation_matrix = cv2.getRotationMatrix2D((len(img[0]) / 2, len(img) / 2), rotation_angle, 1)
	img = cv2.warpAffine(img, rotation_matrix, (IMG_SIZE[0], IMG_SIZE[1]), img, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)


	# POST ROTATION
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	canny = cv2.Canny(gray, 110, 100)
	lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 80, 0, 30, 10)[0]

#   for line in lines:
#     cv2.line(img, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), cv2.cv.CV_RGB(0, 255, 0), 4)



	(primary_angle, primary_line) = get_primary_axis_angle(lines)
#   cv2.line(img, (int(primary_line[0]), int(primary_line[1])), (int(primary_line[2]), int(primary_line[3])), cv2.cv.CV_RGB(0, 0, 255), 4)

	(shear_angle, secondary_line) = get_secondary_axis_angle(lines)

#   if (secondary_line != None):
#     cv2.line(img, (int(secondary_line[0]), int(secondary_line[1])), (int(secondary_line[2]), int(secondary_line[3])), cv2.cv.CV_RGB(255, 0, 0), 4)

	img = unshear(img, shear_angle)
	
	return (img, rotation_angle, shear_angle)

''' displays img in a window with title, waits for a key to be
pressed, then destroys the window, if display is true '''
def show(img, title):
	if (display):
		cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
		cv2.imshow(title, img)
		cv2.waitKey(0)
		cv2.destroyWindow(title)


''' converts the coordinates of a point on a derotated and desheared
angle into a point on the original rotated and sheared angle. Does not
incorporate deskewing yet but that will be easily added '''
def virtual_to_real(point, rotation_angle, shear_angle, perspective_xform):
	unsheared = np.mat(point[0],point[1],point[3])
	unsheared[1] = point[1] + (point[0] * (-np.sin(np.deg2rad(-shear_angle))))
	unsheared[2] = 1
	unrotation_matrix = np.mat([[.0,.0,.0],[.0,.0,.0],[.0,.0,.0]])
	unrotation_matrix[0,0] = np.cos(np.deg2rad(-rotation_angle)) 
	unrotation_matrix[0,1] = np.sin(np.deg2rad(-rotation_angle))
	unrotation_matrix[0,2] = (1 - ((np.cos(np.deg2rad(rotation_angle)) - np.sin(np.deg2rad(rotation_angle))))) * IMG_SIZE[0] / 2
	unrotation_matrix[1,0] = -np.sin(np.deg2rad(-rotation_angle))
	unrotation_matrix[1,1] = np.cos(np.deg2rad(rotation_angle))
	unrotation_matrix[1,2] = (1 - ((np.sin(np.deg2rad(rotation_angle)) + np.cos(np.deg2rad(rotation_angle))))) * IMG_SIZE[1] / 2
	untransformed = unrotation_matrix*unsheared
	
	return untransformed

def virtual_to_robot(point):
	print "Point:",point
	print"center:", V_CARD_CENTER        
	x = (point[0] - V_CARD_CENTER[0]) * V2R[0]
	y = -(point[1] - V_CARD_CENTER[1]) * V2R[1]
	

	#if orientation == 90:
		
	#elif orientation == 180:

	#elif orientation == 270:
			
		
	return x,y

''' transforms an image opposite of the rotation_angle and shear_angle
passed, used to revert an image that has undergone
derotate_and_deshear '''
def backTranslate(img, rotation_angle, shear_angle):
	img = unshear(img, -shear_angle)

	rotation_matrix = cv2.getRotationMatrix2D((len(img[0]) / 2, len(img) / 2), -rotation_angle, 1)

	img = cv2.warpAffine(img, rotation_matrix, IMG_SIZE, img, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
	show(img, "backtranslated")

	return img

''' static method, src and dst should be tweaked once the final frame
is built to get a constant perspective shift, since the camera will
always be offset by the same amount. Alternatively we can add code for
a calibration phase that will have a known image reference (let's say
a card) and then get a perspective shift based on how that card is
seen in the current setup versus how the reference shows it'''
def perspective_shift(img):
	global display, ideal_positions
	
	contour_boxes = overlap_elimination( find_contours_MSER(img, 5, 100000, True, (100, 2)),100)
	landmarks = [(float(x[0] + (x[2]/2)), float(x[1] +(x[3]/2))) for x in contour_boxes]
	landmarks = sorted(landmarks, key=lambda x: x[0] + (x[1] / 2))
	
	MAX_TRIES = 2
	tries = 0
	perspective_xform = None
	shifted_img = img

	while True:
		# TODO: Adjust the parameters to the find_contours_MSER to try to get 4 landmarks
#                if len(landmarks) > 4:
 #                       break

  #              elif len(landmarks) < 4:
   #                     break
		
		contour_boxes =  (filter (lambda x: x[2]*x[3] < 2000, overlap_elimination( find_contours_MSER(img, 5, 100000, True, (100, 2)),100)))
		landmarks = [(float(x[0] + (x[2]/2)), float(x[1] +(x[3]/2))) for x in contour_boxes]

		tempimg = np.copy(img)
		for box in contour_boxes:
			cv2.rectangle(tempimg, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
		cv2.imshow(WINDOW_NAME, tempimg)
		print "Identified these landmarks (press any key to continue)"
		cv2.waitKey(2000)
		
		# square is in "Z" pattern
		def in_square(square, points):
			result = list()
			for point in points:
				if (point[0] > square[0][0] and point[1] > square[0][1] and point[0] <square[1][0] and point[1] > square[0][1] and point[0] > square[2][0] and point[1] < square[2][1] and point[0] < square[3][0] and point[1] < square[3][1]):
					result.append(point)

			return result

		def sort_to_square(points):
			print points
			square = list()
			y_sorted = sorted(points, key=lambda p: p[1])
			square.append(min(y_sorted[0:2], key=lambda p: p[0]))
			square.append(max(y_sorted[0:2], key=lambda p: p[0]))
			square.append(min(y_sorted[2:4], key=lambda p: p[0]))
			square.append(max(y_sorted[2:4], key=lambda p: p[0]))
			print square
			return square
			
		def rectangularity (points):
			return abs(points[0][0] - points[2][0])+ abs(points[0][1] - points[1][1]) + abs(points[1][0] - points[3][0]) + abs(points[2][0] - points[3][0])
		
		def shift(square, img):
			src =np.array(square, np.float32)
			print(src)
			dst = np.array(ideal_positions, np.float32 )
			print(dst)
			xform =cv2.getPerspectiveTransform(src,dst)
			shifted_img = cv2.warpPerspective(img, xform, IMG_SIZE)

			return shifted_img, xform
		
		if len(landmarks) == 4:
			landmarks = sort_to_square(landmarks)
			shifted_img, perspective_xform = shift(landmarks, img)
			
		elif len(landmarks) > 4:
			perms = list(itertools.combinations(range(len(landmarks)), 4))
			potentials = list()                
			for perm in perms:
				print perm
				chosen = (landmarks[perm[0]], landmarks[perm[1]], landmarks[perm[2]],landmarks[perm[3]])
				chosen = sort_to_square(chosen)
				unchosen = [p for p in filter(lambda x: x not in chosen, landmarks)]

				if (len(in_square(chosen, unchosen))== 0):
					potentials.append((rectangularity(chosen),chosen))
					
				 #       print "RECT:", rectangularity(chosen)
				 #       tempimg = np.copy(img)
				 #       for point in chosen:
				 #               cv2.circle(tempimg, (int(point[0]), int(point[1])), 5, GREEN_COLOR, 3)
				 #       show(tempimg, "contours")

			

			potentials = [x[1] for x in sorted(potentials)]
			for group in potentials:
				shifted_img, perspective_xform = shift(group, img)
				if good_transform(shifted_img, ideal_positions):
					print "good"
					break
		else:
			cv2.imshow(WINDOW_NAME, img)
		
		tries += 1
		if (len(landmarks) or 4 and good_transform(shifted_img, ideal_positions)) or (tries >= MAX_TRIES):
			break
	
	print tries
	if (tries == MAX_TRIES):
		return img, None
	else:
		return (shifted_img, perspective_xform)


def good_transform (img, goal):
	# TODO: determine exactly how to measure a good transform -
	# obviously it will have the 4 contours in the right
	# positions; some way to test if it has no other contours
	# inside that box perhaps?
	contour_boxes = overlap_elimination( find_contours_MSER(img, 5, 100000, True, (100, 2)),100)
	return len(contour_boxes) == 4
	

''' creates the detector and trains it on features of the img '''
def create_detector(img):
	global region
		
	surfDetector = cv2.FeatureDetector_create("SURF")
	surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")
	
	imgg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	
	if (region != None):
		imgg = imgg[region[0]:region[2], region[1]:region[3]]

			
	keypoints = surfDetector.detect(imgg)
	keypoints, descriptors = surfDescriptorExtractor.compute(imgg,keypoints)

	samples = np.array(descriptors)
	responses = np.arange(len(keypoints),dtype= np.float32)
	
	# kNN training
	knn = cv2.KNearest()
	knn.train(samples, responses)
	return knn

''' returns True if the current image has too many feature mismatches
compared to the image used to train the detector'''
def detect_change(cur_img, mismatch_threshold, detector_to_use):
	global region
	DIST_THRESHOLD = 0.1

	surfDetector = cv2.FeatureDetector_create("SURF")
	surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")

	cur_imgg = cv2.cvtColor(cur_img,cv2.COLOR_BGR2GRAY)
	if (region != None):
		cur_imgg = cur_imgg[region[0]:region[2], region[1]:region[3]]
	
	cur_keypoints = surfDetector.detect(cur_imgg)
	cur_keypoints, cur_descriptors = surfDescriptorExtractor.compute(cur_imgg,cur_keypoints)

	mismatches = 0
	 
	for h,des in enumerate(cur_descriptors):
		des = np.asmatrix(des,np.float32)
		retval, results, neigh_resp, dists = detector_to_use.find_nearest(des,1)
		res,dist =  int(results[0][0]),dists[0][0]

		if dist > DIST_THRESHOLD:
			mismatches += 1
			
	return mismatches > mismatch_threshold


def on_button_identified(event, x, y, flag, param):
	global DROP_Z
	imagename, image, buttondict, buttonkey = param

	if event == cv2.EVENT_LBUTTONUP:
		print x,y
		scratchimage = np.zeros(image.shape, np.uint8)
		scratchimage[:] = image
		cv2.circle(scratchimage, (x,y), CIRCLE_RADIUS, PURPLE_COLOR, CIRCLE_THICKNESS)
		x,y = virtual_to_robot((x,y))
		buttondict[str(buttonkey)]= {'x':x, 'y':y, 'z':DROP_Z}
		pprint.pprint(buttondict)
		cv2.imshow(imagename, scratchimage)
		button = buttondict[str(buttonkey)]

		move(button['x'], button['y'], button['z'])

''' makes the window, displays the image, and adds the mouse callback to pick a button '''
def pick_button(frame, winname, buttondict, buttonname):
	global writedelay, increment
	cv2.setMouseCallback(winname, on_button_identified, (winname, frame, buttondict, buttonname))
	print "Click the %s button on the screen and then adjust Z axis using 'q' and 'e'" % buttonname
	print "Press 'c' when calibrated"
	cv2.imshow(winname, frame)
	ch = cv2.waitKey()

	while ch != ord('c')and ch != ord('v'):
		if buttonname in buttondict:
			if ch == ord('q'):
				button = buttondict[buttonname]
				button["z"] = button["z"] - increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])
											
			elif ch == ord('e'):
				button = buttondict[buttonname]
				button["z"] = button["z"] + increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])
			
			elif ch == ord('w'):
				button = buttondict[buttonname]
				button["y"] = button["y"] + increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])
				
			elif ch == ord('s'):
				button = buttondict[buttonname]
				button["y"] = button["y"] - increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])

			elif ch == ord('a'):
				button = buttondict[buttonname]
				button["x"] = button["x"] - increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])

			elif ch == ord('d'):
				button = buttondict[buttonname]
				button["x"] = button["x"] + increment
				button = buttondict[buttonname]
				direct_move(button['x'],button['y'],button['z'])

			elif ch == ord('p'):
				print "dumping", buttondict
				pickle.dump(buttondict, open("buttons.p", 'w'))

			elif ch == ord('b'):
				ch = cv2.waitKey()
				if (str(unichr(ch)) in buttondict):
					button =buttondict[str(unichr(ch))]
					move(button['x'],button['y'],button['z'])
				
			elif str(unichr(ch)) in [str(x) for x in range(10)]:
				increment = float(str(unichr(ch)))/ 10
				print "Changing Step-Size to: ", increment

				
		ch = cv2.waitKey()

def on_point_clicked(event, x, y, flag, param):
	global DROP_Z, CURRENT_POINT
	imagename, image = param

	if event == cv2.EVENT_LBUTTONUP:
		print x,y
		scratchimage = np.zeros(image.shape, np.uint8)
		scratchimage[:] = image
		cv2.circle(scratchimage, (x,y), CIRCLE_RADIUS, PURPLE_COLOR, CIRCLE_THICKNESS)
		x,y = virtual_to_robot((x,y))
		CURRENT_POINT = {'x':x, 'y':y, 'z':DROP_Z}
		cv2.imshow(imagename, scratchimage)
		move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
		
def calibrate_buttons():
	global cam, detector, increment, writedelay
	
	frame =get_frame()
	cv2.imshow(WINDOW_NAME, frame)

	print "Position the device under the robot."
	print "Press 'w' when this is completed"
	ch = None
	while ch != ord('w'):
		ch = cv2.waitKey()

	print "Calibration Beginning"
	
	detector = create_detector(get_frame())

	frame =get_frame()
	
	cv2.imshow(WINDOW_NAME, frame)
		
	
	find_drop()
	
	buttons = {}

	print "Press 'o'  to load a preconfigured button layout"
	ch = cv2.waitKey()
	if ch == ord('o'):
		buttons = pickle.load(open("buttons.p", 'r'))
	
	#print "Is there an \"OK\" button? (y/n)"
	#ch = cv2.waitKey()
	#if ch == ord('y'):
	#	pick_button(frame, WINDOW_NAME, buttons, "OK")
	
	#for number in range(0,10):
	#	pick_button(frame, WINDOW_NAME, buttons, str(number))


	cv2.setMouseCallback(WINDOW_NAME, on_point_clicked, (WINDOW_NAME, frame))
	print "Click on the screen and use q,w,e,a,s,d to move the robot."
	print "Press a number key (1-9) to change the size of the step for keyboard movements"
	print "Type \"SetBUTTONNAME\" to set a button's location"
	print "Type \"GotoBUTTONNAME\" to move to a previously defined button's location"
	print "Press Escape when finished"		
	cv2.imshow(WINDOW_NAME, frame)
	ch = cv2.waitKey()

	while ch != ord('c')and ch != ord('v'):
		if ch == ord('q'):
			CURRENT_POINT['z'] -= increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
										
		elif ch == ord('e'):
			CURRENT_POINT['z'] += increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
	
		elif ch == ord('w'):
			CURRENT_POINT['y'] += increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
							
		elif ch == ord('s'):
			CURRENT_POINT['y'] -= increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
			
		elif ch == ord('a'):
			CURRENT_POINT['x'] -= increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])

		elif ch == ord('d'):
			CURRENT_POINT['x'] += increment
			direct_move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
							
		elif ch == ord('p'):
			print "dumping", buttons
			pickle.dump(buttons, open("buttons.p", 'w'))

		elif str(unichr(ch)) in [str(x) for x in range(10)]:
			increment = float(str(unichr(ch)))/ 10
			print "Changing Step-Size to: ", increment

		elif get_word(ch, "Set"):
			set_button(buttons)
			
		elif get_word(ch, "Goto"):
			goto_button(buttons)
			
		elif ch == 27:
			break
		ch = cv2.waitKey()


	pickle.dump(buttons, open("buttons.p", 'w'))
				
	return buttons

def goto_button(buttons):
	global CURRENT_POINT

	print "Enter the name of the button to go to (press 'Return' when finished)"
	button_name = get_user_word()

	if  button_name in buttons:
		CURRENT_POINT = buttons[button_name]
		move(CURRENT_POINT['x'], CURRENT_POINT['y'], CURRENT_POINT['z'])
	else:
		print "No such button found"
	
	
	
def set_button(buttons):
        global CURRENT_POINT
        
        print "Enter the name of the button to set (press 'Return' when finished)"
        button_name = get_user_word()
        
        print "Set %s to %s" %(button_name, CURRENT_POINT)
        buttons[button_name] = CURRENT_POINT

def get_user_word(terminal_character="\r"):
	word = ""
	ch =cv2.waitKey()
	while ch != ord(terminal_character):
		word += str(unichr(ch))
		ch = cv2.waitKey()

	return word

''' this function tries to use cv2.waitKey to get words entered by a
user instead of just a single character. It assumes the user has
already entered the character contained by character_entered, and
begins by checking if that is the first character in word. If it is it
continues by querying for additional input, otherwise it returns
false.'''
def get_word(character_entered, word):
	if character_entered == ord(word[0]):
		for char in word[1:]:
			if cv2.waitKey() != ord(char):
				return False
		return True
	else:
		return False

def define_buttons():
	global image
	typedbuttons=raw_input("How many buttons? [10] ")
	numbuttons = int(typedbuttons) if typedbuttons.isdigit() else 10
	typedzero = raw_input("Is there a 0? [True] ")
	haszero = False if typedzero in FALSELIST else True
	print "haszero:%s"%haszero
	typedok = raw_input("Is there an OK or Enter button? [False] ")
	hasok = typedok in TRUELIST
	print "hasok:%s"%hasok
	
	
	buttoncoords={}
	
	if haszero:
		startnum=0
	else:
		startnum=1
		numbuttons+=1

	workingimage = np.zeros(image.shape, np.uint8)
	workingimage[:] = image
	
	for i in range(startnum,numbuttons):
		print("Click on button %d:"%i)
		pick_button(workingimage, "Click Button #%d"%i, buttoncoords, i)        
		cv2.waitKey(0)
		cv2.circle(workingimage, buttoncoords[str(i)], CIRCLE_RADIUS, GREEN_COLOR, CIRCLE_THICKNESS)
		
	if hasok:
		print("Click on OK/Enter button:")
		pick_button(workingimage, "Click OK", buttoncoords, 'OK')
		cv2.waitKey(0)
	
	return buttoncoords



def calibrate_camera(cam):
	global orientation
	
	perspective_xform = None
	
	print "Calibrating Camera: Press \"w\" when the camera is in position and the calibration card is placed directly under the robot"

	while True:
		vis = get_frame()
		
		cv2.imshow(WINDOW_NAME, vis)
		ch = 0xFF & cv2.waitKey(5)
		if ch == 27:
			cv2.destroyAllWindows()
			break
		if ch == ord('c'):
			cv.SaveImage("calib.jpg", cv.fromarray(vis))
		if ch == ord('w'):
			
			image = cv2.resize(vis,  IMG_SIZE, fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)
			
			#detector = create_detector(image)
			print "Calibrating..."
			image, perspective_xform = perspective_shift(image)

			if (perspective_xform != None):
				print "Calibration Attempted: press 'w' to accept, 'd' to reject and try again"
				cv2.imshow(WINDOW_NAME, image)
				ch = cv2.waitKey()
				while ch != ord('w') and ch != ord('d'):
					ch = cv2.waitKey()
					
				if ch == ord('w'):
					break
					
			else:
				print "Calibration Failed."
				print "Try repositioning the camera or card"

	return perspective_xform

def setup_camera(camnum=0):
	global cam
	cam = cv2.VideoCapture(camnum)
	cam.open(camnum)
	return cam

def get_frame():
	global perspective_xform, rotation_angle, shear_angle, orientation, cam
	if (cam == None):
		setup_camera()

	cam.read()
	frame = cv2.resize(cam.read()[1],  IMG_SIZE, fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)
	
	if orientation != 0:
		rotation_matrix = cv2.getRotationMatrix2D((IMG_SIZE[0]/2, IMG_SIZE[1]/2),orientation, 1)
		frame = cv2.warpAffine(frame, rotation_matrix, IMG_SIZE, frame, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
		
	
	if (perspective_xform != None):
		frame = cv2.warpPerspective(frame, perspective_xform, IMG_SIZE)
	
	
	if (rotation_angle != None):
		rotation_matrix = cv2.getRotationMatrix2D((len(frame[0]) / 2, len(frame) / 2), rotation_angle, 1)
		frame = cv2.warpAffine(frame, rotation_matrix, IMG_SIZE, frame, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)


	if (shear_angle != None):
		frame = unshear(frame, shear_angle)

	
	
	return frame                        

def readuntil(file,target):
	char =''
	total=''
	try:
		while char !='>':
			char=file.read(1)
			total+=char
		ser.flushInput()
		ser.flushOutput()
	except Exception as inst:
		print type(inst)     # the exception instance
		print inst.args      # arguments stored in .args
		print inst           # __str__ allows args to printed directly
		x, y = inst.args
		print 'x =', x
		print 'y =', y
	
	return total
	

def write(output):
	global writedelay
	"""Send output to the robot"""
	if SERIALTOCONSOLE:
		print "++TOSERIAL:%s"%output
	ser.write(output)
	time.sleep(writedelay)
	fromserial=readuntil(ser,'>')
	if SERIALTOCONSOLE:
		print "--FROMSERIAL:%s"%fromserial

def move(x,y,z):
	"""Move to the coordinates given"""
	write("MV X%s Y%s Z%s;"%(x,y,z))

def direct_move(x,y,z):
	write("DM X%s Y%s Z%s;"% (x,y,z))

def brutekeys(pinlength, keys="0123456789", randomorder=False):
	"""
	Returns a list of all possibilities to try, based on the length of PIN and buttons given.
	
	Yeah, lots of slow list copying here, but who cares, it's dwarfed by the actual guessing.
	"""
	allpossible = list(itertools.imap(lambda x: "".join(x),itertools.product(keys, repeat=pinlength)))
	if randomorder:
		random.shuffle(allpossible)

	return allpossible

def bruteloop(brutelist, buttondict, maxtries=None, actionlist=()):
	global done_cracking
	"""Try to push the buttons for each possible PIN in the given list
		
		If an actionlist is given, function in second position will be called
		every [first position] number of guesses, AFTER the guess.  For example, to wait 2 seconds after every
		guess, put an actionlist of ((1,somefuncthatwaits2seconds),). 
		
		The function called should have the signature funcname(guessnum, PIN, persistdata).  
		Return value should be None, or a tuple of a bool and a persistence value.  
		Any returned persistence value will be passed into persistdata on the next call.
		If the bool in the tuple is False, the bruteforcing is stopped.  When bruteloop exits,
		it returns the persistdata.  This can be used to indicate why the stop occurred.        
	"""
	
	if maxtries is None:
		maxtries=sys.maxint
	
	tries=0
	persister=None
	brutecontinue=True
			
	for pin in brutelist:
		print "===Pushing %s:"%(pin,)
					#for number in pin:
					#print "pushing %s"%number
					#push(toIdentifier(number))
		enterpin(pin, buttondict)
				
				#push(toIdentifier('Z'))
		tries+=1
		if (tries % 5 == 0):
			move(0,0,0)
			time.sleep(1)
			coordinate = buttondict['8']
			move(coordinate['x'], coordinate['y'], coordinate['z'])
			move(0,0,0)
			for i in range(30):
				time.sleep(1)
							
		for modulo,func in actionlist:
			if tries % modulo == 0:                 
				returnvalue=func(tries,pin,persister)
				if returnvalue is not None:
					brutecontinue, persister = returnvalue
		if tries>=maxtries or not brutecontinue or done_cracking:
			break
				
	move(0,0,0)


""" Couldn't get the EP command to work with 5 sets of coordinates - too much in one serial send?"""
def enterpin(pin, buttondict):
	global writedelay, additional_detectors, done_cracking, correct_pin, detector
	ok_required = True
	for number in pin:
		coordinate = buttondict[str(number)]
		move(coordinate['x'], coordinate['y'], coordinate['z'])
		time.sleep(writedelay)
		
	if (ok_required):
		coordinate = buttondict["OK"]
		move(coordinate['x'], coordinate['y'], coordinate['z'])
		time.sleep(writedelay)

	move(0,0,4)
	time.sleep(.2)
	frame = get_frame()
	if detect_change(frame, 100, detector):
		change_detected =True
		for d in additional_detectors:
			if (not detect_change(frame, 100, d)):
			    change_detected =False
			    break
		if (change_detected):
			print "CHANGE DETECTED!"
			print "Is it unlocked? (y/n)"
			cv2.namedWindow("Change", cv2.WINDOW_AUTOSIZE)
			cv2.imshow(WINDOW_NAME, frame)
			ch = cv2.waitKey()
			if ch == ord("n"):
				additional_detectors.append(create_detector(frame))
			else:
				correct_pin = pin
				done_cracking = True
				print "CORRECT PIN: ", pin

def find_drop():
	global DROP_Z, increment
	print "Use 'q' and 'e' to lower the finger until it contacts the device"
	print "Press 'c' when finished"
	DROP_Z = 4
	ch = cv2.waitKey()
	while ch != ord('c'):
		if ch == ord('q'):
			DROP_Z -= increment
			direct_move(0,0,DROP_Z)
		elif ch == ord('e'):
			DROP_Z += increment
			direct_move(0,0,DROP_Z)
		elif str(unichr(ch)) in [str(x) for x in range(10)]:
			increment = float(str(unichr(ch)))/ 10
		ch =cv2.waitKey()
	

def main(args):
	global display, image, cam, shear_angle, rotation_angle, perspective_xform, orientation, detector
	
	parser = argparse.ArgumentParser(description='This program controls a brute-forcing robot. Load arguments from a file with @FILENAME', fromfile_prefix_chars='@')
	#parser.add_argument('-c','--config', help='NI! loads a config file')
	parser.add_argument('-p','--positions',help='NI! import a saved positions file')
	parser.add_argument('-r','--resume',help='NI! resume a previous run')
	parser.add_argument('-s','--serialdevice',help='NI! serial device (Mac/Linux) or COM port like "COMx" (Windows)')
	parser.add_argument('-v','--videonum',help='NI! Video capture device. "0" is the first, default value')
	parser.add_argument('-k','--keyconfig', help='NI! Use keyboard configuration, not camera configuration', action="store_true")
	parser.add_argument('-n','--nodetect', help='NI! Do not attempt to detect a finished run.  Runs until the series is completed', action="store_true")
	parser.add_argument('-f','--pinfile', help='NI! Load brute force attempts from a file')
	parser.add_argument('-a','--android', help='NI! Android mode.  Waits 30 seconds each 5 guesses, then presses ok', action="store_true")
	
	
	args = parser.parse_args()
 
	## show values ##
	print args
 
	#exit()
	
	display = True

	# move robot out of the way
	serialsetup()
	move(0,0,5)


	cam = setup_camera()

	cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
	print "Press a key to indicate the rotation of the camera: 'q' = viewing from left, 'w' = viewing from top, 'e' = viewing from right,'r' = viewing from bottom"
	while True:
		cv2.imshow(WINDOW_NAME, get_frame())
		ch = cv2.waitKey(5)
		if ch == ord('q'):
			orientation = 270
			break
		elif ch == ord('w'):
			orientation = 180
			break
		elif ch == ord('e'):
			orientation = 90
			break
		elif ch == ord('r'):
			orientation = 0
			break
	
	perspective_xform = calibrate_camera(cam) 
	if perspective_xform == None:
		print "Calibration Failed. Exiting"
		return 

	print "Now place device to PIN crack as close to the middle of the calibration card as possible"
	print "Press \"w\" when this is completed"
	
			
	#(frame, rotation_angle, shear_angle) = derotate_and_deshear(frame)
	
	image = get_frame()
	writedelay = .5
	
	buttons = calibrate_buttons()

	move(0,0,4)

	keys = brutekeys(4, randomorder=False)
	bruteloop(keys,buttons, maxtries=10000)

	cv2.destroyAllWindows()
	return 0
	show(image, "fixed image")

	# run until ESCAPE is pressed
	while True:
		pick_buttons()
		if cv2.waitKey(10)== 27:
			break

	buttons=define_buttons()
	pprint.pprint(buttons)
		
	cv2.destroyAllWindows()
		
if __name__ == "__main__":
	main(sys.argv)
