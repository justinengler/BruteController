import threading
import sys
import os
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

ideal_positions = ([(163,51),(301,47),(152,428),(305,433)])
#ideal_positions = ([(210,41),(423,41),(214,600),(423, 600)])

V_CARD_WIDTH= ideal_positions[1][0] - ideal_positions[0][0]
V_CARD_HEIGHT = ideal_positions[2][1] - ideal_positions[0][1]
V_CARD_CENTER = (ideal_positions[0][0] + (V_CARD_WIDTH/2), ideal_positions[0][1] + (V_CARD_HEIGHT/2))

#real_positions = [(-2.3, 4.7, -2.2), (1.5,3., -2.1), (-2.3,-2.8,-1.8),(1.5,-2.8,-1.8)]
real_positions =[(-2.34,3.74,-2.5),(1.16,3.74,-2.3),(-2.34,-3.56,-2.4),(1.16,-3.26,-2.2)]
REAL_CARD_WIDTH = abs(real_positions[0][0] - real_positions[1][0])#3.6
REAL_CARD_HEIGHT = abs(real_positions[0][1] - real_positions[3][1]) #7.5

V2R = (float(REAL_CARD_WIDTH) / float(V_CARD_WIDTH), float(REAL_CARD_HEIGHT)/float(V_CARD_HEIGHT))

CIRCLE_THICKNESS = 2
CIRCLE_RADIUS =5
GREEN_COLOR=(0,255,0)
PURPLE_COLOR=(255,0,255)

DROP_Z = 0
CURRENT_POINT = {'x':0.0, 'y':0.0, 'z':0.0}

WINDOW_NAME = "R2B2"

CLIPSIZE = 20
IMG_SIZE = 480

FALSELIST = ("0","False","false","FALSE","f","F","no","No", "NO")
TRUELIST = ("1","True","true","TRUE","t","T", "yes", "Yes", "YES")

DUMMYIMAGE = "pinpad.jpg"

DONE = False

drag_start=False
selection = None

cooldown = False
cooldown_time = 0
attempts_per_cooldown = 0

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
increment = 0.1
display = True



ser = None

FIRSTCHAR=ord('a')


DEFAULTSERIALPORT='COM4'


writedelay=.5

"""If True, the input and output to serial are shown on the console"""
SERIALTOCONSOLE=False

def serialsetup(serialport,isreverse):
	global ser
	ser = serial.Serial(serialport, 57600, timeout=1)
	ser.flushInput()
	ser.flushOutput()
	readuntil(ser,'>')
	if isreverse:
		write("RV;")


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
	minMargin = 0.003
	edgeBlurSize = 5
	mser = cv2.MSER(delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMargin, edgeBlurSize)

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
	boxes = [x[0] for x in map(rectAndArea, filter(lambda cnt: cv2.contourArea(cnt) < maxsize and cv2.contourArea(cnt) > minsize, [r.reshape(-1, 1, 2) for r in contours]))]
	
	boxes = squarish(boxes, squarish_margin)
	if len(boxes) > 0:
		average_area = sum(b[2] * b[3] for b in boxes) / len(boxes)
	else:
		average_area = 0
		
	return boxes, average_area


def virtual_to_robot(point):
	print "Point:",point
	print"center:", V_CARD_CENTER        
	x = (point[0] - V_CARD_CENTER[0]) * V2R[0]
	y = -(point[1] - V_CARD_CENTER[1]) * V2R[1]
		
	return x,y

def point_in_box(point, box):
	x,y =point
	bx,by,bw,bh = box
	return x>= bx and x <= (bx+bw)and y>=by and y<=(by+bh)

def select_landmark(event, x, y, flag, param):
	frame, landmarks,selected_landmarks  = param
	in_box = False
	if event == cv.CV_EVENT_LBUTTONDOWN:
		for mark in landmarks:
			if point_in_box((x,y),mark):
				in_box = True
				if (mark[0] + mark[2]/2, mark[1] + mark[3]/2)not in selected_landmarks:
					selected_landmarks.append((mark[0] + mark[2]/2, mark[1] + mark[3]/2))
				break

		if not in_box:
			selected_landmarks.append((x,y))



''' static method, src and dst should be tweaked once the final frame
is built to get a constant perspective shift, since the camera will
always be offset by the same amount. Alternatively we can add code for
a calibration phase that will have a known image reference (let's say
a card) and then get a perspective shift based on how that card is
seen in the current setup versus how the reference shows it'''
def perspective_shift(img):
	global display, ideal_positions
	max_area = 2000
	min_contour_size  = 5
	max_contour_size = 1000000
	margins = (100,2)
	
	MAX_TRIES = 10
	tries = 0
	perspective_xform = None
	shifted_img = img
	
	while True:
		# TODO: Adjust the parameters to the find_contours_MSER to try to get 4 landmarks
#                if len(landmarks) > 4:
 #                       break

  #              elif len(landmarks) < 4:
   #                     break
		
		contour_boxes =  (filter (lambda x: x[2]*x[3] < 2000, overlap_elimination( find_contours_MSER(img, min_contour_size, max_contour_size, True,  margins),100)))
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

	

		
		if len(landmarks) == 4:
			landmarks = sort_to_square(landmarks)
			shifted_img, perspective_xform = shift(landmarks, img)
			print "Are these landmarks correct? (y/n)"
			while True:
				ch = cv2.waitKey()
				if ch ==ord('y'):
					break
				elif ch == ord('n'):
					print "Perform manual landmark entry (or try again)? (y/n)"
					while True:
						ch = cv2.waitKey()
						if ch == ord('y'):
							shifted_img,perspective_xform = do_manual_selection(img, contour_boxes)
							break
						elif ch == ord('n'):
							break
					break
							
						
					
		elif len(landmarks) != 4:
			if len(landmarks) > 4:
				print "More than 4 marks found, please select the four calibration marks or press Esc"
			else:
				print "Fewer than 4 marks found, please select the four calibration marks or press Esc"

			shifted_img, perspective_xform = do_manual_selection(img, contour_boxes)				
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
			
def shift(square, img):
	src =np.array(square, np.float32)
	print(src)
	dst = np.array(ideal_positions, np.float32 )
	print(dst)
	xform =cv2.getPerspectiveTransform(src,dst)
	shifted_img = cv2.warpPerspective(img, xform, (IMG_SIZE,IMG_SIZE))

	return shifted_img, xform

def do_manual_selection(img, contour_boxes):
	correct_landmarks =list()
	
	cv2.setMouseCallback(WINDOW_NAME, select_landmark, (img, contour_boxes,correct_landmarks))
	ch = cv2.waitKey(5)
	while len(correct_landmarks)< 4 and ch != 23:
		ch = cv2.waitKey(5)

	landmarks = sort_to_square(correct_landmarks)
	return shift(landmarks, img)
	
def good_transform (img, goal):
	# TODO: determine exactly how to measure a good transform -
	# obviously it will have the 4 contours in the right
	# positions; some way to test if it has no other contours
	# inside that box perhaps?
	contour_boxes = overlap_elimination( find_contours_MSER(img, 5, 100000, True, (100, 2)),100)
	return len(contour_boxes) == 4
	

''' creates the detector and trains it on features of the img '''
def create_detector(img):
	global selection
		
	surfDetector = cv2.FeatureDetector_create("SURF")
	surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")
	
	imgg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	
	if (selection != None):
		imgg = imgg[selection[1]:selection[1]+selection[3], selection[0]:selection[0] +selection[0]]

			
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
	global selection
	DIST_THRESHOLD = 0.1

	surfDetector = cv2.FeatureDetector_create("SURF")
	surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")

	cur_imgg = cv2.cvtColor(cur_img,cv2.COLOR_BGR2GRAY)
	if (selection != None):
		cur_imgg = cur_imgg[selection[1]:selection[1]+selection[3], selection[0]:selection[0] +selection[0]]

	
	cur_keypoints = surfDetector.detect(cur_imgg)
	cur_keypoints, cur_descriptors = surfDescriptorExtractor.compute(cur_imgg,cur_keypoints)

	mismatches = 0
	 
	if cur_descriptors != None:
		for h,des in enumerate(cur_descriptors):
			des = np.asmatrix(des,np.float32)
			retval, results, neigh_resp, dists = detector_to_use.find_nearest(des,1)
			res,dist =  int(results[0][0]),dists[0][0]

			if dist > DIST_THRESHOLD:
				mismatches += 1
			
	return mismatches > mismatch_threshold

def selection_drag(event, x, y, flags, param):
	global drag_start, selection

	selection_img = np.copy(param)
	if event == cv.CV_EVENT_LBUTTONDOWN:
	    drag_start = (x, y)
	    selection = (x,y,1,1)
	if event == cv.CV_EVENT_LBUTTONUP:
	    drag_start = None
	    selection = None
	if drag_start:
	    xmin = min(x, drag_start[0])
	    ymin = min(y, drag_start[1])
	    xmax = max(x,drag_start[0])
	    ymax = max(y, drag_start[1])
	    selection = (xmin, ymin, xmax - xmin, ymax - ymin)
	    cv2.rectangle(selection_img, (selection[0],selection[1]), (selection[0] + selection[2], selection[1] + selection[3]), (0, 255, 0), 2)
	    cv2.imshow("R2B2", selection_img)


def get_focus_area(frame):
	global selection, drag_start,detector
	
	print "Drag to select the area of the image to focus on for changes to avoid false positive unlocks"
	print "(the box the PIN appears in is recommended)"
	print "Press 'w' when this is satisfactory"

	selection_img = np.copy(frame)

	cv2.setMouseCallback(WINDOW_NAME, selection_drag, (selection_img))
	ch = cv2.waitKey(5)
	while ch != ord('w'):
		ch =cv2.waitKey(5)

	detector = create_detector      (frame)
	

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
	
		
def calibrate_buttons(keyboardonly=False, ):
	global cam, detector, increment, writedelay
	
	frame =get_frame()
	cv2.imshow(WINDOW_NAME, frame)

	print "Position the device under the robot."
	print "Press 'w' when this is completed"
	ch = None
	while ch != ord('w'):
		ch = cv2.waitKey()

	print "Calibration Beginning"

	get_frame()
	frame = get_frame()
	get_focus_area(frame)

	cv2.destroyWindow("R2B2")
	cv2.namedWindow("R2B2", cv2.WINDOW_AUTOSIZE)

	cv2.imshow(WINDOW_NAME, frame)
		
	
	find_drop()
	
	buttons = {}

	cv2.setMouseCallback(WINDOW_NAME, on_point_clicked, (WINDOW_NAME, frame))
	print "Click on the screen and use q,w,e,a,s,d to move the robot."
	print "Commands are as folllows: "
	print "Press a number key (1-9) to change the size of the step for keyboard movements"
	print "\"SetBUTTONNAME\" to set a button's location"
	print "\"GotoBUTTONNAME\" to go to a previously defined button's location"
	print "\"ReadFILENAME\" to load a previously saved configuration file"
	print "\"WriteFILENAME\" to save this configuration"
	print "\"Defined\" to print the buttons defined so far"
	print "Press ESC when finished"         
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

		elif get_word(ch, "Read"):
			buttons = load_config()

		elif get_word(ch, "Write"):
			save_config(buttons)

		elif get_word(ch, "Defined"):
			print_config(buttons)
		
			
		elif ch == 27:
			print "Are you sure you are ready to start? ('y/n')"
			print_config(buttons)
			ch =cv2.waitKey()
			if ch ==ord('y'):
				break
						
		ch = cv2.waitKey()


	pickle.dump(buttons, open("buttons-auto.p", 'w'))
				
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
	
	newpoint=dict(CURRENT_POINT)
	
	print "Set %s to %s" %(button_name, newpoint)
	buttons[button_name] = newpoint

def load_config():
	print "Enter the name of the configuration file to read(press 'Return' when finished)"
	file_name = get_user_word()
	return pickle.load(open(file_name, 'r'))

def save_config(buttons):
	print "Enter the name of the file to write to (press 'Return' when finished)"
	file_name = get_user_word()
	pickle.dump(buttons, open(file_name, 'w'))

def print_config(buttons):
	sorted_buttons = sorted((i,j) for i,j in buttons.items())	
	print "BUTTONS DEFINED:"
	for pair in sorted_buttons:
		print pair
	

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



def calibrate_camera(cam):
	global orientation
	
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
			
			image = cv2.resize(vis,  (IMG_SIZE, IMG_SIZE), fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)
			
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
	global perspective_xform, rotation_angle, shear_angle, orientation, cam, IMG_SIZE
	if (cam == None):
		#setup_camera()
		"""This means we're in a non-camera mode.  Return a dummy image"""
		return cv2.imread(DUMMYIMAGE)

	cam.read()
	frame = cv2.resize(cam.read()[1], (IMG_SIZE, IMG_SIZE), fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)
	
	if orientation != 0:
		rotation_matrix = cv2.getRotationMatrix2D((IMG_SIZE/2,IMG_SIZE/2),orientation, 1)
		frame = cv2.warpAffine(frame, rotation_matrix,(IMG_SIZE, IMG_SIZE), frame, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
		
	
	if (perspective_xform != None):
		frame = cv2.warpPerspective(frame, perspective_xform, (IMG_SIZE, IMG_SIZE))
	
	
	if (rotation_angle != None):
		rotation_matrix = cv2.getRotationMatrix2D((len(frame[0]) / 2, len(frame) / 2), rotation_angle, 1)
		frame = cv2.warpAffine(frame, rotation_matrix, (IMG_SIZE,IMG_SIZE), frame, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)


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
	#z=-z if reversez else z
	write("MV X%s Y%s Z%s;"%(x,y,z))

def direct_move(x,y,z):
	#z=-z if reversez else z
	write("DM X%s Y%s Z%s;"% (x,y,z))

def brutekeys(pinlength, keys="0123456789", randomorder=False):
	"""
	Returns a list of all possibilities to try, based on the length of s and buttons given.
	
	Yeah, lots of slow list copying here, but who cares, it's dwarfed by the actual guessing.
	"""
	allpossible = list(itertools.imap(lambda x: "".join(x),itertools.product(keys, repeat=length)))
	if randomorder:
		random.shuffle(allpossible)

	return allpossible

def bruteloop(brutelist, buttondict, maxtries=None, actionlist=(), startpoint = 0):
	global cooldown_time, attempts_per_cooldown, cooldown
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

	i = startpoint
	while i < len(brutelist)and brutecontinue and tries < maxtries:
		pin = brutelist[i]
		print "===Pushing %s:"%(pin,)
		print "Press ESC to pause"

		brutecontinue = enterpin(pin, buttondict)
				
		tries+=1
							
		for modulo,func in actionlist:
			if tries % modulo == 0:                 
				returnvalue=func(tries,pin,persister, buttondict)
				if returnvalue is not None:
					brutecontinue, persister = returnvalue
		ch = cv2.waitKey(1)
		if ch == 27:
			print "Press ESC to resume"
			print "Type 'quit' to exit"
			ch = cv2.waitKey()
			while True:
				if ch == 27:
					print "Resuming"
					break
				elif get_word(ch, "quit"):
					print "Quitting"
					brutecontinue = False
					break
				ch = cv2.waitKey()
			
			
			
		i += 1
		
	move(0,0,0)


""" Couldn't get the EP command to work with 5 sets of coordinates - too much in one serial send?"""
def enterpin(pin, buttondict):
	global writedelay, additional_detectors, correct_pin, detector
	ok_required = True
	for number in pin:
		if (number in buttondict):
			coordinate = buttondict[str(number)]
			move(coordinate['x'], coordinate['y'], coordinate['z'])
			time.sleep(writedelay)
		
	if ('OK' in buttondict):
		coordinate = buttondict["OK"]
		move(coordinate['x'], coordinate['y'], coordinate['z'])
		time.sleep(writedelay)
	elif ('ok' in buttondict):
		coordinate = buttondict["ok"]
		move(coordinate['x'], coordinate['y'], coordinate['z'])
		time.sleep(writedelay)


	move(0,0,0)
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
			ch = cv2.waitKey()
			if ch == ord("n"):
				additional_detectors.append(create_detector(frame))
			else:
				correct_pin = pin
				print "CORRECT PIN: ", pin
				return false
	return True
				
def find_drop():
	global DROP_Z, increment
	print "Use 'q' and 'e' to lower the finger until it contacts the device"
	print "Press 'c' when finished"
	DROP_Z = 0
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
	global display, image, cam, shear_angle, rotation_angle, perspective_xform, orientation, detector, IMG_SIZE
	
	parser = argparse.ArgumentParser(description='This program controls a brute-forcing robot. Load arguments from a file with @FILENAME', fromfile_prefix_chars='@')
	parser.add_argument('-p','--positions',help='NI! import a saved positions file')
	parser.add_argument('-r','--resume',help='NI! resume a previous run')
	parser.add_argument('-s','--serialdevice',help='Serial device (Mac/Linux) or COM port like "COMx" (Windows)', default=DEFAULTSERIALPORT)
	parser.add_argument('-v','--videonum',help='Video capture device. "0" is the first, default value', default=1)
	parser.add_argument('-k','--keyconfig', help='Use keyboard configuration, not camera configuration', action="store_true")
	parser.add_argument('-n','--nodetect', help='NI! Do not attempt to detect a finished run.  Runs until the series is completed', action="store_true")
	parser.add_argument('-f','--pinfile', help='Load brute force attempts from a file')
	parser.add_argument('-a','--android', help='Android mode.  Waits 30 seconds each 5 guesses, then presses ok', action="store_true")
	parser.add_argument('-z','--reversez', help='Reverse the Z axis', action="store_true")
	
	args = parser.parse_args()
 
	## show values ##
	print args
 	
 
	#exit()
	
	display = True
	
	
	newreversez=bool(args.reversez)
	if args.reversez is None:
		print "User test here"
		print "make sure to set true or false to newreversez"


	# move robot out of the way
	serialsetup(args.serialdevice, args.reversez)
	move(0,0,3)

	if not args.keyconfig:
		cam = setup_camera(int(args.videonum))
		perspective_xform = calibrate_camera(cam) 
		if perspective_xform == None:
			print "Calibration Failed. Exiting"
			return 

	centername= "robot" if args.keyconfig else "calibration card"
	print "Now place device to PIN crack as close to the middle of the %s as possible"%centername
	print "Press \"w\" when this is completed"
	
	image = get_frame()
	writedelay = .05
	
	buttons = calibrate_buttons()

	move(0,0,0)

	if args.pinfile != None:
		key = load_pinfile(args["pinfile"])
	else:
		keys = load_pinfile("pins.txt")
		#keys = brutekeys(4, randomorder=False)

	if args.android:
		bruteloop(keys,buttons, maxtries=10000, actionlist = [(5, standroid_PIN_wait)])
	else:
		bruteloop(keys,buttons, maxtries=10000)

	cv2.destroyAllWindows()
	return 0

def standroid_PIN_wait(tries, pin, persistant_data, buttondict):
	move(0,0,1)
	button = buttondict['8']
	move(button['x'], button['y'], button['z'])
	move (0,0,1)
	time.sleep(30)
	return True, persistant_data
								      

def load_pinfile(pinfile_name):
	keys = list()
	pinfile = open(pinfile_name, 'r')
	for line in pinfile:
		keys.append(line.split('#')[0])
	return keys
		
if __name__ == "__main__":
	main(sys.argv)
