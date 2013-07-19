import sys
import cv2
import cv2.cv as cv
import numpy as np
import itertools as it
import pprint

CIRCLE_THICKNESS = 2
CIRCLE_RADIUS =5
GREEN_COLOR=(0,255,0)
PURPLE_COLOR=(255,0,255)


CLIPSIZE = 20
IMG_SIZE = 480

FALSELIST = ("0","False","false","FALSE","f","F","no","No", "NO")
TRUELIST = ("1","True","true","TRUE","t","T", "yes", "Yes", "YES")

button_list= list()
image = None

display = True
'''
Currently set up to find characters, not buttons
USE: python recognize.py [image] <b>
b indicates finding buttons, by default it
will find characters at the moment
'''

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
the main object, and attempts to rotate and shear the image to make
these axes exactly vertical and horizontal'''
def derotate_and_deshear(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # ret,thresh = cv2.threshold(gray,150,255,1)

        # Find contours with cv2.RETR_CCOMP
#   contours, hierarchy = cv2.find_contours(sub, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        canny = cv2.Canny(gray, 110, 100)

        MIN_LINE_LENGTH = 30
        lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 80, 0, MIN_LINE_LENGTH, 30)[0]

#     for line in lines:
#       cv2.line(img, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), cv2.cv.CV_RGB(0, 255, 0), 4)
        (primary_angle, primary_line) = get_primary_axis_angle(lines)
        if (primary_angle > 0):
                rotation_angle = 90 - primary_angle
        else:
                rotation_angle = -(90 + primary_angle)
        
#   cv2.line(img, (int(primary_line[0]), int(primary_line[1])), (int(primary_line[2]), int(primary_line[3])), cv2.cv.CV_RGB(255,0,255), 4)

        rotation_matrix = cv2.getRotationMatrix2D((len(img[0]) / 2, len(img) / 2), rotation_angle, 1)
        img = cv2.warpAffine(img, rotation_matrix, (IMG_SIZE, IMG_SIZE), img, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)


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

def combine(orient, img1, img2, img3=None, img4=None):
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        if orient == 'f':
                h3, w3 = img3.shape[:2]
                h4, w4 = img4.shape[:2]
                vis = np.zeros(max(h1, h2) + max(h3, h4), max(w1, w3) + max(w2, w4), np.uint8)
                vis[0:h1, 0:w1] = img1
                vis[0:h2, w1:w1 + w2] = img2
                vis[h1:h1 + h3, 0:w3] = img3
                vis[h2:h2 + h4, w3:w3 + w4] = img4
        elif orient == 'h':
                vis = np.zeros((max(h1, h2), w1 + w2), np.uint8)
                vis[0:h1, 0:w1] = img1
                vis[0:h2, w1:w1 + w2] = img2
        elif orient == 'v':
                vis = np.zeros((max(h1, h2), w1 + w2), np.uint8)
                vis[0:h1, 0:w1] = img1
                vis[h1:h1 + h2, 0:w2] = img2

        # vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        return vis

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
        unrotation_matrix[0,2] = (1 - ((np.cos(np.deg2rad(rotation_angle)) - np.sin(np.deg2rad(rotation_angle))))) * IMG_SIZE / 2
        unrotation_matrix[1,0] = -np.sin(np.deg2rad(-rotation_angle))
        unrotation_matrix[1,1] = np.cos(np.deg2rad(rotation_angle))
        unrotation_matrix[1,2] = (1 - ((np.sin(np.deg2rad(rotation_angle)) + np.cos(np.deg2rad(rotation_angle))))) * IMG_SIZE / 2
        untransformed = unrotation_matrix*unsheared
        
        return untransformed

''' transforms an image opposite of the rotation_angle and shear_angle
passed, used to revert an image that has undergone
derotate_and_deshear '''
def backTranslate(img, rotation_angle, shear_angle):
        img = unshear(img, -shear_angle)

        rotation_matrix = cv2.getRotationMatrix2D((len(img[0]) / 2, len(img) / 2), -rotation_angle, 1)

        img = cv2.warpAffine(img, rotation_matrix, (IMG_SIZE, IMG_SIZE), img, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
        show(img, "backtranslated")

        return img

''' static method, src and dst should be tweaked once the final frame
is built to get a constant perspective shift, since the camera will
always be offset by the same amount. Alternatively we can add code for
a calibration phase that will have a known image reference (let's say
a card) and then get a perspective shift based on how that card is
seen in the current setup versus how the reference shows it'''
def perspective_shift(img):
        global display
        display = False
        contour_boxes = overlap_elimination( find_contours_MSER(img, 5, 100000, True, (100, 2)),100)
        landmarks = [(float(x[0] + (x[2]/2)), float(x[1] +(x[3]/2))) for x in contour_boxes]

        display = True
        #tempimg = np.copy(img)
        #for box in contour_boxes:
        #        cv2.rectangle(tempimg, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 1)
        #        print box[0] + (box[2] / 2), box[1] + (box[3]/2)
        #show(tempimg, "LANDMARKS")

        ideal_positions = ([(163,51),(301,47),(152,428),(305,433)])

        
        landmarks = sorted(landmarks, key=lambda x: x[0] + (x[1] / 2))
        
        if (len(landmarks)== 4):
                # assuming camera is mounted to the right and angled in
                src =np.array(landmarks, np.float32)# np.array([(0.0,100.0),(IMG_SIZE,-100.0),(IMG_SIZE,IMG_SIZE+ 100.0),(0.0,IMG_SIZE-100)], np.float32)
                print(src)
                dst = np.array(ideal_positions, np.float32 )#np.array([(0.0,0.0),(IMG_SIZE,0.0),(IMG_SIZE,IMG_SIZE),(0.0,IMG_SIZE)], np.float32)
                print(dst)
                perspective_xform = cv2.getPerspectiveTransform(src,dst)
                img = cv2.warpPerspective(img, perspective_xform, (IMG_SIZE,IMG_SIZE))
                #show(img, "shifted")
                cv2.imshow("cam", img)

        
                return (img, perspective_xform)
        else:
                cv2.imshow("cam", img)
                return None, None

''' creates the detector and trains it on features of the img '''
def create_detector(img):
        surfDetector = cv2.FeatureDetector_create("SURF")
        surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")
        
        imgg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
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
def detect_change(cur_img, knn, mismatch_threshold):
        DIST_THRESHOLD = 0.1

        surfDetector = cv2.FeatureDetector_create("SURF")
        surfDescriptorExtractor = cv2.DescriptorExtractor_create("SURF")

        cur_imgg = cv2.cvtColor(cur_img,cv2.COLOR_BGR2GRAY)
        cur_keypoints = surfDetector.detect(cur_imgg)
        cur_keypoints, cur_descriptors = surfDescriptorExtractor.compute(cur_imgg,cur_keypoints)

        mismatches = 0
         
        for h,des in enumerate(cur_descriptors):
                des = np.asmatrix(des,np.float32)
                retval, results, neigh_resp, dists = knn.find_nearest(des,1)
                res,dist =  int(results[0][0]),dists[0][0]

                if dist > DIST_THRESHOLD:
                        mismatches += 1
                        
        return mismatches > mismatch_threshold

''' some GUI code, adds the circle to the image and stores the x,y, and value entered int oa button list '''
def on_click(event, x, y,flag,param):
        global image, button_list
        if event == cv2.EVENT_LBUTTONUP:
                print("CLICK")
                print x,y
                cv2.circle(image, (x,y), CIRCLE_RADIUS, CIRCLE_COLOR, CIRCLE_THICKNESS)
                v = raw_input("Enter Button Value: ")
                
                print(v)    
                button_list.append((x,y,v))
                ''' TODO: add removing a button functionality in case you mis-click ''' 
        elif event == cv2.EVENT_RBUTTONUP:
                print("RIGHTCLICK")



def on_button_identified(event, x, y, flag, param):
	imagename, image, buttondict, buttonkey = param
	
	if event == cv2.EVENT_LBUTTONUP:
		print x,y
		scratchimage = np.zeros(image.shape, np.uint8)
		scratchimage[:] = image
		cv2.circle(scratchimage, (x,y), CIRCLE_RADIUS, PURPLE_COLOR, CIRCLE_THICKNESS)
		buttondict[str(buttonkey)]=(x,y)
		pprint.pprint(buttondict)
		cv2.imshow(imagename, scratchimage)

''' makes the window, displays the image, and adds the mouse callback to pick a button '''
def pick_button(image, name, buttondict, buttonname):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, image)
        cv2.setMouseCallback(name, on_button_identified, (name, image, buttondict, buttonname))

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


def get_calib_image():
        import time
        
        video_src = 0
        #cam = video.create_capture(video_src)
        cam = cv2.VideoCapture(0)
        cam.open(0)
        ret, frame = cam.read()
        cv2.namedWindow('cam')
        
        while True:
                err, vis = cam.read()
                #cv2.imshow('cam', vis)
                ch = 0xFF & cv2.waitKey(5)
                if ch == 27:
                        cv2.destroyAllWindows()
                        break
                if ch == ord('c'):
                        cv.SaveImage("calib.jpg", cv.fromarray(vis))
                if ch == ord('q'):
                        
                        image = cv2.resize(vis, (IMG_SIZE, IMG_SIZE), fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)
                        
                        #detector = create_detector(image)

                        # comment this in if your camera is not directly overhead, and tweak the src parameters
                        # the ones in there now are for about a 45-degree angle off to the right
                        image, perpsective_xform = perspective_shift(image)

                        ch = cv2.waitKey(500)
                        if ch == ord('x'):
                                
                
                    



def main(args):
        global display, image
        display = True

        get_calib_image()
        return 0

        if (len(args) < 2):
                img_name = "calib.jpg"
                find_chars = True
        else:
                img_name = args[1]
                if (len(args)>2 and args[2] == 'b'):
                    find_chars = False
                else:
                    find_chars = False
         
        image = cv2.resize(cv2.imread(img_name), (IMG_SIZE, IMG_SIZE), fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)

        detector = create_detector(image)

        # comment this in if your camera is not directly overhead, and tweak the src parameters
        # the ones in there now are for about a 45-degree angle off to the right
        image, perpsective_xform = perspective_shift(image)

        image2 =cv2.resize(cv2.imread("calib.jpg"), (IMG_SIZE,IMG_SIZE), fx=0.0, fy=0.0, interpolation=cv2.INTER_AREA)

        CHANGE_THRESHOLD = 100
        detect_change(image2, detector, CHANGE_THRESHOLD)
        return 0


        # run until ESCAPE is pressed
        #while True:
        #        pick_buttons()
        #        if cv2.waitKey(10)== 27:
        #                break

        buttons=define_buttons()
        pprint.pprint(buttons)
		
        cv2.destroyAllWindows()
		
        #nonmainstuff(image, find_chars)

def nonmainstuff(image, find_chars):
        (image, rotation_angle, shear_angle) = derotate_and_deshear(image)

        buttons = find_contours_MSER(image, 60, 14400, find_chars, (10, 1.5))
#     buttons = find_contours_FC(image, 200, 14400, find_chars, (10, 1.5))

        print(buttons)

        if (find_chars):
                all_symbols = []
                for button in buttons:
#     symbols = [(box[0] + button[0], box[1] + button[1], box[2], box[3]) for box in find_contours_MSER(img[button[1]: button[1] + button[3], button[0]:button[0] + button[2], ], 0, 500, True)]
#     all_symbols.extend(symbols)
                        all_symbols.append(button)
#     print(symbols)
#     print(all_symbols)


                tempimg = np.copy(image)
                for box in all_symbols:
                        print(box)
                        cv2.rectangle(tempimg, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 1)

                show(tempimg, "ALL SYMBOLS")

#   selected = np.zeros((CLIPSIZE,CLIPSIZE*len(buttons)))
#   for i in range(len(buttons)):
#     selected[0:CLIPSIZE,i*CLIPSIZE:(i+1)*CLIPSIZE] = buttons[i]
#
#   show(selected)
#   for i in range(len(buttons)):
#     show(buttons[i])

                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                canny = cv2.Canny(gray, 0, 50)

                block_size = 5
                param = 1

                thresh = gray
                erosion_size = 3
                kernel = cv2.getStructuringElement(cv2.MORPH_ERODE, (erosion_size, erosion_size))
                thresh = cv2.erode(thresh, kernel)
                print("SYMOBL COUNT:" + str(len(all_symbols)))

                def slice_and_resize(img, box):
                        clip = gray[box[1]:box[1] + box[3], box[0]:box[0] + box[2]]

                        w, h = box[2], box[3]

                        padded = np.copy(gray[0:CLIPSIZE, 0:CLIPSIZE])
                        padded[:, :] = 125

                        if (h > w):
                                scale_factor = CLIPSIZE / float(h)
                                clip = cv2.resize(clip, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
                                i = 0
                                for col in clip.T:
                                        padded[:, i] = col
                                        i += 1

                        else:
                                scale_factor = CLIPSIZE / float(w)
                                clip = cv2.resize(clip, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
                                i = 0
                                for row in clip:
                                        padded[i, :] = row
                                        i += 1
        #     show(padded, "PADDED")
                        return padded

                display = True
        #   thresh = cv2.adaptiveThreshold(thresh, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, param)
                symbol_images = [ slice_and_resize(gray, box) for box in all_symbols]
                #symbol_images = [ cv2.resize(gray[box[1]:box[1] + box[3], box[0]:box[0] + box[2]], (CLIPSIZE, CLIPSIZE), fx=0, fy=0, interpolation=cv2.INTER_NEAREST) for box in all_symbols]

        #     symbol_images = [cv2.erode(image, kernel) for image in symbol_images]
        #     symbol_images = [image - 2 * cv2.GaussianBlur(image, (5,5), sigmaX=0) for image in symbol_images]
                [show(image, "blank") for image in symbol_images]
        #     symbol_images =[cv2.resize(image, (CLIPSIZE,CLIPSIZE)) for image in symbol_images]
                #svm(symbol_images)


if __name__ == "__main__":
        main(sys.argv)
