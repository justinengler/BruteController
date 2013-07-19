#########################################################
#
#         Brute Controller
#   Copyright Justin Engler, 2013
#
#  A more fully featured version will be available at
#  isecpartners.com after DEFCON
#
#######################################################

import pickle
import itertools
import random
import sys
import time
import re
import serial
#import serial.tools.list_ports

ser = None

FIRSTCHAR=ord('a')

XOF1=1.8
YOF1=.8
DR=1.75
LI=0;

global_layout = None
writedelay=.5
buttonids={}
counter=0

"""If True, the input and output to serial are shown on the console"""
SERIALTOCONSOLE=True

def serialsetup():
	global ser
	ser = serial.Serial('COM4', 57600, timeout=1)
	ser.flushInput()
	ser.flushOutput()
	readuntil(ser,'>')
	
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
	
	

#assumes grid layout, even spacing
def buttonguesslayout(xof1,yof1):
	return {
				'0':{'x':0,	'y':-(2*yof1), 	'z':0},
				'1':{'x':xof1,'y':yof1, 	'z':0},
				'2':{'x':0,	'y':yof1, 		'z':0},
				'3':{'x':-xof1,'y':yof1, 	'z':0},
				'4':{'x':xof1,'y':0, 		'z':0},
				'5':{'x':0,	'y':0, 			'z':0},
				'6':{'x':-xof1,'y':0, 		'z':0},
				'7':{'x':xof1,'y':-yof1, 	'z':0},
				'8':{'x':0,	'y':-yof1, 		'z':0},
				'9':{'x':-xof1,'y':-yof1, 	'z':0},
				'Z': {'x':-xof1,'y':-(2*yof1), 'z':0},
				'X': {'x':0,'y':-(.5*yof1),'z':0}
				}		

def write(output):
	"""Send output to the robot"""
	if SERIALTOCONSOLE:
		print "++TOSERIAL:%s"%output
	ser.write(output)
	time.sleep(writedelay)
	fromserial=readuntil(ser,'>')
	if SERIALTOCONSOLE:
		print "--FROMSERIAL:%s"%fromserial



def pointover(button):
	"""Position the effector above a given button"""
	write("PO %s;"%button)

""" Couldn't get the EP command to work with 5 sets of coordinates - too much in one serial send?"""
def enterpin(pin):
	global global_layout
#	command = "EP&"
	for number in pin:
		coordinate = global_layout[str(number)]
#	command += " X" + str(coordinate['x'] ) + " Y" + str(coordinate['y']) + " Z" + str(coordinate['z'])+ " &"
		move(coordinate['x'], coordinate['y'], coordinate['z'])
		time.sleep(writedelay)
		
	coordinate = global_layout[str('Z')]
	#command += " X" + str(coordinate['x'] ) + " Y" + str(coordinate['y']) + " Z" + str(coordinate['z'])+ ";"
	move(coordinate['x'], coordinate['y'], coordinate['z'])
	time.sleep(writedelay)
	move(0,0,0)
	time.sleep(writedelay)
			
	#print command
	#write(command)

def push(button):
	"""Push a given button"""	
	write("PU %s;"%button)

def move(x,y,z):
	"""Move to the coordinates given"""
	write("MV X%s Y%s Z%s;"%(x,y,z))
	
	
def draw(button):
	"""Move the robot to the given button without lifting the effector"""
	write("DR %s;"%button)
	
def writepoint(ident, x,y,z):
	"""SETUP: Define a new button with coordinates (x,y,z) and the letter ident associated with that button"""
	write("WP %s X%s Y%s Z%s;"%(toIdentifier(ident),
								x,y,z))
									
def writedrop(z):
	"""SETUP: Define the z coordinate of a button that is "pushed"""
	write("WD %s;"%z)

def writelift(z):
	"""SETUP: Define the z coordinate where the effector is not pushing a button"""
	write("WL %s;"%z)


def toIdentifier(target):
	"""Translates a logical button identifier (1, 2, etc.) to its robot-understood identifier
	(As defined by writepoint()/WP)
	"""
	global counter
	
	#if not target.isalnum():	
	#	print "--Could not convert to button: '%s'"%target
	#	return
	
	if target not in buttonids:
		buttonids[target]=counter
		counter+=1
	#print "toIdentifier: %s : %s"%(target,buttonids)
	
	return chr(int(buttonids[target])+FIRSTCHAR)


def robotconfig(drop=None, lift=None, buttoncoords=None):
	"""Define all the values needed for a run of the robot (button coordinates, drop/lift, etc.)"""
	
	buttoncoords=buttonguesslayout(XOF1,YOF1) if buttoncoords is None else buttoncoords
	
	writedrop(DR if drop is None else drop)
	writelift(LI if lift is None else lift)
	
	for k in buttoncoords.keys():
		print k, buttoncoords[k]
		writepoint(k,buttoncoords[k]['x'],buttoncoords[k]['y'],buttoncoords[k]['z'])

def finddrop(x=0, y=0, z=0, usebutton=False):
	"""Slowly lower the effector until a button is pressed.  Button can be hardware linked
		to Arduino, or the user can press the keyboard until the touch device registers a touch"""
	x=0
	y=0
	z=0	
	stepsize=.1
	
	if not usebutton:
		print "DEPTH DETECTION"
		print " Place the device to be bruted under the robot."
		print " Device surface needs to be no more than a few centimeters below robot"
		print " Keep pressing Enter until the robot touches the screen."
		print " Type u to go up a step."
		print " WARNING:  DO NOT HOLD! WAIT FOR ROBOT! Don't break your screen!"
		print " Type s and enter to save the z location"
	
		curr=""
		stopchar="s"
		while True:
			move(x,y,z)
			print z
			curr=raw_input()
			if stopchar in curr:
				break;
			elif "u" in curr:
				z-=stepsize
			else:
				z+=stepsize
	
	else:
		print "NOT IMPLEMENTED"
		MakeArduinoFindZ()
	
	print "z was: ",z
	
	move (x,y,0)
	return z
		
def findaxes(x,y,z):
	print "AXIS ALIGNMENT:"
	print " Keep pressing Enter to sweep the effector in the X and Y directions."
	print " Type x or y to sweep only that, or type a for all directions"
	print " Align the device's direction with the robot's sweep,"
	print " (      Y     )"
	print " (            )"
	print " (    1 2 3   )"
	print " (-X  4 5 6  X)"
	print " (    7 8 9   )"
	print " (            )"
	print " (     -Y     )"
	print " Alignment need not be perfect at this point."
	print " Type s and press enter when the device is aligned."
	curr=raw_input()
	
	newx=x+2
	newy=y+2
	stopchar='s'
	lastmoved='y'
	move (x,y,z)
	while True:
		move(newx,newy,z)
		print "X, Y = %s, %s"%(newx,newy)

		curr=raw_input()
		if stopchar in curr:
			break;
		else:
			if "x" in curr:
				newx=-newx
				lastmoved='x'
			elif "y" in curr:
				newy=-newy
				lastmoved='y'
			else:
				if lastmoved=='x':
					newx=-newx
					lastmoved='x'
				else:
					newy=-newy
					lastmoved='y'
	print "DONE!"
	move (x,y,0)
	
def definebuttons(x,y,z):
	move(x,y,z)
	print "BUTTON DEFINITION:"
	print " Keeping in mind the rough axis alignment, "
	print " put the device's center key (usually 5) under the robot"
	print " press enter when the device is centered."
	raw_input()
	
	print "Now move the device over the top left key (usually 1, sometimes 7)"
	print "type w, s, a, or d and enter to move the device"
	print "Y, -Y, -X, or X respectively."
	print " (      W     )"
	print " (            )"
	print " (    1 2 3   )"
	print " (A   4 5 6  D)"
	print " (    7 8 9   )"
	print " (            )"
	print " (      S     )"
	print " IMPORTANT:  Needs to be precise and centered!"
	print "Type k and press enter when finished"
	
	onekey=manualmove(x,y,z)
	print onekey
	move(x,y,z)
	layout=buttonguesslayout(onekey[0],onekey[1])
	return layout
		
""" ADDED 7-18: 'o' loads layout.p, 'p' saves the current layout to
layout.p, Z is the "OK" button, X is the "OK" button for the cooldown screen"""
def manualmove(x,y,z,stepsize=.1, layout=None, fix=False):
	global global_layout
	newx=x
	newy=y
	newz=z
	
	curr=""
	stopchar='k'
	
	while True:
		move(newx,newy,newz)
		print "X, Y, Z = %s, %s, %s"%(newx,newy,newz)
		curr=raw_input()
		if curr=='w':
			newy+=stepsize
		elif curr=='s':
			newy-=stepsize
		elif curr=='a':
			newx-=stepsize
		elif curr=='d':
			newx+=stepsize
		elif curr=='q':
			newz+=stepsize
		elif curr=='e':
			newz-=stepsize
		elif curr=='o':
			global_layout = pickle.load(open("layout.p", 'r'))
			layout = global_layout
			print layout
		elif curr=='p':
			pickle.dump(layout, open("layout.p", 'w'))
		elif curr=='k':
			return (newx, newy, newz)
		elif layout is not None:
			if curr in layout:
				#pointover(curr)
				print "Point ", curr
				newx=layout[curr]['x']
				newy=layout[curr]['y']
				newz=layout[curr]['z']
			if curr=='p':
				print layout
			if fix and len(curr)> 0 and curr[0]=='F':
				layout[curr[1]]['x']=newx
				layout[curr[1]]['y']=newy
				layout[curr[1]]['z']=newz
	
		global_layout= layout
	
def bruteloop(brutelist, maxtries=None, actionlist=()):
	global global_layout
	cooldown_ok = global_layout['X']
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
	start =True
	
	for pin in brutelist:
		print "===Pushing %s:"%(pin,)
		if start:
			#for number in pin:
				#print "pushing %s"%number
				#push(toIdentifier(number))
			enterpin(pin)
			
			#push(toIdentifier('Z'))
			tries+=1
			if (tries == 5):
				move(0,0,1)
				
				
				break;
			
			if (tries % 5 == 0):
				move(0,0,0)
				time.sleep(1)
				move(cooldown_ok['x'], cooldown_ok['y'], cooldown_ok['z'])
				move(0,0,0)
				androidPINwait(0,0,0)
			
			for modulo,func in actionlist:
				if tries % modulo == 0:			
					returnvalue=func(tries,pin,persister)
					if returnvalue is not None:
						brutecontinue, persister = returnvalue
			if tries>=maxtries or not brutecontinue:
				break
				
	move(0,0,0)
			

def brutekeys(pinlength, keys="0123456789", randomorder=False):
	"""
	Returns a list of all possibilities to try, based on the length of PIN and buttons given.
	
	Yeah, lots of slow list copying here, but who cares, it's dwarfed by the actual guessing.
	"""
	allpossible = list(itertools.imap(lambda x: "".join(x),itertools.product(keys, repeat=pinlength)))
	if randomorder:
		random.shuffle(allpossible)


	allpossible = ("1234", "0000", "2580", "1111", "5555")
	return allpossible


def loadlist(filename):
	"""Returns an iterable of possible keys as loaded from a file"""
	f=open(filename,"r")
	return f


def fixdigits(pinlist, fixlist):
	"""Reduces a pinlist to a smaller set based on fixlist.
		Fixlist should be a tuple with the same length as each PIN in pinlist
		If the number in that position doesn't matter, set it to None.
		Otherwise, the list will be pared down to any items that match.
		
		Returns the new, reduced list, original is unchanged
	"""
	newlist=list()
	for pin in pinlist:
		keep=True
		for i in range(0,len(pin)):
			if fixlist[i] is not None and pin[i] not in fixlist[i]:
				keep=False
				break

		if keep:
			newlist.append(pin)
	return newlist


def fixbyregex(pinlist, regex):
	"""Reduces a pinlist to a smaller set based on regex.
		returns the new, reduced list, original is unchanged.
		
		NOTE: If you pass this a non-string pinlist, it won't work.
	"""
	newlist=list()
	for pin in pinlist:
		keep=True
		if re.match(regex,pin) is not None:
			newlist.append(pin)
	return newlist


def androidPINwait(guessnum, PIN, persistdata):
	"""For use in an actionlist as part of a brute attempt (see bruteloop)
		Android has a 30 second delay every n bad guesses, this function
		accounts for that
	"""
	#TODO:Push buttons here
	print "Sleeping 30 seconds"
	time.sleep(30)

def dance():
	while True:
		move(2,2,0)
		#move(2,1,0)
		move(2,0,0)
		#move(2,-1,0)
		move(2,-2,0)
		#move(1,-2,0)
		move(0,-2,0)
		#move(-1,-2,0)
		move(-2,-2,0)
		#move(-2,-1,0)
		move(-2,0,0)
		#move(-2,1,0)
		move(-2,2,0)
		#move(-1,2,0)
		move(0,2,0)
		#move(1,2,0)
		
			

def main():
	global writedelay
	global global_layout
	print "Brute Controller"
	print "\t This is prerelease code.  A better version will be available after DEFCON at:"
	print "\t\twww.isecpartners.com"
	print "\tYou will probably need to edit this file before it works."
	print "\tSpecfically, you might need to adjust the serial port"
	print "\tThe easiest way to discover which serial port your Arduino is on "
	print "\tis to check the Arduino software's menu"
	print "\tYou will also probably want to make some edits to the main function"
	serialsetup()
	writedelay=.2
	print "\n\n"
	print "+="*10+" Configure Robot "+"+="*10
	print "\n\n"
#	drop=finddrop()
#	lift=drop
#	findaxes(x=0,y=0, z=lift)
	lift = 0
	layout=definebuttons(x=0,y=0,z=lift)

	print "everything's set!  To check calibration, type numbers."
	print "to correct, use WSADQE to move head to correct location for a key," 
	print "then type 'F#' where # is the key to fix."
	print "o to load a configuration from the pickle file \"layout.p\""
	print "k to begin brute"
	manualmove(x=0,y=0,z=lift, layout=layout, fix=True)
	if not global_layout is None:
		layout = global_layout
	print "Calibrating robot"
	print layout
	#robotconfig(drop=0, lift=lift, buttoncoords=layout)	
	
	writedelay=.15
		
	print "*="*5+"test all 4 digit pins, random order"
	brutes=brutekeys(4, randomorder=False)
	bruteloop(brutes,maxtries=10000)
	
	"""
	print "*="*5+"test all 4 digit pins ending in 99[98]"
	brutes=brutekeys(4)
	print len(brutes)
	fixedbrute = fixdigits(brutes,(None,('9',),('9',),('9','8')))
	print fixedbrute 
	print len(fixedbrute)
	"""
	
	"""
	print "*="*5+"test all 4 digit pins ending in 44[45] (regex)"
	brutes=brutekeys(4)
	print len(brutes)
	fixedbrute = fixbyregex(brutes,".44[45]")
	print fixedbrute 
	print len(fixedbrute)
	"""
	
	"""
	#print "*="*5+"test load pins from a file, pause every 5 guesses"
	#print "load from file"
	#bruteloop(loadlist("testPINS.txt"), actionlist=((5,androidPINwait),))
	"""
	
	ser.flushInput()
	ser.flushOutput()
	ser.close()
	
if __name__=="__main__":
	main()
