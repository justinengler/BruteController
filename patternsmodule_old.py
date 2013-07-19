#########################################################
#
#         Brute Controller
#   Copyright Paul Vines, 2013
#
#  A more fully featured version will be available at
#  isecpartners.com after DEFCON
#
#######################################################

import numpy as np
import itertools
import random
import sys
import time
import re
import serial
import brutecontroller as bc

gridsize = (0,0)
buttonids={}
counter=0
FIRSTCHAR=ord('a')

def getgridsize():
	global gridsize
	print "Enter the size of the grid as NxM (e.g. 3x3, 4x2)"
	size = raw_input()
	gridsize = (int(size[0]), int(size[2]))
	print "Grid Size", gridsize

def robotconfigpattern(drop=None, lift=None, buttoncoords=None):
	"""Define all the values needed for a run of the robot (button coordinates, drop/lift, etc.)"""
	
	buttoncoords=buttonguesslayout(XOF1,YOF1, lift) if buttoncoords is None else buttoncoords
	
	bc.writedrop(DR if drop is None else drop)
	bc.writelift(LI if lift is None else lift)
	
	for k in buttoncoords.keys():
		writepoint(k,buttoncoords[k]['x'],buttoncoords[k]['y'],buttoncoords[k]['z'])
    

def patternguesslayout(xof1,yof1,lift):
	global gridsize
	print(gridsize)
	top = ((gridsize[0] -1) / 2)* yof1
	left = ((gridsize[1] -1) / 2) *yof1
	grid ={}
	for i in range(gridsize[0]):
		for j in range(gridsize[1]):
			grid[str(i) + str(j)] = {'x':(left - (yof1*j)), 'y':(top - (yof1*i)), 'z':lift}

	print grid
	return grid
	

def definepatterns(x,y,z):
	bc.move(x,y,z)
	print "PATTERN DEFINITION:"
	print " Keeping in mind the rough axis alignment, "
	print " put the device's center dot (or the dot up and left of center)"
	print "directly under the robot "
	print " press enter when the device is centered."
	raw_input()
	
	print "Now move the device to the dot directly above the previous dot"
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
	
	onekey=manualmovepattern(x,y,z)
	print onekey
	bc.move(x,y,z)
	layout=patternguesslayout(onekey[0],onekey[1], z)
	return layout


def manualmovepattern(x,y,z,stepsize=.1, layout=None, fix=False):
	newx=x
	newy=y
	newz=z
	
	curr=""
	stopchar='k'
	
	while True:
		bc.move(newx,newy,newz)
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
				layout[curr[1:]]['x']=newx
				layout[curr[1:]]['y']=newy
				layout[curr[1:]]['z']=newz

def brutekeys(pinlength, randomorder=False):
	global gridsize

	keys = [str(x) for x in range(gridsize[0] * gridsize[1])]
	
	allpossible = list(itertools.imap(lambda x: "".join(x),itertools.permutations(keys, pinlength)))
	if randomorder:
		random.shuffle(allpossible)

	return allpossible


def brutelooppattern(brutelist, maxtries=None, actionlist=()):
	if maxtries is None:
		maxtries=sys.maxint
	
	tries=0
	persister=None
	brutecontinue=True
	
	for pin in brutelist:
		print "===Pushing %s:"%(pin,)
		for i,number in enumerate(pin):
			if i == 0:
				bc.push(toIdentifier(str(int(number)/gridsize[1])+ str(int(number)% gridsize[1])))
			else:
				bc.draw(toIdentifier (str(int(number)/gridsize[1])+ str(int(number)% gridsize[1])))
		bc.move(0,0,0)
		tries+=1
		for modulo,func in actionlist:
			if tries % modulo == 0:			
				returnvalue=func(tries,pin,persister)
				if returnvalue is not None:
					brutecontinue, persister = returnvalue
		if tries>=maxtries or not brutecontinue:
			break
			
	bc.move(0,0,0)
	
def writepoint(ident, x,y,z):
	"""SETUP: Define a new button with coordinates (x,y,z) and the letter ident associated with that button"""
	bc.write("WP %s X%s Y%s Z%s;"%(toIdentifier(ident),
								x,y,z))


def toIdentifier(target):
	"""Translates a logical button identifier (1, 2, etc.) to its robot-understood identifier
	(As defined by writepoint()/WP)
	"""
	global counter
	
	if target not in buttonids:
		buttonids[target]=counter
		counter+=1
	#print "toIdentifier: %s : %s"%(target,buttonids)
	return chr(int(buttonids[target])+FIRSTCHAR)

def main():
	global writedelay
	print "Brute Controller"
	print "\t This is prerelease code.  A better version will be available after DEFCON at:"
	print "\t\twww.isecpartners.com"
	print "\tYou will probably need to edit this file before it works."
	print "\tSpecfically, you might need to adjust the serial port"
	print "\tThe easiest way to discover which serial port your Arduino is on "
	print "\tis to check the Arduino software's menu"
	print "\tYou will also probably want to make some edits to the main function"
	bc.serialsetup()
	writedelay=.2
	print "\n\n"
	print "+="*10+" Configure Robot "+"+="*10
	print "\n\n"
	getgridsize()
	
	brutes=brutekeys(4, randomorder=True)
	drop=bc.finddrop()
	lift=drop
	bc.findaxes(x=0,y=0, z=lift)
	layout=definepatterns(x=0,y=0,z=lift)

	print "everything's set!  To check calibration, type numbers."
	print "to correct, use WSADQE to move head to correct location for a key," 
	print "then type 'F#' where # is the key to fix."
	print "k to begin brute"
	manualmovepattern(x=0,y=0,z=lift, layout=layout, fix=True)
	print "Calibrating robot"
	robotconfigpattern(drop, lift, layout)	
	
	writedelay=.5
	print "*="*5+"test all 4 digit pins, random order"
	brutelooppattern(brutes,maxtries=10)
	
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
