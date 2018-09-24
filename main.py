#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from parameter import *
import time

import sys
import signal
import math

t_begin_program = time.time()

Continue = True
while Continue:
	mode = input("Quit: q ; Continue: c\n$ ")
	if (mode == "q" or mode == "Q"):
		Continue = False	
	else:
		go_robot_go(myI2cDev, myLoc, 50)
		

if edit_file:
	my_file.close()

myLoc.finish()
myI2cDev.finish()
del myLoc
del myI2cDev
time.sleep(0.1)
sys.exit()
