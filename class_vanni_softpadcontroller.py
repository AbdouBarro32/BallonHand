#!/usr/bin/env python

#We use this script to measure the height of the mini softpads at a certain pressure

import math
import sys
import time
import rospy
from pneumaticbox import api, utils, io, setups
import numpy as np
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Float32 
from numpy import mean
import random
import os
import signal
from pneumaticbox.api import *
'''
import pneumaticbox.io as io
import pneumaticbox.api as api
'''

try:
    import getch
except ImportError:
    print ("Please install 'getch' for interactive keyboard control (https://pypi.python.org/pypi/getch)")
    pass
    
from pneumaticbox import utils
airserver = utils.connectToDefaultPneumaticbox(receiveInBackground=True)

#setup the default threshold controllers
print "Configuring and activating Threshold controllers in default configuration."
msgs_config=[]
for i in range(8):
    msgs_config.append(MsgConfigurationControllerThreshold(i=i))#, offset=0))
    msgs_config.append(MsgControllerActivate(i))
airserver.submit(msgs_config)
time.sleep(0.05) 
delay=1
signals_buffer=[] 
# 6 channels
#duration=[0.01,0.01,0.01,0.01,0.01,0.01] 
#duration_deflating=5.0
msgs_channels_inflate=[[],[],[],[],[],[]]



#this function makes sure that when killing the script, the channels get deflated
def _on_exit(signum, frame):
    print("Script killed, deflating hand.")
   # it_msgs = deflate(8.0,True,0,delay)
   # airserver.submit(it_msgs)
    sys.exit()                      

def printKeys():
    print "~~~~~~~~~~~~~~~~~~~~~~~~~~~"
    print "Available controls:"
    print "  d: deflate all"
    print "  h: inflate all to 2 kPa"
    print "  l: inflate all to 1.5 kPa"
    print "  a: actuation"
    print "  +: inflate all"
    print "  -: deflate all"
    print "  [1-6]: adjust one pad"
    print "  q: quit"
    print "  f: automatic adjust"
    print "  x: translation along the x-axis"
    print "  c: controll all"
    print "  e: controll each finger cycle"
    print "~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"

	
	
def actuation():
	print("Actuation")


def deflate(duration_deflating, deflate_all,number, delay=0.0):
	if deflate_all==True:
		print ("Deflating all with duration:", duration_deflating)
		for i in range(6):
			yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + i, -1.0, delay      )
			yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + i,  0.0, delay + duration_deflating )
	else:
		print ("Deflating",number," with duration:", duration_deflating)
		yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + number-1, -1.0, delay      )
		yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + number-1,  0.0, delay + duration_deflating )
		

def inflate(duration, inflate_all,number, delay=0.0):
	if inflate_all==True:
		print ("Inflating all with duration:", duration)
		for i in range(6):
			yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + i, 1.0, delay      )
			yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + i,  0.0, delay + duration[i] )
	else:
		print ("Inflating",number," with duration:", duration)
		yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + number-1, 1.0, delay      )
		yield MsgSignalEvent(BLOCK_SIGNALS_CLIENT + number-1,  0.0, delay + duration )

def automatic():
	#rospy.init_node('inflating', anonymous=True)
#	time.sleep(1)
#	listener()
	
#def listener():
	global sub
	sub=rospy.Subscriber("/pneumaticbox/pressures",Float64MultiArray, callback)
	print("sono qui")
	rospy.spin()
	
	
def callback(data): 
	#rospy.loginfo(rospy.get_caller_id())
	#print("I heard", data.data)
	diff=[]
	s=data.data # here there are pressure measurements
	file1=open("pressures_height_mini.txt","a")
	file1.write(';'.join(str(e) for e in s))
	file1.write('\n')
	file1.close()
	
	#print('flag',flag)
	if 'flag' in globals():
		print('yes')
		print('flag',flag)
		if (flag==0):
			check_differences(s)
		if (flag==1):
			adjustments(s,diff)
			'''
		if (flag==2):
		#rospy.signal_shutdown('ciaoo')
			time.sleep(3)
			sys.exit(keyboardControl())
			#flag=0
			'''
	else:
		print('no')
		global flag
		check_differences(s)
		flag=0
	


def check_differences(signals___):
	global signals_buffer
	global goal,flag,sub
	c=len(signals_buffer)
	#print('c',c) 
	if c==0:
		signals_buffer=signals___
		counter_readings=1
		flag=0
	elif c==10:
		signals=tuple(mean(signals_buffer,axis=0))
		#print('Pressures: ',signals)
		counter_readings=0
		signals_buffer=[]
		#goal=mean(signals)
		#goal=2
		flag=1
		print('goal ',goal)
		
		
	else:
		signals_buffer=np.row_stack((signals_buffer,signals___))
		counter_readings=len(signals_buffer)
		flag=0
	print("iteration number",counter_readings)
	print('flag interna',flag)
		         
		         
		         
		         
def adjustments(ss,diff):
	flag=2
	initial_pad = 1
	final_pad = 5
	print('signals',ss)
	for i in range(initial_pad,final_pad+1): #va rimesso da 1 a 7
		print('signal',ss[i-1])
		diff=np.append(diff,ss[i-1]-goal)
		#alfa_infl=-8.258+(9.282*signals[i-1])
		alfa_infl=300
		alfa_defl=200
		print('diff',diff)
		
		# Thresholds for the automatic adjustements:
		th_plus = 10
		th_minus = -0.8
		if diff[i-1]>th_plus:
			#print('Do you want to deflate', i, 'for ', diff[i-1]/alfa_defl, 'seconds?')
			#raw_input('Press Enter to change pressures')
			#if (i!=8):
				it_msgs = deflate(diff[i-1]/alfa_defl,False,i,delay)
				airserver.submit(it_msgs)
				flag=1
				
		elif diff[i-1]<th_minus:
			#print('Do you want to inflate',i,'  for ', -diff[i-1]/alfa_infl , 'seconds?')
			#raw_input('Press Enter to change pressures')
			#if (i!=8):
				if -diff[i-1]/alfa_infl<0.02:
					it_msgs = inflate(-diff[i-1]/alfa_infl,False,i,delay)
					airserver.submit(it_msgs)
					flag=1
	print('flag tarocca',flag)
	if (flag==1):
		sub.unregister()
		time.sleep(3)
		print('sono quooooo')
		automatic()
	if (flag==2):
		sub.unregister()
		time.sleep(3)
		#sys.exit(keyboardControl())
		keyboardControl()
	
class CustomController:
    def __init__(self):
        
        self.s1 = None
        self.cycle_flag = 0
        self.count_time = 0
        self.flag = [0, 0, 0, 0, 0, 0]
        self.adjust_N1 = [0.015, 0.001, 0.001, 0.001, 0.001, 0.001]
        self.adjust_N2 = [0.001, 0.015, 0.001, 0.001, 0.001, 0.001]
        self.adjust_N3 = [0.001, 0.001, 0.015, 0.001, 0.001, 0.001]
        self.adjust_N4 = [0.001, 0.001, 0.001, 0.015, 0.001, 0.001]
        self.adjust_N5 = [0.001, 0.001, 0.001, 0.001, 0.015, 0.001]
        self.adjust_N6 = [0.001, 0.001, 0.001, 0.001, 0.001, 0.015]
        
        
    
    def initialize_all(self):
        self.sub_1 = rospy.Subscriber("/pneumaticbox/pressures",Float64MultiArray, self.get_state)
        self.timer = rospy.Timer(rospy.Duration(1.0),self.adjust_all)
        rospy.spin()
    
    def initialize_each(self):
        self.sub_1 = rospy.Subscriber("/pneumaticbox/pressures",Float64MultiArray, self.get_state)
        self.timer = rospy.Timer(rospy.Duration(1.0),self.adjust_each)
        rospy.spin()
    
    
    def get_state(self, data):
        self.s1 = data.data
    
    
    def adjust_each(self, timer_event):       ########## make a cycle for each of the three fingers  
        self.count_time = self.count_time + 1
        if self.count_time == 25:
            self.cycle_flag = self.cycle_flag + 1
            self.cycle_flag = self.cycle_flag % 3
            self.count_time = 0
            
            if self.cycle_flag == 0:
                
                it_msgs_ = deflate(1, False, 5, delay)
                airserver.submit(it_msgs_)
                it_msgs_ = deflate(1, False, 6, delay)
                airserver.submit(it_msgs_)
                time.sleep(0.3)
                print("flag == 0")
            
            elif self.cycle_flag == 1:
                
                it_msgs_ = deflate(1, False, 1, delay)
                airserver.submit(it_msgs_)
                it_msgs_ = deflate(1, False, 2, delay)
                airserver.submit(it_msgs_)
                time.sleep(0.3)
                print("flag == 1")
                
            elif self.cycle_flag == 2:
                
                it_msgs_ = deflate(1, False, 3, delay)
                airserver.submit(it_msgs_)
                it_msgs_ = deflate(1, False, 4, delay)
                airserver.submit(it_msgs_)
                time.sleep(0.3)
                print("flag == 2")
                    
                
        
        if self.cycle_flag == 0:
            if self.s1[0]<50:
                self.flag[0] = self.flag[0]+1
                self.flag[0] = self.flag[0]%2
                if self.flag[0] == 0:
                    print(self.s1[0])
                    it_msgs = inflate(self.adjust_N1,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                
            if self.s1[1]<50:
                self.flag[1] = self.flag[1]+1
                self.flag[1] = self.flag[1]%2
                if self.flag[1] == 0:
                    it_msgs = inflate(self.adjust_N2,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                    
        elif self.cycle_flag == 1:
            if self.s1[2]<100:
                self.flag[2] = self.flag[2]+1
                self.flag[2] = self.flag[2]%2
                if self.flag[2] == 0:
                    it_msgs = inflate(self.adjust_N3,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                    
            if self.s1[3]<100:
                self.flag[3] = self.flag[3]+1
                self.flag[3] = self.flag[3]%2
                if self.flag[3] == 0:
                    it_msgs = inflate(self.adjust_N4,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
        
        elif self.cycle_flag == 2:
            if self.s1[4]<100:
                self.flag[4] = self.flag[4]+1
                self.flag[4] = self.flag[4]%2
                if self.flag[4] == 0:
                    it_msgs = inflate(self.adjust_N5,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)

            if self.s1[5]<100:
                self.flag[5] = self.flag[5]+1
                self.flag[5] = self.flag[5]%2
                if self.flag[5] == 0:
                    it_msgs = inflate(self.adjust_N6,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
            
               
        
    
    
    
    
    
    def adjust_all(self, timer_event):
        if self.s1 is not None:
            
            if self.s1[0]<50:
                self.flag[0] = self.flag[0]+1
                self.flag[0] = self.flag[0]%2
                if self.flag[0] == 0:
                    print(self.s1[0])
                    it_msgs = inflate(self.adjust_N1,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
               
            if self.s1[1]<50:
                self.flag[1] = self.flag[1]+1
                self.flag[1] = self.flag[1]%2
                if self.flag[1] == 0:
                    it_msgs = inflate(self.adjust_N2,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                    
            if self.s1[2]<100:
                self.flag[2] = self.flag[2]+1
                self.flag[2] = self.flag[2]%2
                if self.flag[2] == 0:
                    it_msgs = inflate(self.adjust_N3,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                    
            if self.s1[3]<100:
                self.flag[3] = self.flag[3]+1
                self.flag[3] = self.flag[3]%2
                if self.flag[3] == 0:
                    it_msgs = inflate(self.adjust_N4,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)

            if self.s1[4]<100:
                self.flag[4] = self.flag[4]+1
                self.flag[4] = self.flag[4]%2
                if self.flag[4] == 0:
                    it_msgs = inflate(self.adjust_N5,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)

            if self.s1[5]<100:
                self.flag[5] = self.flag[5]+1
                self.flag[5] = self.flag[5]%2
                if self.flag[5] == 0:
                    it_msgs = inflate(self.adjust_N6,True,0,delay)
                    airserver.submit(it_msgs)
                    time.sleep(0.3)
                    
                    
        self.print_all()		
        
    def print_all(self):
        for idx in range(6):
            print("pressione ", idx+1, " = ", self.s1[idx])
	

    
    
	
        
	
def keyboardControl():
	channel_ID = 0
	mass_desired_counter = 0 #mg
	global airserver
	global nominal_volume_ccm
	global duration
	global goal 
	printKeys()
	try: 
		while not rospy.is_shutdown():
			print("\n~~~~~~~~~~......~~~~~~~~~~~~~")
			char = getch.getch()
			if char == 'q':
				print "exit interactive mode"
				rospy.signal_shutdown('ciaoo')
			elif char == 'd':
				print "Deflate all!"
				duration_deflating=3.0
				it_msgs = deflate(duration_deflating,True,0,delay)
				airserver.submit(it_msgs)
			elif char == 'h':
				print ("Inflate all high pressure!")
				duration=[0.4,0.2,0.4,0.4,0.4,0.4]
				#duration=[0.8,0.8,0.8,0.8,0.8,0.8]
				#duration=[0.05,0.05,0.05,0.05,0.05,0.05]
				#goal=2.3
				#goal=1.8
				it_msgs = inflate(duration,True,0,delay)
				airserver.submit(it_msgs)
			elif char == 'l': 
				print "Inflate all low pressure!"
				# Pads are inflated for the following time :
				duration=[0.03,0.021,0.021,0.022,0.018,0.021]
				# Goal pressure for the automatic adjustement:
				goal=10 # kPa
				it_msgs = inflate(duration,True,0,delay)
				airserver.submit(it_msgs)
			elif char == 'x': 
				print "Translation along the x axis"
				for i in range(10):
					# Deflation pads finger 1:
					it_msgs = deflate(0.5,False,2,delay)
					airserver.submit(it_msgs)
					it_msgs = deflate(0.5,False,3,delay)
					airserver.submit(it_msgs)
					# Inflation pads finger 2:
					it_msgs = inflate(0.022,False,4,delay)
					airserver.submit(it_msgs)
					it_msgs = inflate(0.0165,False,5,delay)
					airserver.submit(it_msgs)
					#~ raw_input("Press Enter to continue")
					time.sleep(1.5)
					# Inflation pads finger 1:
					it_msgs = inflate(0.025,False,2,delay)
					airserver.submit(it_msgs)
					it_msgs = inflate(0.025,False,3,delay)
					airserver.submit(it_msgs)
					# Deflation pads finger 2:
					it_msgs = deflate(0.5,False,4,delay)
					airserver.submit(it_msgs)
					it_msgs = deflate(0.5,False,5,delay)
					airserver.submit(it_msgs)
					time.sleep(1.5)
			elif char == 'z': 
				print "Translation along the z axis"
				for i in range(10):
					# Deflation lower pads:
					it_msgs = deflate(0.5,False,3,delay)
					airserver.submit(it_msgs)
					it_msgs = deflate(0.5,False,4,delay)
					airserver.submit(it_msgs)
					# Inflation upper pads:
					it_msgs = inflate(0.022,False,2,delay)
					airserver.submit(it_msgs)
					it_msgs = inflate(0.0165,False,5,delay)
					airserver.submit(it_msgs)
					#~ raw_input("Press Enter to continue")
					time.sleep(1.5)
					# Inflation lower pads:
					it_msgs = inflate(0.025,False,3,delay)
					airserver.submit(it_msgs)
					it_msgs = inflate(0.025,False,4,delay)
					airserver.submit(it_msgs)
					# Deflation upper pads:
					it_msgs = deflate(0.5,False,2,delay)
					airserver.submit(it_msgs)
					it_msgs = deflate(0.5,False,5,delay)
					airserver.submit(it_msgs)
					time.sleep(5)
			elif char == u'+':
				duration=[0.03,0.01,0.01,0.01,0.01,0.01]
				it_msgs = inflate(duration,True,0,delay)
				airserver.submit(it_msgs)
			elif char == u'-':
				duration_deflating=0.01
				it_msgs = deflate(duration_deflating,True,0,delay)
				airserver.submit(it_msgs)
			elif char == 'a':
				print "Actuation!"
				actuation()
    
			elif char == "c":
				custom_controller = CustomController()
				custom_controller.initialize_all()
				
			elif char == "e":
				custom_controller = CustomController()
				custom_controller.initialize_each()
               
                
			elif char == 'f':
				goal=15.68
				print "Automatic!"
				automatic()
			elif char in [str(i) for i in np.arange(7)]:
				print ("Adjust pressure in pad", int(char))
				print ("Press + or -")
				char_ = getch.getch()
				if char_ == u'+':
					#duration=0.05
					duration=0.02
					it_msgs = inflate(duration,False,int(char),delay)
					airserver.submit(it_msgs)
				elif char_ == u'-':
					#duration_deflating=0.05
					duration_deflating=0.008
					it_msgs = deflate(duration_deflating,False,int(char),delay)
					airserver.submit(it_msgs)
				else:	
					print ("ciao")
				
			else:
				print "unknown command <%r>"%char
		return
	except NameError as e:
		print "({})".format(e)
		return

    

    
def main(args):
    return 0
    

if __name__ == '__main__':
	signal.signal(signal.SIGINT, _on_exit) #make sure to always deflate if script gets killed
	rospy.init_node('inflating', anonymous=True,disable_signals=False)
	keyboardControl()
	duration_deflating=3.0
	it_msgs = deflate(duration_deflating,True,0,delay)
	airserver.submit(it_msgs)
	#rospy.signal_shutdown('ciaoo')
	#print('sono qua')
	sys.exit(main(sys.argv))
	
	













          




