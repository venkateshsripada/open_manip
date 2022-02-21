#!/usr/bin/env python

import sys
import wx
import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from darknet_ros.msg import coordinates

class MyFrame(wx.Frame):
	def __init__(self):
		super(MyFrame, self).__init__(parent = None, title = 'Hello World', size = (700, 450))
		rospy.init_node('gui_publish', anonymous = True)

		panel = wx.Panel(self)
		# my_sizer = wx.BoxSizer(wx.VERTICAL)

		# self.text_ctrl = wx.TextCtrl(panel, pos = (5,5))
		# my_sizer.Add(self.text_ctrl, 0, wx.ALL | wx.EXPAND, 5)


		# bmp = wx.Bitmap("NEW.BMP", wx.BITMAP_TYPE_BMP) 
		# self.bmpbtn = wx.BitmapButton(panel, id = wx.ID_ANY, bitmap = bmp, size = (bmp.GetWidth()+10, bmp.GetHeight()+10), pos = (10, 10))

		# my_button = wx.Button(panel, label='Press Here', pos = (5,55))
		# my_button.Bind(wx.EVT_BUTTON, self.on_press)
		# my_sizer.Add(my_button, 0, wx.ALL | wx.CENTER, 5)

		buttonmoveLeft = wx.Button(panel, id = wx.ID_ANY, size = (80,60), label = u"\u2B05", pos = (220, 250))
		buttonmoveLeft.Bind(wx.EVT_BUTTON, self.leftbutton)

		buttonmoveUp = wx.Button(panel, id = wx.ID_ANY, size = (80,60), label = u"\u2B06", pos = (305, 210))
		buttonmoveUp.Bind(wx.EVT_BUTTON, self.upbutton)
		# my_sizer.Add(self.buttonmoveUp, 0, wx.ALL | wx.CENTER, 5)

		buttonmoveDown = wx.Button(panel, id = wx.ID_ANY, size = (80,60), label = u"\u2B07", pos = (305, 300))
		buttonmoveDown.Bind(wx.EVT_BUTTON, self.downbutton)
		# my_sizer.Add(self.buttonmoveDown, 0, wx.ALL | wx.CENTER, 5)

		buttonmoveRight = wx.Button(panel, id = wx.ID_ANY, size = (80,60), label = u"\u27A1", pos = (390, 250))
		buttonmoveRight.Bind(wx.EVT_BUTTON, self.rightbutton)

		buttonmoveForward = wx.Button(panel, id = wx.ID_ANY, size = (80,60),label = u"\u2B08", pos = (400, 180))
		buttonmoveForward.Bind(wx.EVT_BUTTON, self.forwardbutton)

		buttonmoveBackward = wx.Button(panel, id = wx.ID_ANY, size = (80,60), label = u"\u2B0B", pos = (205, 320))
		buttonmoveBackward.Bind(wx.EVT_BUTTON, self.backwardbutton)

		buttonGrasp = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u0120", pos = (550, 180))
		buttonGrasp.Bind(wx.EVT_BUTTON, self.graspbutton)

		buttonDrop = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u0110", pos = (550, 310))
		buttonDrop.Bind(wx.EVT_BUTTON, self.dropbutton)

		buttonHome = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u0A2E", pos = (20, 180))
		buttonHome.Bind(wx.EVT_BUTTON, self.homebutton)

		buttonRest = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u24C7", pos = (20, 310))
		buttonRest.Bind(wx.EVT_BUTTON, self.restbutton)

		buttonOne = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u2776", pos = (380, 20))
		buttonOne.Bind(wx.EVT_BUTTON, self.onebutton)

		buttonTwo = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u2777", pos = (550, 20))
		buttonTwo.Bind(wx.EVT_BUTTON, self.twobutton)
 
		# buttonThree = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u2778", pos = (210, 20))
		# buttonThree.Bind(wx.EVT_BUTTON, self.threebutton)

		# buttonFour = wx.Button(panel, id = wx.ID_ANY, size = (120,100), label = u"\u2779", pos = (40, 20))
		# buttonFour.Bind(wx.EVT_BUTTON, self.fourbutton)

		# self.win = wx.Window(self, -1, size=(50,50), style=wx.SIMPLE_BORDER, pos = (100, 200))
		# self.win.SetBackgroundColour("white")

		# self.SetCursor(wx.StockCursor(wx.CURSOR_CROSS))

		self.Show()
	
	def publishbutton(self, val):
		'''Publishes one at a time '''
		string = coordinates()
		boolean = Bool()

		if val == "left":
			string.x = 1
		elif val == "right":
			string.x = 3
		elif val == "up":
			string.x = 2
		elif val == "down":
			string.x = 4
		elif val == "forward":
			string.x = 5
		elif val == "backward":
			string.x = 6
		elif val == "grasp":
			string.x = 7
		elif val == "drop":
			string.x = 8
		elif val == "home":
			string.x = 9
		elif val == "rest":
			string.x = 10
		elif val == "one":
			string.x = 11
		elif val == "two":
			string.x = 12
		elif val == "three":
			string.x = 13
		elif val == "four":
			string.x = 14
	
		string_pub = rospy.Publisher('button', coordinates, queue_size = 10)
		# bool_pub = rospy.Publisher('Flag', Bool, queue_size = 10)
		
		string_pub.publish(string)
	
	def leftbutton(self, event):
		"""What to do when the left arrow is pressed"""
		# pub = rospy.Publisher('left_button', String, queue_size = 10)
		# left_string = "left"
		# pub.publish(left_string)

		# btn = event.GetEventObject().GetValue() 
		# print("Label of pressed button = ",btn)

		self.publishbutton("left")

	def rightbutton(self, event):
		"""What to do when the right arrow is pressed"""
		self.publishbutton("right")

	def upbutton(self, event):
		"""What to do when the up arrow is pressed"""
		self.publishbutton("up")

	def downbutton(self, event):
		"""What to do when the down arrow is pressed"""
		self.publishbutton("down")

	def forwardbutton(self, event):
		"""What to do when the forward arrow is pressed"""
		self.publishbutton("forward")

	def backwardbutton(self, event):
		"""What to do when the backward arrow is pressed"""
		self.publishbutton("backward")

	def graspbutton(self, event):
		"""What to do when the grasp arrow is pressed"""
		self.publishbutton("grasp")

	def dropbutton(self, event):
		"""What to do when the drop arrow is pressed"""
		self.publishbutton("drop")

	def homebutton(self, event):
		"""What to do when the drop arrow is pressed"""
		self.publishbutton("home")

	def restbutton(self, event):
		"""What to do when the drop arrow is pressed"""
		self.publishbutton("rest")

	def onebutton(self, event):
		"""What to do when the one is pressed"""
		self.publishbutton("one")
	def twobutton(self, event):
		"""What to do when the two is pressed"""
		self.publishbutton("two")
	def threebutton(self, event):
		"""What to do when the three is pressed"""
		self.publishbutton("three")
	def fourbutton(self, event):
		"""What to do when the four is pressed"""
		self.publishbutton("four")



	def on_press(self, event):
		value = self.text_ctrl.GetValue()
		if not value:
			print("You did not enter anything")
		else:
			print("You typed: ", value)


if __name__  == '__main__':
	app = wx.App()
	frame = MyFrame()
	app.MainLoop()