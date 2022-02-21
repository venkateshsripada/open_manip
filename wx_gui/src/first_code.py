"""
import wx
import cv2

class MyFrame(wx.Frame):
	def __init__(self):
		super(MyFrame, self).__init__(parent = None, title = 'Hello World', size = (500, 300))

		panel = wx.Panel(self)
		# my_sizer = wx.BoxSizer(wx.VERTICAL)

		self.text_ctrl = wx.TextCtrl(panel, pos = (5,5))
		# my_sizer.Add(self.text_ctrl, 0, wx.ALL | wx.EXPAND, 5)

		my_button = wx.Button(panel, label='Press Here', pos = (5,55))
		my_button.Bind(wx.EVT_BUTTON, self.on_press)
		# my_sizer.Add(my_button, 0, wx.ALL | wx.CENTER, 5)

		self.buttonmoveLeft = wx.Button(panel, -1, u"\u2B05", pos = (215, 225))

		self.buttonmoveUp = wx.Button(panel, -1, u"\u2B06", pos = (300, 200))
		# my_sizer.Add(self.buttonmoveUp, 0, wx.ALL | wx.CENTER, 5)

		self.buttonmoveDown = wx.Button(panel, -1, u"\u2B07", pos = (300, 240))
		# my_sizer.Add(self.buttonmoveDown, 0, wx.ALL | wx.CENTER, 5)

		self.buttonmoveRight = wx.Button(panel, -1, u"\u27A1", pos = (385, 225))

		# self.win = wx.Window(self, -1, size=(50,50), style=wx.SIMPLE_BORDER, pos = (100, 200))
		# self.win.SetBackgroundColour("white")

		# self.SetCursor(wx.StockCursor(wx.CURSOR_CROSS))
		self.show_video()

		self.Show()

	def show_video(self):
		cv2.NamedWindow("camera", 1)
		capture = cv2.CaptureFromCAM(0)
		img = cv2.QueryFrame(capture)
		cv2.ShowImage("camera", img)

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

# app = wx.App()
# frame = wx.Frame(parent=None, title='Hello World')

# frame.Show()
# app.MainLoop() 

"""

import wx
class OtherFrame(wx.Frame):
    """
    Class used for creating frames other than the main one
    """
    
    def __init__(self, title, parent=None):
        wx.Frame.__init__(self, parent=parent, title=title)
        self.Show()
class MyPanel(wx.Panel):
    
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        
        btn = wx.Button(self, label='Create New Frame')
        btn.Bind(wx.EVT_BUTTON, self.on_new_frame)
        self.frame_number = 1
        
    def on_new_frame(self, event):
        title = 'SubFrame {}'.format(self.frame_number)
        frame = OtherFrame(title=title)
        self.frame_number += 1
        
class MainFrame(wx.Frame):
    
    def __init__(self):
        wx.Frame.__init__(self, None, title='Main Frame', size=(800, 600))
        panel = MyPanel(self)
        self.Show()
        
if __name__ == '__main__':
    app = wx.App(False)
    frame = MainFrame()
    app.MainLoop()