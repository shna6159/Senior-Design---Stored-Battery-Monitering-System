# importing only those functions which
# are needed
#from tkinter import Tk, mainloop, TOP
#from tkinter.ttk import Button
#import _tkinter
#from json.tool import main
#from pickletools import read_stringnl_noescape
#import tkinter as tk
#from tkinter import *
#import threading
#import serial
#import datetime
#import time

import _tkinter
from json.tool import main
from pickletools import read_stringnl_noescape
import tkinter as tk
from tkinter import *
import smtplib, ssl
#import serial
import time
import datetime
import threading
 
global FLAG

class GUI(tk.Tk):
    def __init__(root):
        super().__init__()
        
        root.threadBase = len(threading.enumerate())

        #Create Window
        root.geometry("800x600")

        #Duration Options
        root.dur_options = [
            "1 minute",
            "15 minutes",
            "30 minutes",
            "1 hour",
            "2 hours",
            "6 hours",
            "12 hours",
            "1 day",
            "1 week",
            "4 weeks"
        ]
        root.dur_choice = tk.StringVar(root)
        root.dur_choice.set(root.dur_options[3])
        

        #Initialize Inputs
        root.temp_High_User = 0
        root.temp_Low_User = 0
        root.volt_High_User = 0
        root.volt_Low_User = 0
        root.dur_User = 0
        root.receiver_email = 0
        
        #Make GUI
        root.create_widgets()


    def create_widgets(root):
        #Tempertaure Inputs
        root.temp_Low = Entry(root, width = 20)
        root.temp_Low.place(x = 100, y = 150)

        root.temp_High = Entry(root, width = 20)
        root.temp_High.place(x = 250, y = 150)

        tempL = Label(root, text = 'Low Temperature' )
        tempL.place(x = 100, y = 127)

        tempH = Label(root, text = 'High Temperature' )
        tempH.place(x = 250, y = 127)

        #Voltage Inputs
        root.volt_Low = Entry(root, width = 20)
        root.volt_Low.place(x = 100, y = 250)

        root.volt_High = Entry(root, width = 20)
        root.volt_High.place(x = 250, y = 250)

        voltL = Label(root, text = 'Low Voltage' )
        voltL.place(x = 100, y = 227)

        voltH = Label(root, text = 'High Voltage' )
        voltH.place(x = 250, y = 227)

        #Duration Input
        
        root.dur = OptionMenu(root , root.dur_choice,*root.dur_options)
        root.dur.place(x = 100, y = 350)

        durL = Label(root, text = 'Duration' )
        durL.place(x = 100, y = 327)
        
        #Email Input
        root.email = Entry(root, width = 45 )
        root.email.place(x = 100, y = 450)

        emailL = Label(root, text = 'Email' )
        emailL.place(x = 100, y = 427)

        #Submit button
        submit = Button(root, text = "Start", command = root.thread)
        submit.place(x = 100, y = 527)

        #Livefeed
        feed_background = Canvas(root , bg = 'grey' , width = 500,height = 600)
        feed_background.place(x = 450, y = 150)

        feedL = Label(root, text = 'Measurements' )
        feedL.place(x = 450, y = 127)

        root.txtarea = Text(feed_background, width=40, height=20)
        root.txtarea.pack(pady=0)

        pathh = Entry(feed_background)
        pathh.pack(side=LEFT, expand=True, fill=X, padx=0)

    def comms(root):
        FLAG = True
        root.temp_High_User = int(root.temp_High.get())
        root.temp_Low_User = int(root.temp_Low.get())
        root.volt_High_User = int(root.volt_High.get())
        root.volt_Low_User = int(root.volt_Low.get())
        root.dur_User = root.dur_choice.get()
        root.receiver_email = root.email.get()


        '''
        try:
            serialcomm = serial.Serial("COM9", 9600)
            serialcomm.timeout = 5
            #print "Port is open"
        except serial.SerialException:
            #serialcomm.close()
            time.sleep(30)
            #serialcomm = serial.Serial("COM9", 9600)
            #serialcomm.timeout = 5
            print("WUT")
            
            #serial.Serial("COM9", 9600).close()
            #print "Port is closed"
            #serialcomm = serial.Serial("COM9",9600)
            #serialcomm.timeout = 5
            #print "Port is open again"
        
        #serialcomm = serial.Serial('COM9',9600)
        #serialcomm.timeout = 5
        '''

    
        while FLAG == True:
            time.sleep(2)


            # Getting temp values
            flag = 1
            #serialcomm.write(flag.to_bytes(1,'big'))
            #time.sleep(0.5)
            #byte_val1 = serialcomm.readline()
            #temp = int.from_bytes(byte_val1, 'big')
            #temp_str = str(temp)
            temp = 30
            temp_str = str(temp)
            print(temp)

            # Getting volt values
            flag = 2
            #serialcomm.write(flag.to_bytes(1,'big'))
            #time.sleep(0.5)
            #byte_val2 = serialcomm.readline()
            #volt = int.from_bytes(byte_val2, 'big')
            #volt_str = str(volt)
            volt = 20
            volt_str = str(volt)
            print(volt)

            #serialcomm.close()

            ##### Test Write function
            ts = datetime.datetime.now()
            t = ts.strftime("%H:%M")
            #year = ts.strftime("%Y")
            #month = ts.strftime("%m")
            #day = ts.strftime()
            day = ts.strftime("%m_%d_%Y")
            file = "SBMS_" + day + ".txt"
            header = "Time\tTemperature[C]\tVoltage[V]\n---------------------------------\n"
            #print(ts + " " + temp + " " + volt)
            #f = open(file,'w+')
            #title = f.readline()
            #f.close()
            f = open(file, 'a')
            f.write(header)  
            #if title != header:
                #f.write(header)
            f.write(t + "\t" + temp_str + "\t" + volt_str + "\n")
            f.close()

            #Display file in GUI
            tf = open(file)  # or tf = open(tf, 'r')
            data = tf.read()
            root.txtarea.delete(1.0,END)
            root.txtarea.insert(END, data)
            tf.close()

            if temp > root.temp_High_User or root.temp_Low_User > temp or volt > root.volt_High_User or root.volt_Low_User > volt:
                port = 465  # For SSL
                smtp_server = "smtp.gmail.com"
                sender_email = "sbmsalertsystem@gmail.com"  # Enter your address
                #receiver_email = "harrelljoseph91@gmail.com"  # Enter receiver address
                password = "sbmspassword1!" #input("Type your password and press enter: ")
                message = """\
                Subject: SBMS Alert System Test
                Battery is in Danger."""
                print(message)
                #context = ssl.create_default_context()
                #with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
                #    server.login(sender_email, password)
                #    server.sendmail(sender_email, gui.receiver_email, message)

            threadNum = len(threading.enumerate())
            if threadNum > root.threadBase+1:
                break



    def thread(root):
        FLAG = False
        thread = threading.Thread(target=root.comms)
        thread.daemon = True
        thread.start()
        '''
        if FLAG == True:
            #thread.join()
            thread = threading.Thread(target=root.comms)
            thread.daemon = True
            thread.start()
            #comms()
            #loop = threading.Thread(target=mainloop)
            #loop.daemon = True
            #loop.start()
        else:
            thread = threading.Thread(target=root.comms)
            #make test_loop terminate when the user exits the window
            thread.daemon = True
            #if FLAG == True:
            #   thread.join() 
            #if thread.is_alive():
            #  thread.done()
            thread.start()
        '''

gui = GUI()
gui.mainloop()



    
 
#button = Button(root, text = 'Geeks', command = thread)
#button.pack(side = TOP, pady = 5)
 
#print('Running...')
# Calculating starting time
#start = time()
 
# in after method 5000 milliseconds
# is passed i.e after 5 seconds
# main window i.e root window will
# get destroyed

'''
#Frame
leftframe = Frame(root)
leftframe.pack(side=LEFT)
 
rightframe = Frame(root)
rightframe.pack(side=RIGHT)
'''









# calculating end time
#end = time()
print('Kill Yourself')


# time function used to calculate time
#from time import time
FLAG = False
serialcomm = None



        #time.sleep(1)





"""
temp_High = Entry(root, width = 20)
temp_High.insert(0,'High Temperature')
temp_High.pack(padx = 40, pady = 5, side=tk.LEFT)
 
temp_Low = Entry(root, width = 20)
temp_Low.insert(0,'Low Temperature')
temp_Low.pack(padx = 40, pady = 10, side=tk.LEFT)
"""
"""
button1 = Button(leftframe, text = "Button1")
button1.pack(padx = 3, pady = 3)
button2 = Button(rightframe, text = "Button2")
button2.pack(padx = 3, pady = 3)
button3 = Button(leftframe, text = "Button3")
button3.pack(padx = 3, pady = 3)
"""

 

