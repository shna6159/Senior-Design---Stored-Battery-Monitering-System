import _tkinter
from json.tool import main
from pickletools import read_stringnl_noescape
import tkinter as tk
from tkinter import *
import smtplib, ssl
from email.mime.text import MIMEText
import serial
import time
import datetime
import os
import threading
 

class GUI(tk.Tk):

    Flag = False
    serialcomm = None

    def __init__(root):
        super().__init__()

        root.threadBase = len(threading.enumerate())
        
        #Create Window
        root.title('SBMS')
        root.geometry("800x600")
        #root.python_image = tk.PhotoImage(file='SBMS_LOGO2.png')
        #root.iconphoto(False, root.python_image)

        #my_label = Label(root, 'SBMS')

        #Duration Options
        root.dur_options = [
            "1 minute",
            "5 minutes",
            "1 hour",
            "6 hours",
            "12 hours",
            "1 day",
            "1 week",
            "4 weeks"
        ]
        root.dur_choice = tk.StringVar(root)
        root.dur_choice.set(root.dur_options[2])
        

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

        #Start button
        root.submit = Button(root, text = "Start", command = root.thread)
        root.submit.place(x = 100, y = 527)

        #End Button
        root.end = Button(root, text = "End", command = root.Sys_End)
        root.end.place(x=250,y=527)

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
        global FLAG
    
        while FLAG == True:
            global serialcomm

            Er = False
            E_print = True
            while Er == False:
                try:
                    root.temp_High_User = float(root.temp_High.get())
                    root.temp_Low_User = float(root.temp_Low.get())
                    root.volt_High_User = float(root.volt_High.get())
                    root.volt_Low_User = float(root.volt_Low.get())
                    root.dur_User = root.dur_choice.get()
                    root.receiver_email = root.email.get()
                    Er = True
                except ValueError:
                    if E_print == True:
                        E_file = "Error.txt"
                        tf = open(E_file) 
                        data = tf.read()
                        root.txtarea.delete(1.0,END)
                        root.txtarea.insert(END, data)
                        tf.close()
                        E_print = False

            try:
                serialcomm = serial.Serial("COM6", 115200)
                serialcomm.timeout = 1
            except serial.SerialException:
                print("WUT")

            # Getting Values
            if serialcomm.inWaiting():
                time.sleep(5)
                string = serialcomm.readline().decode('ascii').strip()
                arr = string.split(',')
                temp_str1 = arr[0]
                temp1 = float(temp_str1)
                temp_str2 = arr[1]
                temp2 = float(temp_str2)
                volt_str = arr[2]
                volt = float(volt_str)
                Batt_Life_str = arr[3]
                Batt_Life = float(Batt_Life_str)
                print(Batt_Life)

                ##### Write data to text file
                ts = datetime.datetime.now()
                t = ts.strftime("%H:%M")
                month = ts.strftime("%m_%Y")
                #day = ts.strftime("%m_%d_%Y")
                file = "SBMS_" + month + ".txt"
                header = "Time\tTemperature[C]\tVoltage[V]\n---------------------------------\n"
                f = open(file, 'a')
                if os.stat(file).st_size == 0:
                    f.write(header)
                f.write(t + "\t" + temp_str1 + "\t" + temp_str2 + "\t" + volt_str + "\n")
                f.close()

                #Display file in GUI
                tf = open(file) 
                data = tf.read()
                root.txtarea.delete(1.0,END)
                root.txtarea.insert(END, data)
                tf.close()

                interval = "\r"
                # Getting Interval 
                if root.dur_User == "1 minute":
                    interval = "\r" #1byte = 1800 seconds
                if root.dur_User == "5 minutes":
                    interval = "1\r" #2bytes = 3600 seconds
                if root.dur_User == "1 hour":
                    interval = "11\r" #3bytes = 7200 seconds
                if root.dur_User == "6 hours":
                    interval = "111\r" #4bytes = 21600 seconds
                if root.dur_User == "12 hours":
                    interval = "1111\r" #5bytes = 43200 seconds
                if root.dur_User == "1 day":
                    interval = "11111\r" #6bytes = 86400 seconds
                if root.dur_User == "1 week":
                    interval = "111111\r" #7bytes = 604800 seconds
                if root.dur_User == "4 weeks":
                    interval = "1111111\r" #8bytes = 2416200 seconds

                # Write Interval to dongle
                serialcomm.write(interval.encode())

                # Sending Alert Email Temp
                if temp1 > root.temp_High_User or root.temp_Low_User > temp1 or temp2 > root.temp_High_User or root.temp_Low_User > temp2:
                    port = 465  # For SSL
                    smtp_server = "smtp.gmail.com"
                    sender_email = "sbmsalertsystem@gmail.com"
                    password = "sbmspassword1!"

                    msg = MIMEText('Battery temperature is out of range')

                    msg['Subject'] = 'SBMS Alert System: Temperature Alert'

                    context = ssl.create_default_context()
                    with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
                        server.login(sender_email, password)
                        server.sendmail(sender_email, gui.receiver_email, msg.as_string())

                # Sending Alert Email for Volt
                if volt > root.volt_High_User or root.volt_Low_User > volt:
                    port = 465  # For SSL
                    smtp_server = "smtp.gmail.com"
                    sender_email = "sbmsalertsystem@gmail.com"
                    password = "sbmspassword1!"

                    msg = MIMEText('Battery voltage is out of range')

                    msg['Subject'] = 'SBMS Alert System: Voltage Alert'

                    context = ssl.create_default_context()
                    with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
                        server.login(sender_email, password)
                        server.sendmail(sender_email, gui.receiver_email, msg.as_string())
                

                # Sending Alert Email Battery Life
                if Batt_Life < 3:
                    port = 465  # For SSL
                    smtp_server = "smtp.gmail.com"
                    sender_email = "sbmsalertsystem@gmail.com"
                    password = "sbmspassword1!"

                    msg = MIMEText('SBMS Battery is Low.  Please Charge')

                    msg['Subject'] = 'SBMS Alert System: Battery Life Alert'

                    context = ssl.create_default_context()
                    with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
                        server.login(sender_email, password)
                        server.sendmail(sender_email, gui.receiver_email, msg.as_string())

                serialcomm.close()

            threadNum = len(threading.enumerate())
            if threadNum > root.threadBase+1:
                break


    def thread(root):
        global FLAG 
        FLAG = True
        root.submit['text'] = "Running"
        root.submit['state'] = tk.DISABLED
        thread = threading.Thread(target=root.comms)
        thread.daemon = True
        thread.start()
        
    def Sys_End(root):
        global FLAG
        root.submit['text'] = "Start"
        root.submit['state'] = tk.NORMAL
        FLAG = False

gui = GUI()
gui.mainloop()