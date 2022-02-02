import _tkinter
from json.tool import main
from pickletools import read_stringnl_noescape
import tkinter as tk
from tkinter import *
import smtplib, ssl
import serial
import time

#Display duration
def display_selected(choice):
    choice = variable.get()
    print(choice)

#Obtain Entries
def retrieve():
    temp_High_User = int(temp_High.get())
    temp_Low_User = int(temp_Low.get())
    volt_High_User = int(volt_High.get())
    volt_Low_User = int(volt_Low.get())
    receiver_email = email.get()   ######################################## Joseph this is the variable I'm storing the email in for now

    ##### Writing to live feed #### Should have intial header than data feed will be lower 
    #row = 20
    #column = 50
    #feed_background.create_text(column, row, text="Date", fill="black", font=('Helvetica 15 bold'))
    #feed_background.create_text((column + 30), row, text="Temperature[C]", fill="black", font=('Helvetica 15 bold'))
    #feed_background.create_text((column + 60), row, text="Temperature[V]", fill="black", font=('Helvetica 15 bold'))

    #### Maybe create a start button instead of doing it all on submit

    serialcomm = serial.Serial('COM9',9600)
    serialcomm.timeout = 5
    temp = 'a'
    volt = 'b'
    while True:
        time.sleep(2)

        # Getting temp values
        flag = 1
        serialcomm.write(flag.to_bytes(1,'big'))
        time.sleep(0.5)
        byte_val1 = serialcomm.readline()
        temp = int.from_bytes(byte_val1, 'big')
        #temp = serialcomm.readline().decode('ascii')
        print(temp)

        flag = 2
        serialcomm.write(flag.to_bytes(1,'big'))
        time.sleep(0.5)
        byte_val2 = serialcomm.readline()
        volt = int.from_bytes(byte_val2, 'big')
        #temp = serialcomm.readline().decode('ascii')
        print(volt)

        if temp > temp_High_User or temp_Low_User > temp or volt > volt_High_User or volt_Low_User > volt:
            port = 465  # For SSL
            smtp_server = "smtp.gmail.com"
            sender_email = "sbmsalertsystem@gmail.com"  # Enter your address
            #receiver_email = "harrelljoseph91@gmail.com"  # Enter receiver address
            password = "sbmspassword1!" #input("Type your password and press enter: ")
            message = """\
            Subject: SBMS Alert System Test

            Battery is in Danger."""

            context = ssl.create_default_context()
            with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
                server.login(sender_email, password)
                server.sendmail(sender_email, receiver_email, message)
        
        if temp != 'a' and volt != 'b':
            break

#Set up
root = Tk()
root.geometry("1000x800")
frame = Frame(root)
frame.pack() 

#file = PhotoImage(file = "download.png")
#sbms = PhotoImage(file = "sbmslogo3.png")

#canvas = Canvas(frame,bg='white', width = 1000,height = 800)
#image = canvas.create_image(150, 150, image=sbms)
#image = root.create_image(150, 150, image=file)

#Frame
leftframe = Frame(root)
leftframe.pack(side=LEFT)
 
rightframe = Frame(root)
rightframe.pack(side=RIGHT)

#Tempertaure Inputs
temp_High = Entry(root, width = 20)
temp_High.place(x = 250, y = 150)
 
temp_Low = Entry(root, width = 20)
temp_Low.place(x = 100, y = 150)

tempL = Label(root, text = 'Low Temperature' )
tempL.place(x = 100, y = 127)

tempH = Label(root, text = 'High Temperature' )
tempH.place(x = 250, y = 127)

#Voltage Inputs
volt_High = Entry(root, width = 20)
volt_High.place(x = 250, y = 250)
 
volt_Low = Entry(root, width = 20)
volt_Low.place(x = 100, y = 250)

voltL = Label(root, text = 'Low Voltage' )
voltL.place(x = 100, y = 227)

voltH = Label(root, text = 'High Voltage' )
voltH.place(x = 250, y = 227)

#Duration Input
dur_options = [
    "1 minute",
    "15 minutes",
    "30 minutes",
    "1 hour",
    "2 hours",
    "6 hours",
    "12 hour",
    "1 day",
    "1 week",
    "4 weeks"
]

dur_choice = tk.StringVar()
dur_choice.set(dur_options[3])

dur = OptionMenu( root , dur_choice, *dur_options, command = display_selected )
dur.place(x = 100, y = 350)

durL = Label(root, text = 'Duration' )
durL.place(x = 100, y = 327)

#Email Input
email = Entry(root, width = 20)
email.place(x = 100, y = 450)

emailL = Label(root, text = 'Email' )
emailL.place(x = 100, y = 427)

#Submit button
submit = Button(root, text = "Submit", command = retrieve)
submit.place(x = 100, y = 527)

#Livefeed
feed_background = Canvas(root , bg = 'grey' , width = 500,height = 600)
feed_background.place(x = 450, y = 150)

feedL = Label(root, text = 'Measurements' )
feedL.place(x = 450, y = 127)
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

root.title("SBMS")
root.mainloop()
