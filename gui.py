import _tkinter
from pickletools import read_stringnl_noescape
import tkinter as tk
from tkinter import *

#Display duration
def display_selected(choice):
    choice = variable.get()
    print(choice)

#Obtain Entries
def retrieve():
    temp_High_User = temp_High.get()
    temp_Low_User = temp_Low.get()
    volt_High_User = volt_High.get()
    volt_Low_User = volt_Low.get()
    email_User = email.get()   ######################################## Joseph this is the variable I'm storing the email in for now
    print(email_User)


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