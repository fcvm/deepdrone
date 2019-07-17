# Python 2
from Tkinter import *
# Python 3
#from tkinter import *


# Create a blank window (instance of Tk class) and store it in root
root = Tk()

'''
#VIDEO1
# Put text in the window
theLabel = Label(root, text="This is too easy.")

# Place the widget in the first place you find in the window
theLabel.pack()
#VIDEO1
'''

'''
#VIDEO2
# Create invisible containers to structure the window
topFrame = Frame(root)
topFrame.pack()
bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)


# Make 4 Buttons
button1 = Button(topFrame, text="Button 1", fg="red")
button2 = Button(topFrame, text="Button 2", fg="blue")
button3 = Button(topFrame, text="Button 3", fg="green")
button4 = Button(bottomFrame, text="Button 4", fg="purple")


# Display widgets, button 1-3 aside each other, button 4 beneath
button1.pack(side=LEFT)
button2.pack(side=LEFT)
button3.pack(side=LEFT)
button4.pack(side=BOTTOM)
#VIDEO2'''


'''
#VIDEO3
# bg-background, fg-text
one = Label(root, text="One", bg="red", fg="white")
one.pack()
two = Label(root, text="Two", bg="green", fg="black")
# fill=X, Widget fills window in x-direction
two.pack(fill=X)
three = Label(root, text="Three", bg="blue", fg="white")
# fill=Y, Widget fills window in y-direction
three.pack(side=LEFT, fill=Y)
#VIDEO3'''



#VIDEO4+5
# Make two labels
label_1 = Label(root, text="Name")
label_2 = Label(root, text="Password")
# Make two entries
entry_1 = Entry(root)
entry_2 = Entry(root)
# Organize widgets in rows and columns, by default: column=0
# Sticky (N, E, S, W) text orientation in the grid cell
label_1.grid(row=0, column=0, sticky=E)
label_2.grid(row=1, column=0)
entry_1.grid(row=0, column=1)
entry_2.grid(row=1, column=1)

# Make keep me locked in-checkbox
c = Checkbutton(root, text="Keep me signed in")
# Widget over several grid cells
c.grid(row=2, column=0, columnspan=2)
#VIDEO4+5'''



'''
#VIDEO6

# Binding a function to a widget
# Sample function
def printName(event):
    print("Hello my name is Bucky!")

# Make  a button, call a function (leave parenthesis)
button_1 = Button(root, text="Print my name")
# Event ("<Button-1>") = Left mouse click
button_1.bind("<Button-1>", printName)
button_1.pack()

#VIDEO6'''


'''#VIDEO7
def leftClick(event):
    print("Left")
def middleClick(event):
    print("Middle")
def rightClick(event):
    print("Right")

frame = Frame(root, width=300, height=250)
frame.bind("<Button-1>", leftClick)
frame.bind("<Button-2>", middleClick)
frame.bind("<Button-3>", rightClick)
frame.pack()
#VIDEO7'''


'''
#VIDEO8
# Using with Classes
class BuckysButtons:
    # root as same as master
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()

        self.printButton = Button(frame, text="Print Message", command=self.printMessage)
        self.printButton.pack(side=LEFT)

        self.quitButton = Button(frame, text="Quit", command=frame.quit)
        self.quitButton.pack(side=LEFT)

    def printMessage(self):
        print("Wow, this actually worked!")

b = BuckysButtons(root)

#VIDEO8
'''



'''
#VIDEO9+10+11
# Create a Menu

def doNothing():
    print("Ok, ok, I won't...")

menu = Menu(root)
root.config(menu=menu)

subMenu = Menu(menu)
menu.add_cascade(label="File", menu=subMenu)
subMenu.add_command(label="New Project...", command=doNothing)
subMenu.add_command(label="New...", command=doNothing)
subMenu.add_separator()
subMenu.add_command(label="Exit", command=doNothing)

editMenu = Menu(menu)
menu.add_cascade(label="Edit", menu=editMenu)
editMenu.add_command(label="Redo", command=doNothing)


# Create Toolbar

toolbar = Frame(root, bg="blue")

insertButt = Button(toolbar, text="Insert Image", command=doNothing)
# Padding~extra space
insertButt.pack(side=LEFT, padx=2, pady=2)
printButt = Button(toolbar, text="Print", command=doNothing)
printButt.pack(side=LEFT, padx=2, pady=2)

toolbar.pack(side=TOP, fill=X)


# Create Status Bar

# bd~bordered, relief=SUNKEN~"embedded into the window", anchor=W~Text is left-orientated
status = Label(root, text="Preparing to do nothing...", bd=1, relief=SUNKEN, anchor=W)
status.pack(side=BOTTOM, fill=X)


#VIDEO9+10+11'''


'''#VIDEO12

Python 2: 
import tkMessageBox
tkMessageBox.showinfo('Window Title', "Monkeys can live up to 300 years.")

answer = tkMessageBox.askquestion('Question 1', 'Do you like silly faces?')

if answer == 'yes':
    print(' 8==D~')

#Python 3: 
#import tkinter.messagebox

#tkinter.messagebox.showinfo('Window Title', "Monkeys can live up to 300 years.")
#
#answer = tkinter.messagebox.askquestion('Question 1', 'Do you like silly faces?')
#
#if answer == 'yes':
#    print(' 8==D~')

#VIDEO12'''



'''#VIDEO13

canvas = Canvas(root, width=200, height=100)
canvas.pack()

# Line: Startpoint, Endpoint, Filling (default: black)
blackLine = canvas.create_line(0, 0, 200, 50)
redLine = canvas.create_line(0, 100, 200, 50, fill="red")
# Rectangle: Startpoint, Endpoint, Filling
greenBox = canvas.create_rectangle(25, 25, 175, 75, fill="green")

# Delete Widget
canvas.delete(redLine)
# Delete All Widgets
canvas.delete(ALL)

#VIDEO13'''



'''
#VIDEO14

from PIL import Image, ImageTk


image = Image.open("Broccoli.png")#.resize((100, 100)) 
image.thumbnail((100, 100), Image.ANTIALIAS)
photo = ImageTk.PhotoImage(image)
label = Label(root, image=photo)
label.pack()



image = Image.open("george-michael-139~_v-videowebl.jpg")#.resize((100, 100))
image.thumbnail((200, 200), Image.ANTIALIAS)
photo2 = ImageTk.PhotoImage(image)
label = Label(root, image=photo2)
label.pack()
#VIDEO14'''





# Window continuosly on the screen until press its close button.
root.mainloop() 


