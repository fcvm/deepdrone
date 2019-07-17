#!/usr/bin/env python


#import os
# os.system('roslaunch turtlebot_navigation_ours amcl_customized.launch')

import os
dirname = os.path.dirname(__file__)
#filename = os.path.join(dirname, 'relative/path/to/file/you/want')




try:
    # Python2
    import Tkinter as tk
except ImportError:
    # Python3
    import tkinter as tk


import numpy as np

#from ImportImages import *
#import ImageTk

from PIL import Image, ImageTk



#import matplotlib.pyplot as plt







class LabeledFrame:
    # Labeled Empty Frame
    
    def get_labeledFrame(self, Master, Label):
        Frame = tk.Frame(Master)
        Frame.pack()

        Label = tk.Label(Frame, text=Label)
        Label.pack(anchor=tk.W)
        
        return Frame



class DirectionFrame(LabeledFrame):
    # Frame containing direction buttons

    def __init__(self, Master):

        self.directionFrame = self.get_labeledFrame(Master, "\n--- Direction ---")
        
        # Number of Buttons
        nButtons = 7
        # By default selected Button
        defaultSelection = 3
        # Storing the currently selected Button and the previous selected Button
        self.directionSelection = [defaultSelection, None]

        # 
        
        self.load_directionIcons()

        # Create the Buttons, each of the Buttons passes its index into the Click function for identification
        self.directionButtons = []
        for n in range(nButtons):
            Button = tk.Button(self.directionFrame, image=self.directionIcons[False][n], command=(lambda n=n : self.directionClick(n)))
            Button.pack(side=tk.LEFT)
            self.directionButtons.append(Button)

        self.directionButtons[defaultSelection].config(image=self.directionIcons[True][defaultSelection], relief=tk.SUNKEN)


    def directionClick(self, ClickedButton):
        
        print("Direction Button #" + str(ClickedButton) + " Selected.")
        # Update previous Selection
        self.directionSelection[1] = self.directionSelection[0]
        # Update current Selection
        self.directionSelection[0] = ClickedButton

        # Set previous Button to Off
        self.directionButtons[self.directionSelection[1]].config(relief=tk.RAISED, image=self.directionIcons[False][self.directionSelection[1]])
        # Set clicked Button to On
        self.directionButtons[self.directionSelection[0]].config(relief=tk.SUNKEN, image=self.directionIcons[True][self.directionSelection[0]])


    def load_directionIcons(self):
        IconSize = (31, 31)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/sLeft90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        sleft90Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_sLeft90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_sleft90Image = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/bLeft90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        bleft90Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_bLeft90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_bleft90Image = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/Left45.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        left45Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_Left45.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_left45Image = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/Straight.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        straightImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_Straight.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_straightImage = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/Right45.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        right45Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_Right45.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_right45Image = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/bRight90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        bright90Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_bRight90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_bright90Image = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/sRight90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        sright90Image = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/DirectionFrame/grey_sRight90.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_sright90Image = ImageTk.PhotoImage(LoadedImage)

        self.directionIcons = { True: [sleft90Image, bleft90Image, left45Image, straightImage, right45Image, bright90Image, sright90Image], False: [grey_sleft90Image, grey_bleft90Image, grey_left45Image, grey_straightImage, grey_right45Image, grey_bright90Image, grey_sright90Image]}




class SlopeFrame(LabeledFrame):
    # Frame containing direction buttons

    def __init__(self, Master):

        self.slopeFramS = self.get_labeledFrame(Master, "\n--- Slope ---")
        
        # Number of Buttons
        nButtons = 3
        # By default selected Button
        defaultSelection = 1
        # Storing the currently selected Button and the previous selected Button
        self.slopeSelection = [defaultSelection, None]
        
        self.load_slopeIcons()

        # Create the Buttons, each of the Buttons passes its index into the Click function for identification
        self.slopeButtons = []
        for n in range(nButtons):
            Button = tk.Button(self.slopeFramS, image=self.slopeIcons[False][n], command=(lambda n=n : self.slopeClick(n)))
            Button.pack(side=tk.LEFT)
            self.slopeButtons.append(Button)

        self.slopeButtons[defaultSelection].config(image=self.slopeIcons[True][defaultSelection], relief=tk.SUNKEN)


    def slopeClick(self, ClickedButton):
        
        print("Slope Button #" + str(ClickedButton) + " Selected.")
        # Update previous Selection
        self.slopeSelection[1] = self.slopeSelection[0]
        # Update current Selection
        self.slopeSelection[0] = ClickedButton

        # Set previous Button to Off
        self.slopeButtons[self.slopeSelection[1]].config(relief=tk.RAISED, image=self.slopeIcons[False][self.slopeSelection[1]])
        # Set clicked Button to On
        self.slopeButtons[self.slopeSelection[0]].config(relief=tk.SUNKEN, image=self.slopeIcons[True][self.slopeSelection[0]])


    def load_slopeIcons(self):
        IconSize = (80, 80)

        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/Downward.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        downwardImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/grey_Downward.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_downwardImage = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/Flat.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        flatImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/grey_Flat.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_flatImage = ImageTk.PhotoImage(LoadedImage)

        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/Upward.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        upwardImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/SlopeFrame/grey_Upward.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_upwardImage = ImageTk.PhotoImage(LoadedImage)

        self.slopeIcons = { True: [downwardImage, flatImage, upwardImage], False: [grey_downwardImage, grey_flatImage, grey_upwardImage]}






class BuildFrame(LabeledFrame):
    # Frame containing build button
    def __init__(self, Master):
        
        self.buildFrame = self.get_labeledFrame(Master, "\n--- Build Gate ---")

        self.buildEnabled = True

        self.load_buildIcons()

        #image=self.buildIcons[1]

        self.buildButton = tk.Button(self.buildFrame, image=self.buildIcons[self.buildEnabled][0], command=self.buildClick)
        #self.buildButton = tk.Button(self.buildFrame, command=self.buildClick)
        self.buildButton.pack()

        #self.Poses = [startPose]

    def buildClick(self):
        if self.buildEnabled == True:
            print("Click.")
            # self.Poses.append(np.array([[1, 0, 0], [1, 0, 0]]))
            #print((self.Poses[-1])[0, :])

        else:
            print("Button not enabled.")
        #if ((self.Poses[-1])[0, :] == np.array([0, 0, 0])).all():
        #    self.canBuild = False
        #    self.buildButton.config(image=self.unselectedImages, relief=tk.SUNKEN)
    
    def load_buildIcons(self):
        IconSize = (250, 250)
        LoadedImage = Image.open(os.path.join(dirname, "img/BuildFrame/Gate.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        gateImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/BuildFrame/grey_Gate.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_gateImage = ImageTk.PhotoImage(LoadedImage)
        self.buildIcons = { True: [gateImage], False: [grey_gateImage]}




class WreckFrame(LabeledFrame):
    # Frame containing build button
    
    def __init__(self, Master):
        
        self.wreckFrame = self.get_labeledFrame(Master, "\n--- Demolish Gate ---")

        self.wreckEnabled = False

        self.load_wreckIcons()

        self.wreckButton = tk.Button(self.wreckFrame, image=self.wreckIcons[self.wreckEnabled][0], command=self.wreckClick)
        self.wreckButton.config(relief=tk.SUNKEN)
        self.wreckButton.pack()

    def wreckClick(self):
        if self.wreckEnabled == True:
            print("Click.")
            # self.Poses.append(np.array([[1, 0, 0], [1, 0, 0]]))
            #print((self.Poses[-1])[0, :])

        else:
            print("Button not enabled.")
        #if ((self.Poses[-1])[0, :] == np.array([0, 0, 0])).all():
        #    self.canBuild = False
        #    self.buildButton.config(image=self.unselectedImages, relief=tk.SUNKEN)
    
    def load_wreckIcons(self):
        IconSize = (100, 100)
        LoadedImage = Image.open(os.path.join(dirname, "img/WreckFrame/WreckingMachine.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        gateImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/WreckFrame/grey_WreckingMachine.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_gateImage = ImageTk.PhotoImage(LoadedImage)
        self.wreckIcons = { True: [gateImage], False: [grey_gateImage]}


class TrajectoryFrame(LabeledFrame):
    # Frame containing buttons to define and start trajectory calculation
    
    def __init__(self, Master):
        
        self.trajFrame = self.get_labeledFrame(Master, "\n--- Compute Trajectory ---")

        self.trajEnabled = False

        self.load_trajIcons()

        self.roundFrame = tk.Frame(self.trajFrame)
        self.roundLabel = tk.Label(self.roundFrame, text="Number of Rounds: ")
        self.roundSpinbox = tk.Spinbox(self.roundFrame, from_=0, to=100, width=3)
        self.roundLabel.pack(side=tk.LEFT)
        self.roundSpinbox.pack()
        self.roundFrame.pack(side=tk.TOP)

        self.trajButton = tk.Button(self.trajFrame, image=self.trajIcons[self.trajEnabled][0], command=self.trajClick)
        self.trajButton.config(relief=tk.SUNKEN)
        self.trajButton.pack()

    def trajClick(self):
        if self.trajEnabled == True:
            print("Click.")
            # self.Poses.append(np.array([[1, 0, 0], [1, 0, 0]]))
            #print((self.Poses[-1])[0, :])

        else:
            print("Button not enabled.")
        #if ((self.Poses[-1])[0, :] == np.array([0, 0, 0])).all():
        #    self.canBuild = False
        #    self.buildButton.config(image=self.unselectedImages, relief=tk.SUNKEN)
    
    def load_trajIcons(self):
        IconSize = (200, 200)
        LoadedImage = Image.open(os.path.join(dirname, "img/TrajectoryFrame/traj.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        gateImage = ImageTk.PhotoImage(LoadedImage)
        LoadedImage = Image.open(os.path.join(dirname, "img/TrajectoryFrame/grey_traj.png"))#.resize((100, 100)) 
        LoadedImage.thumbnail(IconSize, Image.ANTIALIAS)
        grey_gateImage = ImageTk.PhotoImage(LoadedImage)
        self.trajIcons = { True: [gateImage], False: [grey_gateImage]}



class CreatorFrame(DirectionFrame, SlopeFrame, BuildFrame, WreckFrame, TrajectoryFrame):
    
    def __init__(self, Master):
        self.creatorFrame = tk.Frame(Master)
        self.creatorFrame.pack(side=tk.LEFT)

        DirectionFrame.__init__(self, self.creatorFrame)
        SlopeFrame.__init__(self, self.creatorFrame)
        BuildFrame.__init__(self, self.creatorFrame)
        WreckFrame.__init__(self, self.creatorFrame)
        TrajectoryFrame.__init__(self, self.creatorFrame)

        # MATH
        startPose = np.array([[0, 1,], [0, 0], [0, 0]])
        self.Poses = [startPose]

        self.Turns = [0]

        #self.GateDiameterValue = 1
        #self.directionValues = np.array([[2, 3, 3, 3, 3, 3, 2], [2, 3, 1, 0, -1, -3, -2]]) * self.GateDiameterValue
        #self.directionAngles = np.array([0.5, 0.5, 0.25, 0, -0.25, -0.5, -0.5]) * np.pi
        #self.slopeValues = np.array([-1, 0, 1]) * self.GateDiameterValue

        self.GateDiameterValue = 1
        self.directionValues = np.array([[1.5, 3, 4.5, 3, 4.5, 3, 1.5], [0.5, 1, 1.5, 0, -1.5, -1, -0.5]]) * self.GateDiameterValue
        self.directionAngles = np.array([0.5/3, 0.5/3, 0.5/3, 0, -0.5/3, -0.5/3, -0.5/3]) * np.pi
        self.slopeValues = np.array([-1, 0, 1]) * self.GateDiameterValue


    def buildClick(self):
        if self.buildEnabled == True:
            newPose = self.get_newPose(self.Poses[-1], self.directionSelection[0], self.slopeSelection[0])
            self.Poses.append(newPose)
            print("Current Pose:\n" + str(self.Poses[-1]))
            
            if (self.Poses[0] == self.Poses[-1]).all():
                self.buildEnabled = False
                self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.SUNKEN)
            
            self.wreckEnabled = True
            self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.RAISED)

            self.trajEnabled = True
            self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.RAISED)
        else:
            print("Race track is already closed.")

    def get_newPose(self, currentPose, i_chosenDirection, i_chosenSlope):

        currentPosition = currentPose[:, 0][:, np.newaxis]
        currentOrientation = currentPose[:, 1][:, np.newaxis]

        nextPosition = np.zeros(currentPosition.shape)
        nextOrientation =  np.zeros(currentOrientation.shape)

        chosenDirection = self.directionValues[:, i_chosenDirection][:, np.newaxis]
        chosenOrientation = self.directionAngles[i_chosenDirection]

        self.Turns.append(chosenOrientation)

        chosenSlope = self.slopeValues[i_chosenSlope]

        
        rotationMatrix = np.array([[np.cos(chosenOrientation), -np.sin(chosenOrientation)], 
                                   [np.sin(chosenOrientation), np.cos(chosenOrientation)]])

        self.directionValues = np.matmul(rotationMatrix, self.directionValues)

        nextPosition[:2, 0] = currentPosition[:2, 0] + chosenDirection[:,0]
        nextPosition[-1, 0] = currentPosition[-1, 0] + chosenSlope

        nextOrientation[:2, 0] = np.matmul(rotationMatrix, currentOrientation[:2, 0])
        nextOrientation = nextOrientation / np.linalg.norm(nextOrientation)
        #print(nextOrientation)

        return np.round(np.concatenate([nextPosition, nextOrientation], 1), decimals=4)

    def wreckClick(self):
        if self.wreckEnabled == True:
            del self.Poses[-1]
            
            backRotation = - self.Turns[-1]
            del self.Turns[-1]

            rotationMatrix = np.array([[np.cos(backRotation), -np.sin(backRotation)], 
                                   [np.sin(backRotation), np.cos(backRotation)]])
                
            self.directionValues = np.matmul(rotationMatrix, self.directionValues)


            print("Current Pose:\n" + str(self.Poses[-1]))
            
            if len(self.Poses) == 1:
                self.wreckEnabled = False
                self.wreckButton.config(image=self.wreckIcons[self.wreckEnabled][0], relief=tk.SUNKEN)

                self.trajEnabled = False
                self.trajButton.config(image=self.trajIcons[self.trajEnabled][0], relief=tk.SUNKEN)

            self.buildEnabled = True
            self.buildButton.config(image=self.buildIcons[self.buildEnabled][0], relief=tk.RAISED)
        else:
            print("No Gate built yet.")




def DefineMetrics():
    GateDiameter = 1

    DirectionValues = np.array([[2, 3, 3, 1, 3, 3, 2], [-2, -3, -1, 0, 1, 2, 3]]) * GateDiameter

    SlopeValues = np.array([-1, 0, 1]) * GateDiameter

    return [GateDiameter, DirectionValues, SlopeValues]





if __name__ == '__main__':

    # *** WINDOW ***
    root = tk.Tk()

    GateDiameter, DirectionValues, SlopeValues = DefineMetrics()
    #racetrack2DImage, racetrack3DImage = LoadDynamicImgs()

    CreatorFrame = CreatorFrame(root)

    # Keep window continuosly open until close button pressed
    root.mainloop()