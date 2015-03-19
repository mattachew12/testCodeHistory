#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 daniel (daniel.kappler@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Pyqt example to demonstrate how openrave elements can be controlled by a qt-gui.

author: Daniel Kappler
copyright: 2011 Daniel Kappler (daniel.kappler@gmail.com)

This example shows how a qt-gui can control openrave functionality.
"""

import sys, os, logging, signal, re, time

import xml.etree.ElementTree as etree
import xml.dom.minidom as xmlm
from Tkinter import Tk
from tkFileDialog import askopenfilename
from multiprocessing import Process,Pipe
import csv
from PyQt4 import QtGui, QtCore

from SliderHighlight import *
from OpenRave import *
# removed import cv2, openravepy segmenter qtmocapdrawer *

import rospy
from mocapGUI.srv import *

logger = None
FRAME_RATE = 5 # 5 hundredths of a second, fps = 20

# needed for events such as mouse clicks in the GUI to activate associated functions
class CallbackHandler(QtCore.QThread):
    def __init__(self,pipe,callback=None):
        super(CallbackHandler,self).__init__()
        self.callback = callback
        self.pipe = pipe

    def run(self):
        resultValue = self.pipe.recv()
        msg = [self.callback,resultValue]
        self.emit(QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),msg)

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

# handles all functions relevant to the GUI window itself
class MainGUI(QtGui.QMainWindow):
    # sets up the GUI window and connects it to the other processes
    # xmlPath:      file path to the xml storing the structure of the footage data
    def __init__(self, pipeOR, pipeServer, xmlPath):
        super(MainGUI,self).__init__(None)
        self.pipeServer = pipeServer
        self.pipeOR = pipeOR

        self.CallbackHandler = CallbackHandler(self.pipeOR)
        self.connect(self.CallbackHandler,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.__index = 0

        self.xmlPath = xmlPath

        # init variables
        self.jumpTo = "" # frame number to jump to, "" means don't
        self.editedClipLabel = "" # edited label of an unsaved clip
        self.frameRate = FRAME_RATE # fps to play movie at
        self.maxFrame = 0 # count of frames within the run
        self.currFrame = 0 # current frame in the run
        self.unsavedClipDict = {} # dict of QString : clip

        # start actual processes
        self.readXML() # creates a hierarchy of the xml/data files
        self.initGUI() # sets up the GUI window itself
        self.startAnimation() # opens OpenRave window but doesn't populate it

    # reads the structure of footage data and clip labels from the given xml
    def readXML(self): # path to the xml file to open
        self.xml = etree.parse(self.xmlPath)
        self.root = self.xml.getroot()
        self.rootPath = self.root.attrib["path"]
        # make a dictionary of all the blocks. Key: block ID, Value: dictionary of runs inside
        self.hierarchy = {}

        for rootChild in self.root:
            if rootChild.tag == "setup_path":
                self.setupPath = os.path.join(self.rootPath, rootChild.attrib["path"])
            elif rootChild.tag == "block":
                # make a dictionary of all the runs. Key: run ID, Value: dictionary of clips inside
                runDict = {}
                for run in rootChild:
                    runItemsDict = {}
                    # adds all the components of the run to its dictionary
                    runItemsDict["clips"] = {}
                    runItemsDict["labels"] = {}
                    for runChild in run:
                        if runChild.tag == "markers":
                            runItemsDict["marker file"] = os.path.join(self.rootPath, runChild.attrib["path"])
                        elif runChild.tag == "objects":
                            runItemsDict["object file"] = os.path.join(self.rootPath, runChild.attrib["path"])
                        elif runChild.tag == "images":
                            runItemsDict["image folder"] = os.path.join(self.rootPath, runChild.attrib["path"])
                        elif runChild.tag == "clips":
                            runItemsDict["clip file"] = os.path.join(self.rootPath, runChild.attrib["path"])
                            runItemsDict["clips"], runItemsDict["labels"] = self.createClipDict(runItemsDict["clip file"])

                    runKey = QtCore.QString(run.attrib["runID"]) # the key will be from a QT menu item with its own string type
                    runDict[runKey] = runItemsDict # sets one dict of clips to the runID they are in

                blockKey = QtCore.QString(rootChild.attrib["blockID"]) # the key will be from a QT menu item with its own string type
                self.hierarchy[blockKey] = runDict # sets one dict of runs to the blockID they are in

    # creates the dictionary connecting clips to their runs and labels to their clips
    # clipPath:        path to a run's Clips.csv file on the hard drive
    # return clipDict: map of the QStrings of a tuple to the tuples themselves, "(10, 20)":(10, 20)
    # return labelDict: map of tuples to their labels, (10, 20):"pose 2"
    def createClipDict(self, clipPath):
        clipDict = {}
        labelDict = {}
        with open(clipPath, "r") as s_file:
            # x-by-2 matrix of clips, labels handled as an optional 3rd column
            clips = [] # clips must be turned into QStrings before becoming part of the dict
            for row in csv.reader(s_file, delimiter=","):
                if len(row) >= 2: # gets the clip itself
                    clip = "(%s, %s)" % (row[0], row[1])
                if (len(row) >= 3) and (row[2] != ""): # handles labels
                    labelDict[QtCore.QString(clip)] = row[2]
                clips.append(clip) # adds the tuple (start, end) to dict
            for clip in clips:
                clipDict[QtCore.QString(clip)] = clip # duplicates are just rewritten
            return clipDict, labelDict

    # sets up all the widgets in the GUI
    def initGUI(self):

        ########################## MAIN WIDGETS

        # self = MainWindow, sets up the GUI window
        self.setObjectName(_fromUtf8("MainGUI"))
        self.setGeometry(100, 100, 600, 475)
        self.setMinimumSize(QtCore.QSize(600, 475)) # can't get smaller, but it may grow
        self.setWindowTitle(QtGui.QApplication.translate("MainGUI", "MainGUI", None, QtGui.QApplication.UnicodeUTF8))

        # main central widget which holds everything
        self.centralFrame = QtGui.QWidget(self)
        self.centralFrame.setObjectName(_fromUtf8("centralwidget"))
        self.setCentralWidget(self.centralFrame) # necessary for connect slots by name

        ########################## MENU WIDGETS

        # TODO: Low Priority: implement tree view widget

        # blockMenu selects which block to view
        self.blockMenu = QtGui.QComboBox(self.centralFrame) # parent is the main central window
        self.blockMenu.setObjectName(_fromUtf8("blockMenu")) # used by on_widgetName_event() function
        self.blockMenu.setToolTip("Controls which pair of participants is viewed, which block")
        self.blockMenu.resize(150,25)
        self.blockMenu.move(25,25)

        # populate blockMenu by default
        self.blockMenu.addItem(QtCore.QString("")) # makes a default clear menu item
        for blockID in self.hierarchy.keys(): # fills in from the XML hierarchy
            self.blockMenu.addItem(QtCore.QString(blockID))

        # runMenu selects which run to view
        self.runMenu = QtGui.QComboBox(self.centralFrame) # parent is the main central window
        self.runMenu.setObjectName(_fromUtf8("runMenu")) # used by on_widgetName_event() function
        self.runMenu.setToolTip("Controls which trial is viewed, which run")
        self.runMenu.resize(150,25)
        self.runMenu.move(175,25)
        self.runMenu.hide()

        self.clipMenu = QtGui.QComboBox(self.centralFrame)
        self.clipMenu.setObjectName(_fromUtf8("clipMenu"))
        self.clipMenu.setToolTip("Controls which sections of a run are viewed, which clip")
        self.clipMenu.resize(250,25)
        self.clipMenu.move(325,25)
        self.clipMenu.hide()

        ########################## VIEW CONTROL WIDGETS

        # create box around view controlsMENU
        self.viewControlsFrame = QtGui.QFrame(self.centralFrame)
        self.viewControlsFrame.setFrameStyle(QtGui.QFrame.StyledPanel)
        self.viewControlsFrame.resize(550,175)
        self.viewControlsFrame.move(25,75)

        # label the viewing section controls
        self.viewLabel = QtGui.QLabel("Viewing Controls", self.viewControlsFrame)
        self.viewLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.viewLabel.resize(550,25)
        self.viewLabel.move(0,0) # relative to viewControlsFrame

        # displays the current frame number out of total frames
        # text set at runMenu activation and updated by viewing functions
        self.frameLabel = QtGui.QLabel(("Current Frame: %06d        Total Frames: %06d" %
                                        (self.currFrame, self.maxFrame)), self.viewControlsFrame)
        self.frameLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.frameLabel.resize(550,25)
        self.frameLabel.move(0,25)

        # general low detail scrubber controlling currently viewed frame
        self.scrubSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self.viewControlsFrame)
        self.scrubSlider.setObjectName(_fromUtf8("scrubSlider"))
        self.scrubSlider.setToolTip("Controls the footage frame. Can only be moved validly by dragging with the mouse")
        self.scrubSlider.setStyle(QtGui.QStyleFactory.create('plastique')) # nicer look
        self.scrubSlider.setSingleStep(0) # arrow keys do not work
        self.scrubSlider.setPageStep(0) # clicking the slider without dragging doesn't works
        self.scrubSlider.setTickInterval(self.maxFrame/10) # 10% intervals
        self.scrubSlider.setTickPosition(QSlider.TicksBelow) # 10% intervals
        self.scrubSlider.setRange(0, self.maxFrame)
        self.scrubSlider.move(0,50)
        self.scrubSlider.resize(550,25)

        # highlights saved and current clips
        self.sliderHighlight = Highlight(550, self.viewControlsFrame)
        self.sliderHighlight.move(0,50)
        self.sliderHighlight.resize(550,25)

        # low detail frame control backwards
        self.superSkipBackButton = QtGui.QPushButton("<<<", self.viewControlsFrame)
        self.superSkipBackButton.setObjectName(_fromUtf8("superSkipBackButton"))
        self.superSkipBackButton.setToolTip("Skips backward 200 frames")
        self.superSkipBackButton.move(25,75)
        self.superSkipBackButton.resize(50,25)

        # medium detail frame control backwards
        self.skipBackButton = QtGui.QPushButton("<<", self.viewControlsFrame)
        self.skipBackButton.setObjectName(_fromUtf8("skipBackButton"))
        self.skipBackButton.setToolTip("Skips backward 20 frames")
        self.skipBackButton.move(75,75)
        self.skipBackButton.resize(50,25)

        # precise frame control backwards
        self.rewindButton = QtGui.QPushButton("<", self.viewControlsFrame)
        self.rewindButton.setObjectName(_fromUtf8("rewindButton"))
        self.rewindButton.setToolTip("Moves backward by 1 frame")
        self.rewindButton.move(125,75)
        self.rewindButton.resize(50,25)

        # precise frame control backwards
        self.fastForwardButton = QtGui.QPushButton(">", self.viewControlsFrame)
        self.fastForwardButton.setObjectName(_fromUtf8("fastForwardButton"))
        self.fastForwardButton.setToolTip("Advances forward by 1 frame")
        self.fastForwardButton.move(200,75)
        self.fastForwardButton.resize(50,25)

        # medium detail frame control forwards
        self.skipForwardButton = QtGui.QPushButton(">>", self.viewControlsFrame)
        self.skipForwardButton.setObjectName(_fromUtf8("skipForwardButton"))
        self.skipForwardButton.setToolTip("Skips forward 20 frames")
        self.skipForwardButton.move(250,75)
        self.skipForwardButton.resize(50,25)

        # low detail frame control forwards
        self.superSkipForwardButton = QtGui.QPushButton(">>>", self.viewControlsFrame)
        self.superSkipForwardButton.setObjectName(_fromUtf8("superSkipForwardButton"))
        self.superSkipForwardButton.setToolTip("Skips forward 200 frames")
        self.superSkipForwardButton.move(300,75)
        self.superSkipForwardButton.resize(50,25)

        # button to jump to a specific frame
        self.jumpToButton = QtGui.QPushButton("Jump to:", self.viewControlsFrame)
        self.jumpToButton.setObjectName(_fromUtf8("jumpToButton"))
        self.jumpToButton.setToolTip("Jumps to the frame entered on the right")
        self.jumpToButton.move(375,75)
        self.jumpToButton.resize(75,25)

        # entry for the frame to jump to
        self.jumpToEntry = QtGui.QLineEdit(self.viewControlsFrame)
        self.jumpToEntry.setPlaceholderText("Frame") # background grey text
        self.jumpToEntry.setObjectName(_fromUtf8("jumpToEntry"))
        self.jumpToEntry.setToolTip("Enter a frame number to jump to")
        self.jumpToEntry.move(450,75)
        self.jumpToEntry.resize(75,25)

        # advances through the currently selected footage like a movie
        self.playButton = QtGui.QPushButton("Play", self.viewControlsFrame)
        self.playButton.setObjectName(_fromUtf8("playButton"))
        self.playButton.setToolTip("Plays through the footage like a movie")
        self.playButton.move(138,100)
        self.playButton.resize(50,25)

        # pauses the movie of the run
        self.pauseButton = QtGui.QPushButton("Pause", self.viewControlsFrame)
        self.pauseButton.setObjectName(_fromUtf8("pauseButton"))
        self.pauseButton.setToolTip("Stops playing through the footage")
        self.pauseButton.move(188,100)
        self.pauseButton.resize(50,25)

        # entry for the requested frame rate
        self.frameRateEntry = QtGui.QLineEdit(self.viewControlsFrame)
        self.frameRateEntry.setPlaceholderText("Frame rate") # background grey text
        self.frameRateEntry.setObjectName(_fromUtf8("frameRateEntry"))
        self.frameRateEntry.setToolTip("Enter a frame rate to play footage at (FPS) (optional)")
        self.frameRateEntry.move(250,100)
        self.frameRateEntry.resize(100,25)

        # displays the current view selected
        # text set at runMenu activation and updated by viewing functions
        self.footageLabel = QtGui.QLabel(("Current Footage: %s" % "None"),
                                       self.viewControlsFrame)
        self.footageLabel.setObjectName(_fromUtf8("footageLabel"))
        self.footageLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.footageLabel.resize(550,25)
        self.footageLabel.move(0,125)

        # pauses the movie of the run
        self.editClipButton = QtGui.QPushButton("Edit Clip", self.viewControlsFrame)
        self.editClipButton.setObjectName(_fromUtf8("editClipButton"))
        self.editClipButton.setToolTip("Edits the currently viewed clip")
        self.editClipButton.move(450,150)
        self.editClipButton.resize(100,25)

        ########################## ERROR MESSAGE BOX

        # tells the user if they do anything without apparent results
        self.errorMessageLabel = QtGui.QLabel((""), self.centralFrame)
        self.errorMessageLabel.setObjectName(_fromUtf8("errorMessageLabel"))
        self.errorMessageLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.errorMessageLabel.resize(550,25)
        self.errorMessageLabel.move(25,250)

        ########################## SPLITTING CONTROL WIDGETS

        # create box around view controlsMENU
        self.clipControlsFrame = QtGui.QFrame(self.centralFrame)
        self.clipControlsFrame.setFrameStyle(QtGui.QFrame.StyledPanel)
        self.clipControlsFrame.move(25,275)
        self.clipControlsFrame.resize(550,175)

        # label the viewing section controls
        self.viewLabel = QtGui.QLabel("Clip Editing Controls", self.clipControlsFrame)
        self.viewLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.viewLabel.move(0,0) # relative to clipControlsFrame
        self.viewLabel.resize(550,25)

        # label the viewing section controls
        self.clipListLabel = QtGui.QLabel("Unsaved Clips", self.clipControlsFrame)
        self.clipListLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.clipListLabel.move(25,25) # relative to clipControlsFrame
        self.clipListLabel.resize(225,25)

        self.clipListBox = QtGui.QListWidget(self.clipControlsFrame)
        self.clipListBox.setObjectName(_fromUtf8("clipListBox"))
        self.clipListBox.move(25,50)
        self.clipListBox.resize(225,100)

        # allows a new clip to be formed on top of the current run
        self.newClipButton = QtGui.QPushButton("New Clip", self.clipControlsFrame)
        self.newClipButton.setObjectName(_fromUtf8("newClipButton"))
        self.newClipButton.setToolTip("Allows a new clip to be made within the run")
        self.newClipButton.move(300,25)
        self.newClipButton.resize(100,25)

        # cancels out the process started by the newClipButton
        self.removeClipButton = QtGui.QPushButton("Delete Clip", self.clipControlsFrame)
        self.removeClipButton.setObjectName(_fromUtf8("removeClipButton"))
        self.removeClipButton.setToolTip("Permanently erases the current unsaved clip")
        self.removeClipButton.move(400,25)
        self.removeClipButton.resize(100,25)

        # sets the start frame of the currently selected unsaved clip
        self.startFrameButton = QtGui.QPushButton("Set Start", self.clipControlsFrame)
        self.startFrameButton.setObjectName(_fromUtf8("startFrameButton"))
        self.startFrameButton.setToolTip("Sets the starting frame number of an unsaved clip")
        self.startFrameButton.move(300,50)
        self.startFrameButton.resize(100,25)

        # holds the starting frame number of the current unsaved clip
        self.startFrameLabel = QtGui.QLabel("Start Frame #", self.clipControlsFrame)
        self.startFrameLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.startFrameLabel.setFrameStyle(QtGui.QFrame.StyledPanel)
        self.startFrameLabel.move(400,50)
        self.startFrameLabel.resize(100,25)

        # sets the end frame of the currently selected unsaved clip
        self.endFrameButton = QtGui.QPushButton("Set End", self.clipControlsFrame)
        self.endFrameButton.setObjectName(_fromUtf8("endFrameButton"))
        self.endFrameButton.setToolTip("Sets the ending frame number of an unsaved clip")
        self.endFrameButton.move(300,75)
        self.endFrameButton.resize(100,25)

        # holds the ending frame number of the current unsaved clip
        self.endFrameLabel = QtGui.QLabel("End Frame #", self.clipControlsFrame)
        self.endFrameLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.endFrameLabel.setFrameStyle(QtGui.QFrame.StyledPanel)
        self.endFrameLabel.move(400,75)
        self.endFrameLabel.resize(100,25)

        # sets the new label of the currently selected unsaved clip
        self.labelButton = QtGui.QPushButton("Set Label", self.clipControlsFrame)
        self.labelButton.setObjectName(_fromUtf8("labelButton"))
        self.labelButton.setToolTip("Sets the new label of an unsaved clip")
        self.labelButton.move(300,100)
        self.labelButton.resize(100,25)

        # holds the new label for an unsaved clip
        self.labelEntry = QtGui.QLineEdit(self.clipControlsFrame)
        self.labelEntry.setPlaceholderText(QtCore.QString("Clip Label")) # background grey text
        self.labelEntry.setObjectName(_fromUtf8("labelEntry"))
        self.labelEntry.setToolTip("Add a label to the current unsaved clip here")
        self.labelEntry.move(400,100)
        self.labelEntry.resize(100,25)

        # saves the current clip to run and updates viewer
        self.saveClipButton = QtGui.QPushButton("Save Clips", self.clipControlsFrame)
        self.saveClipButton.setObjectName(_fromUtf8("saveClipButton"))
        self.saveClipButton.setToolTip("Saves the entire group of clips being edited")
        self.saveClipButton.move(350,125)
        self.saveClipButton.resize(100,25)

        self.hideRunWidgets() # hides all widgets not immediately needed

        # allows the following on_widgetName_eventName() functions to work
        QtCore.QMetaObject.connectSlotsByName(self)

##############################################################################
#                             Widget Functions
##############################################################################
# These decorated functions are run by events such as clicking on GUI menus or buttons

    ######################################## Menu Functions

    # prepares the GUI for selecting the run
    # menuText: display name on the menu of the selected block
    @QtCore.pyqtSignature("QString")
    def on_blockMenu_activated(self, menuText):
        if self.unsavedClipDict == {}: # only move when no unsaved clips exist
            if menuText != QtCore.QString(""):

                self.hideRunWidgets()
                self.selectedBlockID = menuText

                # populate run menu with all runs inside the selected block
                self.runMenu.clear() # makes sure the new runs are only in the selected block
                self.runMenu.addItem(QtCore.QString("")) # makes the default first choice blank
                for runID in self.hierarchy[self.selectedBlockID].keys():
                    self.runMenu.addItem(runID)

                self.runMenu.show()
        else:
            self.errorMessageLabel.setText("Please save unsaved clips")
            for index in range(self.blockMenu.count()): # set back to previous ID
                if self.blockMenu.itemText(index) == self.selectedBlockID:
                    self.blockMenu.setCurrentIndex(index)

    # upon run selection the full GUI becomes available
    # menuText: display name on the menu of the selected run
    @QtCore.pyqtSignature("QString")
    def on_runMenu_activated(self, menuText):
        if self.unsavedClipDict == {}: # only move when no unsaved clips exist
            if menuText != QtCore.QString(""):
                self.selectedRunID = menuText

                # gets the attributes of the specified run and passes them to OpenRave
                self.markerFile = self.hierarchy[self.selectedBlockID][self.selectedRunID]["marker file"]
                self.objFile = self.hierarchy[self.selectedBlockID][self.selectedRunID]["object file"]
                self.imgDir = self.hierarchy[self.selectedBlockID][self.selectedRunID]["image folder"]

                self.sendToOR("startDrawingOpenRave", (self.markerFile, self.objFile, self.imgDir))
                self.showRunWidgets() # also resets fields such as maxFrame

            else: # menu was cleared, no run to control
                self.hideRunWidgets()

        else:
            self.errorMessageLabel.setText("Please save unsaved clips")
            for index in range(self.runMenu.count()): # set back to previous ID
                if self.runMenu.itemText(index) == self.selectedRunID:
                    self.runMenu.setCurrentIndex(index)

    # prepares a clip for viewing and editing, and changes the slider bar as necessary
    # menuText: display name on the menu of the selected clip
    @QtCore.pyqtSignature("QString")
    def on_clipMenu_activated(self, menuText):
        self.errorMessageLabel.setText("") # clear the error message box
        if menuText != QtCore.QString(""):

            # clear editing widgets
            self.updateHighlightBoxes()
            self.resetEditingBox()

            # Set the current clip
            self.editClipButton.show()
            self.selectedClipText = menuText
            self.footageLabel.setText("Current Footage: %s" % self.selectedClipText)
            clipFrames = re.findall("\d+", menuText) # gets the numbers from the clip label

            # Jump to start of current clip
            self.jumpToEntry.setText("") # clear jump to clip
            self.jumpTo = int(clipFrames[0])
            self.on_jumpToButton_clicked() # jumps to start of clip and sets slider

            # specially highlight the current clip
            frameTuple = self.menuTextToPercent(menuText)
            self.sliderHighlight.setCurrentClip(frameTuple)

        else: # menu was cleared, no current clip anymore
            self.resetEditingBox()
            self.editClipButton.hide() # hides ability to edit current clip
            self.sliderHighlight.resetCurrentClip() # hides current clip highlight
            self.footageLabel.setText("Current Footage: %s" % self.selectedRunID)

    # hides all widgets besides the block and run menus
    def hideRunWidgets(self):
        self.sendToOR("clearOpenRave") # removes footage from OpenRave
        self.clipMenu.hide()
        self.editClipButton.hide() # shown when viewing a clip
        self.sliderHighlight.resetCurrentClip() # hides current clip highlight

        self.viewControlsFrame.hide() # hide all widgets in the view control box
        self.clipControlsFrame.hide() # hide all widgets in the clip control box

    # activated when a new run is selected, shows all widgets besides the block and run menus
    def showRunWidgets(self):
        self.resetViewingBox() # resets viewing widgets for a new run
        self.unsavedClipDict = {} # clears any previous edited clips
        self.resetEditingBox() # resets editing widgets for a new run
        # show widgets
        self.clipMenu.show()
        self.viewControlsFrame.show() # show all widgets in the view control box
        self.clipControlsFrame.show() # show all widgets in the clip control box


    # resets the fields of the viewing box for a new run
    def resetViewingBox(self):
        self.footageLabel.setText("Current Footage: %s" % self.selectedRunID)
        self.currFrame = 0
        self.setMaxFrame()
        self.scrubSlider.setSliderPosition(0)
        self.scrubSlider.setTickInterval(self.maxFrame/10) # 10% intervals
        self.scrubSlider.setRange(0, self.maxFrame)
        self.frameLabel.setText("Current Frame: %06d        Total Frames: %06d" %
                                        (self.currFrame, self.maxFrame))
        self.frameRateEntry.setText("")
        self.jumpToEntry.setText("")
        self.errorMessageLabel.setText("")

        # clear any leftovers from clipMenu
        self.clipMenu.clear() # makes sure the new clips are only in the selected run
        self.fillClipMenu() # fills the menu in with new clips
        self.editClipButton.hide() # shown by viewing clips, hidden during run viewing
        self.sliderHighlight.resetCurrentClip() # hides current clip highlight
        self.sliderHighlight.resetClips()
        self.updateHighlightBoxes()

    # resets the fields after an edit to a clip has been made
    def resetEditingBox(self):
        self.labelEntry.setText("")
        self.startFrameLabel.setText("Start Frame #")
        self.endFrameLabel.setText("End Frame #")
        self.updateClipListBox() # keeps list up to date
        self.clipListBox.setCurrentItem(None)

    # populates the clip menu
    def fillClipMenu(self):
        self.clipMenu.clear()
        tempDict = self.hierarchy[self.selectedBlockID][self.selectedRunID] # dict of run elements
        self.clipMenu.addItem(QtCore.QString("")) # makes the default first choice blank
        for clipID in tempDict["clips"].keys():
            # highlight the clip on the slider
            frameTuple = self.menuTextToPercent(clipID)
            self.sliderHighlight.addClipRect(frameTuple)

            # create menuText for each clip
            if clipID in tempDict["labels"].keys():
                menuItem = (clipID + " " + tempDict["labels"][clipID])
            else:
                menuItem = clipID
            self.clipMenu.addItem(menuItem)
        self.clipMenu.setCurrentIndex(0)

    # gets the maximum number of frames in a run
    def setMaxFrame(self):
        self.maxFrame = self.addTwoIntsClient(0) # key for getMaxFrame is 0
        print self.maxFrame

    # helper for on_runMenu_activated() and called by view buttons
    # updates the label showing current and max frame numbers
    # delta: number of frames to advance by
    def updateFrameCount(self, delta):
        self.errorMessageLabel.setText("") # clear the error message box
        self.currFrame += delta
        self.sendToOR("changeFrame", delta)
        if self.currFrame > self.maxFrame: # reset at extremes
            self.currFrame = self.maxFrame
            self.errorMessageLabel.setText("advanced past the last frame, " +
                                           "setting to frame %d" % self.maxFrame)
        elif self.currFrame < 0: # reset at extremes
            self.currFrame = 0
            self.errorMessageLabel.setText("rewound past the first frame, " +
                                           "setting to frame 0")

        # set the scrub slider to the correct position
        self.scrubSlider.setSliderPosition(self.currFrame)

        # set the frame label correctly
        self.frameLabel.setText("Current Frame: %06d        Total Frames: %06d" %
                                        (self.currFrame, self.maxFrame))
        self.updateClipFrameLabels()

    ######################################## Viewing Controls

    # updates the current frame upon using the scrub bar
    @QtCore.pyqtSignature("int")
    def on_scrubSlider_sliderMoved(self, frameNumber):
        delta = frameNumber-self.currFrame
        self.updateFrameCount(delta)

    # rewinds the footage very quickly
    @QtCore.pyqtSignature("")
    def on_superSkipBackButton_clicked(self):
        self.updateFrameCount(-200)

    # quickly rewinds the footage
    @QtCore.pyqtSignature("")
    def on_skipBackButton_clicked(self):
        self.updateFrameCount(-20)

    # finely rewinds the footage by a single frame
    @QtCore.pyqtSignature("")
    def on_rewindButton_clicked(self):
        self.updateFrameCount(-1)

    # finely fast-forwards the footage by a single frame
    @QtCore.pyqtSignature("")
    def on_fastForwardButton_clicked(self):
        self.updateFrameCount(1)

    # quickly fast-forwards the footage
    @QtCore.pyqtSignature("")
    def on_skipForwardButton_clicked(self):
        self.updateFrameCount(20)

    # rewinds the footage very quickly
    @QtCore.pyqtSignature("")
    def on_superSkipForwardButton_clicked(self):
        self.updateFrameCount(200)

    # TODO: get msg from OR on frame count, if jumpTo > max, jumpTo text = max
    # jumps to frame in the footage
    @QtCore.pyqtSignature("")
    def on_jumpToButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        # if no errors in the frame value, jump to given frame
        if self.jumpTo != "":
            delta = self.jumpTo - self.currFrame
            self.updateFrameCount(delta)
        else:
            self.errorMessageLabel.setText("No frame specified to jump to")

    # preliminary reader to keep self.jumpTo updated
    # entry: characters in the entry box
    @QtCore.pyqtSignature("const QString&")
    def on_jumpToEntry_textEdited(self, entry):
        self.errorMessageLabel.setText("") # clear the error message box
        try: # validate the entry
            self.jumpTo = int(entry)
        except: # reset text when bad value is entered
            self.jumpTo = ""
            self.jumpToEntry.setText("")
            self.errorMessageLabel.setText("Non-Numeric character given to Jump To")

    # sets frame value to jump to if Jump To button is pressed
    @QtCore.pyqtSignature("")
    def on_jumpToEntry_editingFinished(self):
        self.errorMessageLabel.setText("") # clear the error message box
        entry = self.jumpToEntry.text()
        try: # validate the entry
            self.jumpTo = int(entry)
        except: # reset text when bad value is entered
            self.jumpTo = ""
            self.jumpToEntry.setText("")
            self.errorMessageLabel.setText("Non-Numeric character given to Jump To")

    # plays through the footage like a movie
    @QtCore.pyqtSignature("")
    def on_playButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        self.sendToOR("playFootage")

    # pauses playing through the footage like a movie
    @QtCore.pyqtSignature("")
    def on_pauseButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        self.sendToOR("pauseFootage")

    # preliminary reader to keep self.frameRate updated
    # entry: characters in the entry box
    @QtCore.pyqtSignature("const QString&")
    def on_frameRateEntry_textEdited(self, entry):
        self.errorMessageLabel.setText("") # clear the error message box
        try: # validate the entry
            entry = int(entry)
            if entry > 0: # fps cannot be less than 1
                self.frameRate = entry
                self.errorMessageLabel.setText("Frame rate must be greater than 0")
        except: # reset text when bad value is entered
            self.frameRate = ""
            self.frameRateEntry.setText("")
            self.errorMessageLabel.setText("Non-Numeric character given to Frame Rate")

    # sets frame rate when enter or Play button is pressed
    @QtCore.pyqtSignature("")
    def on_frameRateEntry_editingFinished(self):
        self.errorMessageLabel.setText("") # clear the error message box
        entry = self.frameRateEntry.text()
        try: # validate the entry
            entry = int(entry)
            if entry > 0: # fps cannot be less than 1
                self.frameRate = entry
                self.errorMessageLabel.setText("Frame rate must be greater than 0")
        except: # reset text when bad value is entered
            self.frameRate = ""
            self.frameRateEntry.setText("")
            self.errorMessageLabel.setText("Non-Numeric character given to Frame Rate")

    ######################################## Clip Editing Controls

    # prepares the current clip for editing
    @QtCore.pyqtSignature("")
    def on_editClipButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box

        # remove clip from clipMenu
        currentItemNum = self.clipMenu.currentIndex() # get index in the menu
        self.clipMenu.setCurrentIndex(0) # default back to no clip
        self.clipMenu.removeItem(currentItemNum) # pop from the menu
        self.on_clipMenu_activated(QtCore.QString("")) # reset widgets

        # adds clip to display box of unsaved clips
        start = int(re.findall("\d+", self.selectedClipText)[0]) # first frame
        end = int(re.findall("\d+", self.selectedClipText)[1]) # last frame
        # gets all text after parentheses
        label = self.selectedClipText[str(self.selectedClipText).find(")")+2:]
        self.unsavedClipDict[self.selectedClipText] = [start, end, label,
                                                         QtGui.QListWidgetItem(self.selectedClipText)]

        # sets GUI fields correctly
        self.jumpToEntry.setText("") # clear jump to clip
        self.jumpTo = start
        self.on_jumpToButton_clicked()
        self.startFrameLabel.setText("%d" % start)
        self.endFrameLabel.setText("End Frame #")
        self.labelEntry.setText(label)
        self.editedClipLabel = label

        # remove from clipMenu
        key = QtCore.QString("(%d, %d)" % (start, end))
        tempDict = self.hierarchy[self.selectedBlockID][self.selectedRunID]
        tempDict["clips"].pop(key)
        if key in tempDict["labels"].keys(): # if clip has a label
            tempDict["labels"].pop(key)

        self.refocusCurrentFootage(self.selectedClipText)

    # jumps to the clip start when selected and sets the label
    @QtCore.pyqtSignature("QListWidgetItem*")
    def on_clipListBox_itemClicked(self, item):
        self.errorMessageLabel.setText("") # clear the error message box
        # jumps to start of edited clip
        clipText = item.text() # gets current item text
        [start, end, label, widgetItem] = self.unsavedClipDict[clipText] # get clip data
        self.jumpToEntry.setText("") # clear jump to clip
        self.jumpTo = start
        self.on_jumpToButton_clicked()
        self.footageLabel.setText("Current Footage: %s" % clipText)
        self.selectUnsavedClip(clipText)

        # set appropriate labels
        self.updateClipFrameLabels()
        self.labelEntry.setText(label)
        self.editedClipLabel = label

    # adds a new clip to the unsaved list of clips
    @QtCore.pyqtSignature("")
    def on_newClipButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        # create a new clip in the unsaved clip list at the current frame
        clipText = QtCore.QString("(%d, %d)" % (self.currFrame, self.currFrame))
        if clipText not in self.unsavedClipDict.keys(): # creates new clip
            self.unsavedClipDict[clipText] = [self.currFrame, self.currFrame,
                                                QtCore.QString(""),
                                                QtGui.QListWidgetItem(clipText)]
            self.editedClipLabel = ""
            self.refocusCurrentFootage(clipText)
        else:
            self.errorMessageLabel.setText("New clip already exists at this frame")

    # removes current clip from list of clips being edited
    @QtCore.pyqtSignature("")
    def on_removeClipButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        clipToRemove = self.clipListBox.currentItem()
        if clipToRemove: # if it exists
            menuText = clipToRemove.text()
            self.unsavedClipDict.pop(menuText) # remove from list
        else:
            self.errorMessageLabel.setText("No clip selected to remove")
        self.refocusCurrentFootage()

    # sets the new start frame number of the current unsaved clip
    @QtCore.pyqtSignature("")
    def on_startFrameButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        clipToEdit = self.clipListBox.currentItem()
        if clipToEdit: # if it exists
            clipText = clipToEdit.text()
            end = self.unsavedClipDict[clipText][1] # current end frame
            label = self.labelEntry.text() # current label
            if self.currFrame < end: # if the new clip address is valid
                self.updateClip(clipText, self.currFrame, end, label)
            else:
                self.errorMessageLabel.setText("The start of a clip must be before its end")
        else:
            self.errorMessageLabel.setText("No clip selected for editing")
            self.refocusCurrentFootage()

    # sets the new end frame number of the current unsaved clip
    @QtCore.pyqtSignature("")
    def on_endFrameButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        clipToEdit = self.clipListBox.currentItem()
        if clipToEdit: # if it exists
            clipText = clipToEdit.text()
            start = self.unsavedClipDict[clipText][0] # current start frame
            label = self.labelEntry.text() # current label
            if self.currFrame > start: # if the new clip address is valid
                self.updateClip(clipText, start, self.currFrame, label)
            else:
                self.errorMessageLabel.setText("The end of a clip must be after its start")
        else:
            self.errorMessageLabel.setText("No clip selected for editing")
            self.refocusCurrentFootage()

    # sets the label entry of the current unsaved clip
    @QtCore.pyqtSignature("")
    def on_labelButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        clipToRelabel = self.clipListBox.currentItem()
        if clipToRelabel: # if an item is selected
            clipText = clipToRelabel.text()
            [start, end, label, widgetItem] = self.unsavedClipDict[clipText] # gets the clip data
            # update clip with new label and reset GUI labels
            self.updateClip(clipText, start, end, self.editedClipLabel)
        else: # if no item selected, empty label box
            self.labelEntry.setText("")
            self.errorMessageLabel.setText("No clip selected for editing")

    # preliminary reader to keep self.editedClipLabel updated
    # entry: characters in the entry box
    @QtCore.pyqtSignature("const QString&")
    def on_labelEntry_textEdited(self, entry):
        self.errorMessageLabel.setText("") # clear the error message box
        if str(entry).find(",") != -1: # throws off the CSV file
            self.errorMessageLabel.setText("Commas are not possible in the label")
            self.labelEntry.setText("")
        else:
            self.editedClipLabel = entry

    # sets label for current unsaved clip to change to when Clip Label is clicked
    @QtCore.pyqtSignature("")
    def on_labelEntry_editingFinished(self):
        self.errorMessageLabel.setText("") # clear the error message box
        if str(self.labelEntry.text()).find(",") != -1: # throws off the CSV file
            self.errorMessageLabel.setText("Commas are not possible in the label")
            self.labelEntry.setText("")
        else:
            self.editedClipLabel = self.labelEntry.text()

    # permanently saves the unsaved clips and possibly shorter list of saved clips
    @QtCore.pyqtSignature("")
    def on_saveClipButton_clicked(self):
        self.errorMessageLabel.setText("") # clear the error message box
        self.saveClips() # save clips to file and repopulate the hierarchy

    # selects an unsaved clip and sets GUI fields accordingly
    def selectUnsavedClip(self, clipText):
        # selects clip in list box
        [start, end, label, widgetItem] = self.unsavedClipDict[clipText]
        self.clipListBox.setCurrentItem(widgetItem)

        # add clip to highlighter temporarily
        self.updateHighlightBoxes() # resets the highlighted clips
        percentTuple = self.menuTextToPercent(clipText)
        self.sliderHighlight.addClipRect(percentTuple)
        self.sliderHighlight.setCurrentClip(percentTuple)

        # sets edit fields
        self.labelEntry.setText(label)

    # updates the unsaved clip list from the dict entries
    def updateClipListBox(self):
        # ownership of item is clipListBox
        self.clipListBox.clear() # so this line deletes it from unsaved ClipDict too
        for clipText in self.unsavedClipDict.keys():
            widgetItem = QtGui.QListWidgetItem(clipText) # create new widget item
            self.unsavedClipDict[clipText][3] = widgetItem # save for future comparison
            self.clipListBox.addItem(widgetItem) # add to list

    # clears all outlined boxes and remakes the list from the clip keys
    def updateHighlightBoxes(self):
        self.sliderHighlight.resetClips() # clear outlines
        # update outlines on the slider from the menu clips
        # tempDict is a dict of all the run elements
        tempDict = self.hierarchy[self.selectedBlockID][self.selectedRunID]
        for clipID in tempDict["clips"].keys():
            frameTuple = self.menuTextToPercent(clipID)
            self.sliderHighlight.addClipRect(frameTuple)

    # changes the labels to show valid clip edge changes based on the current frame
    def updateClipFrameLabels(self):
        if self.clipListBox.currentItem(): # if an item is selected
            clipText = self.clipListBox.currentItem().text()
            [start, end, label, widgetItem] = self.unsavedClipDict[clipText]
            # check if a possible frame change would be valid
            if self.currFrame < end:
                self.startFrameLabel.setText("%d" % self.currFrame)
            else:
                self.startFrameLabel.setText("Start Frame #")
            if self.currFrame > start:
                self.endFrameLabel.setText("%d" % self.currFrame)
            else:
                self.endFrameLabel.setText("End Frame #")

    # takes an unsaved clip and edits one of it's values
    # clipText: old QString text of the clip, key in unsavedClipDict
    # start: starting frame number of the new clip
    # end: ending frame number of the new clip
    # label: label of the new clip
    def updateClip(self, clipText, start, end, label):
        newClipText = QtCore.QString("(%d, %d) %s" % (start, end, label))
        self.unsavedClipDict.pop(clipText)
        self.unsavedClipDict[newClipText] = [start, end, label,
                                               QtGui.QListWidgetItem(newClipText)]
        self.refocusCurrentFootage(newClipText)

    # this function resets the GUI to focusing on a run instead of a clip
    # currentClipText: text of current clip to select after edit
    def refocusCurrentFootage(self, currentClipText=None):
        self.updateClipListBox()
        self.updateHighlightBoxes()
        if currentClipText != None:
            self.selectUnsavedClip(currentClipText)
            self.footageLabel.setText("Current Footage: %s" % currentClipText)
        # Change the footage label to update the new name
        elif self.clipMenu.currentIndex() == 0: # no clip selected
            self.footageLabel.setText("Current Footage: %s" % self.selectedRunID)
            self.labelEntry.setText("")
        else:
            self.footageLabel.setText("Current Footage: %s" % self.selectedClipText)
            self.labelEntry.setText("")
        self.updateClipFrameLabels()

    ######################################## Save Functions

    # saves clips to file, updates the XML, and repopulates the hierarchy
    def saveClips(self):
        currentRunDict = self.hierarchy[self.selectedBlockID][self.selectedRunID]
        if "clip file" in currentRunDict.keys(): # overwrite a current file
            absClipPath = currentRunDict["clip file"]
        else: # create a new file and store its path in the XML
            folder, path = os.path.split(currentRunDict["marker file"]) # absolute path
            absClipPath = os.path.join(folder, "Clips.csv") # add on file name
            self.updateXML(absClipPath) # updates the XML with new clips

        self.saveClipsToCSV(absClipPath) # save clips to file
        self.readXML() # repopulates the hierarchy
        self.fillClipMenu() # refills the clip menu

    # adds a clip element to the current run in the XML
    # clipPath: absolute path to clip file
    def updateXML(self, absClipPath):
        relClipPath = absClipPath[len(self.rootPath):] # make a relative path
        # find current run
        for rootChild in self.root:
            if rootChild.tag == "block" and rootChild.attrib["blockID"] == self.selectedBlockID:
                for run in rootChild:
                    if run.attrib["runID"] == self.selectedRunID:
                        # create new clip element in current run
                        etree.SubElement(run, "clips", {"path":relClipPath})
        # write new xml
        self.xml.write(self.xmlPath)

    # saves all saved and unsaved clips to file
    # clipPath: absolute path to clip file
    def saveClipsToCSV(self, clipPath):
        currentRunDict = self.hierarchy[self.selectedBlockID][self.selectedRunID]
        with open(clipPath, "w") as clipFile:
            # saves a possibly shorter list of saved clips than the original
            for clipKey in currentRunDict["clips"].keys():
                # get frame numbers defining the clip
                start, end = self.clipTextToNumbers(currentRunDict["clips"][clipKey])
                clipData = [start, end]
                # get label if one exists
                if clipKey in currentRunDict["labels"].keys():
                    clipData.append(currentRunDict["labels"][clipKey])
                self.writeClipToCSV(clipData, clipFile)

            # saves unsaved clips
            for clipData in self.unsavedClipDict.values():
                self.writeClipToCSV(clipData, clipFile)
            self.unsavedClipDict = {} # clears any previous edited clips
            self.resetEditingBox() # clear unsaved clips and edit box
            self.resetViewingBox() # reset clip menu

    # writes one clip to CSV file
    # clipData: list of data for clip
    # file: file object to write to
    def writeClipToCSV(self, clipData, file):
        outString = ""
        for datum in clipData:
            outString += str(datum)+","
        file.write(outString+"\n")

    ######################################## Converters

    # gets the frame numbers out of a QString
    # clipText: QString to convert from
    # return frame numbers of start and end frames of clip
    def clipTextToNumbers(self, clipText):
        clipFrames = re.findall("\d+", clipText) # gets the numbers from the clip label
        return int(clipFrames[0]), int(clipFrames[1])

    # changes a QString holding frame numbers to percents of the run
    # menuText: QString to convert from
    # return percents of start and end frames from conversion
    def menuTextToPercent(self, menuText):
        start, end = self.clipTextToNumbers(menuText) # gets the numbers from the clip label
        startPercent = self.frameToPercent(start)
        endPercent = self.frameToPercent(end)
        return (startPercent, endPercent)

    # changes a frame number to a percent of the run
    # frame: frame number to convert from
    # return percent from conversion
    def frameToPercent(self, frame):
        if self.maxFrame: # if maxFrame != 0
            return (frame*100/self.maxFrame)
        else: # maxFrame = 0
            return 0

    ######################################## Processing

    # this starts the windows for viewing the footage
    def startAnimation(self):
        self.sendToServer("startOpenRave", args=(self.setupPath))
        self.sendToOR("startOpenRave")

    def close(self):
        self.sendToServer("closeAll")

    def closeEvent(self, event):
        self.close()

    def sendToServer(self,command,args=None):
        self.pipeServer.send([command,args])

    def sendToOR(self,command,args=None):
        self.pipeOR.send([command,args])

    def HandleCallback(self,msg):
        if(len(msg) == 2):
            if(msg[0] is not None):
                self.updateTeOutput(msg[1][0])
                try:
                    msg[0](msg[1][1])
                except Exception as e:
                    logger.error(str(e))
            else:
                self.updateTeOutput("ERROR: "+msg[1][0])
            return
        logger.error("ERROR in request format")

    ######################################## ROS, requires roscore running

    # used to get max frames from OpenRave
    # key: used by OpenRave to interpret current function response
    def addTwoIntsClient(self, key):
        rospy.wait_for_service('addTwoInts')
        try:
            addTwoInts = rospy.ServiceProxy('addTwoInts', AddTwoInts)
            response = addTwoInts(key, 0)
            return response.sum
        except rospy.ServiceException, e:
            return 0

# Control server to run the benchmark in its own process
class Server(object):

    # Setup the shared memory data structure model and initialize the control parts
    def __init__(self, xmlPath):
        self.xmlPath = xmlPath
        self.running = True
        self.orgui = None
        self.qtgui = None
        (self.pipeQtControl, self.pipeDrawerControl) = Pipe() # pipe between GUI and OR
        (self.pipeQtServer, self.pipeServer) = Pipe() # pipe between GUI and Server

        self.StartQtGuiControl() # starts GUI
        self.run() # starts main server loop which waits for input

    # initializes the GUI process
    def StartQtGuiControl(self): # step one of server
        self.qtgui = Process(target=self._StartQtGuiControl)
        self.qtgui.start()

    # process containing the GUI
    def _StartQtGuiControl(self):
        app = QtGui.QApplication(sys.argv)
        form = MainGUI(self.pipeQtControl,self.pipeQtServer, self.xmlPath)
        form.show()
        app.exec_() # Main loop of the QT GUI

    # starts the OpenRave window but does not populate it
    # setupFilePath: path to the setup.csv file for config settings
    def startOpenRave(self, setupFilePath):
       if self.orgui:
           self.orgui.terminate()
       setup = read_setup(setupFilePath)  # gets the configuration data of the experiment
       NB_HUMAN    = setup[0]
       ELBOW_PADS  = setup[1]
       RARM_ONLY   = setup[2]
       # gets the number of markers based on config data
       NB_MARKERS  = get_nb_markers(ELBOW_PADS, RARM_ONLY)
       # last step of Server for start up
       self.orgui = Process(target=OpenRaveWindow,args=(self.pipeDrawerControl, NB_MARKERS, NB_HUMAN, ELBOW_PADS, RARM_ONLY))
       self.orgui.start()

    # Main server loop which waits for and handles input from the GUI
    def run(self):
        while(self.running): # wait for input all the time
            (functionName,args) = self.pipeServer.recv() # command from GUI
            self.executeFunction(functionName, args) # execute requested command

    # handles all commands requested of the Server
    # command:  name of the command to run
    # args:     tuple of any arguments possibly needed by the command to run
    def executeFunction(self,command, args):
        if command == "startOpenRave": # open the OpenRave window
            self.startOpenRave(args)
        if command == "closeAll": # last step in ending scenario
            self.running = False
            try:
                self.qtgui.terminate()
                self.orgui.terminate()
            except:
                pass

# main function for starting GUI
def main():
    if len(sys.argv) == 2: # if the user enters the xml path in the command line
        xmlPath = sys.argv[1] # store the xml path to pass on to the Server
    else:
        Tk().withdraw() # keeps the root window from appearing
        xmlPath = askopenfilename() # shows an "Open" dialog box and returns the path to the xml

    server = Server(xmlPath)

if __name__ == "__main__":
    main() # sends to main
