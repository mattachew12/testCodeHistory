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
import cv2

from Segmenter import *
from QTMocapDrawer import *

import rospy
from mocapGUI.srv import *

FRAME_RATE = 5 # 5 hundredths of a second, fps = 20

# TODO: connect functions in this class to MainGUI
# handles all functions relevant to the OpenRave window
class OpenRaveWindow(Drawer):

    # connect pipes and initialize OpenRave window
    def __init__(self, pipeDrawerControl, marker_list, object_list, elbow_pads, r_arm_only):
        rospy.init_node('openRaveNode') # only one at a time allowed in the same process
        Drawer.__init__(self, marker_list, object_list, elbow_pads, r_arm_only)
        self.pipe = pipeDrawerControl
        self.running = True
        self.drawerStarted = False
        self.segmenter = False
        self.paused = True # whether or not to play the footage as a movie
        self.lastFramePlay = 0 # last section of time when the footage was played
        self.secPerFrame = FRAME_RATE # inverted frame rate, hundredths of a sec
        self.changeFrameServer()
        self.drawOpenRaveServer()
        self.pureFunctionCallServer()
        self.run()

    # inherent function for closing the window, not specifically called anywhere
    def __del__(self):
        self.drawerStarted = False
        RaveDestroy()

    # main loop of OpenRave window
    def run(self):
        while(self.running):
            (functionName,args) = self.pipe.recv() # receive command from GUI
            self.executeFunction(functionName, args) # execute requested command

            # TODO: figure out why this while loop only runs when a command is sent through pipe
            # self.checkToPlayFootage() # checks whether it should play a movie or not
            if self.drawerStarted and self.segmenter: # don't populate window if drawer is not started
                cv_image = cv2.imread(self.segmenter.curr_img, 1)
                if cv_image is not None: # try to display img only if it exists
                    cv2.imshow("Camera Data", cv_image)

                self.clear() # Drawer: clear
                frame = self.get_frame(self.segmenter.curr) # Drawer: get_frame

                self.draw_frame_skeleton(frame) # Drawer

    # main action of OpenRave window without loop
    def mainAction(self):
        if self.drawerStarted and self.segmenter: # don't populate window if drawer is not started
            cv_image = cv2.imread(self.segmenter.curr_img, 1)
            if cv_image is not None: # try to display img only if it exists
                cv2.imshow("Camera Data", cv_image)

            self.clear() # Drawer: clear
            frame = self.get_frame(self.segmenter.curr) # Drawer: get_frame

            self.draw_frame_skeleton(frame) # Drawer

    # run function requested by external process
    def executeFunction(self,name,args):
        if name in dir(self):
            if args is None:
                getattr(self,name)()
            else:
                getattr(self,name)(args)

    def checkToPlayFootage(self):
        curTime = int(time.clock() * 100) # truncate to 2 decimals
        timeInterval = curTime % self.secPerFrame # play at timeInterval = 0
        if not self.paused and self.lastFramePlay != curTime: # if it should be playing
            print "Time interval, checkToPlayFootage in OpenRaveWindow class ", timeInterval
            if timeInterval == 0: # if the time has passed between frames
                self.segmenter.change_frame(1)
                self.lastFramePlay = curTime

    # initialize OpenRave window
    def startDrawingOpenRave(self, (markerFile, objFile, imgDir)):
        self.segmenter = Segmenter(markerFile, objFile, imgDir, self)
        self.env.Reset()
        t_cam = array([[ -0.655253290114, -0.106306078558, 0.747891799297, -0.302201271057] ,
                                [ -0.725788890663, 0.363116971923, -0.584274379801, 2.68592453003] ,
                                [ -0.209460287369, -0.925659269042, -0.315089361376, 2.25037527084] ,
                                [ 0.0, 0.0, 0.0, 1.0]])
        self.env.GetViewer().SetCamera(t_cam) # reset camera on change
        self.segmenter.load_images()  #  prepares images for this run

        # creates the video feed data
        cv2.namedWindow("Camera Data", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Camera Data", 700, 50)
        self.drawerStarted = True

    # removes footage from OpenRave window
    def clearOpenRave(self):
        self.drawerStarted = False

    # gets the number of frames in the current split
    def getMaxFrames(self):
        time.sleep(.1) # gives time for drawing to create a new segmenter if needed
        if self.segmenter:
            return self.segmenter.max
        else:
            return 0

    # exact frame shift control, overlay for segmenter function
    # delta: exact number of frames to shift the footage by
    def changeFrame(self, delta):
        self.segmenter.change_frame(delta)

    # TODO: Figure out timing an action without freezing the Process
    # plays through footage like a movie
    # frameRate: rate to play through at
    def playFootage(self, fps=20):
        print "play"
        self.paused = False
        self.secPerFrame = (100/fps) # truncates to hundredths of seconds

    # pauses playing through footage like a movie
    def pauseFootage(self):
        print "pause"
        self.paused = True

    ############################## ROS, requires roscore running

    # listens for ChangeFrame requests from MocapGUI
    def changeFrameServer(self):
        rospy.Service('changeFrameService', ChangeFrame, self.handleChangeFrame)

    # changes frame as requested by MocapGUI
    # req: initial request instance of ChangeFrame
    # returns instance of ChangeFrame response
    def handleChangeFrame(self, request):
        self.changeFrame(request.frameChangeDelta)
        self.mainAction() # update drawing
        return ChangeFrameResponse()

    # listens for DrawOpenRave requests from MocapGUI
    def drawOpenRaveServer(self):
        rospy.Service('drawOpenRaveService', DrawOpenRave, self.handleDrawOpenRave)

    # starts drawing mocap footage in OpenRave as requested by MocapGUI
    # req: initial request instance of DrawOpenRave
    # returns instance of DrawOpenRave response
    def handleDrawOpenRave(self, request):
        argTuple = (request.markerFile, request.objectFile, request.imgDir)
        self.startDrawingOpenRave(argTuple)
        self.mainAction() # update drawing
        return DrawOpenRaveResponse()

    # listens for function calls with no arguments or return values
    def pureFunctionCallServer(self):
        rospy.Service('pureFunctionCall', PureFunctionCall, self.handlePureFunctionCall)

    # calls functions as requested by MocapGUI
    # req: initial request instance of PureFunctionCall
    # returns the called function's returnVal to MocapGUI
    def handlePureFunctionCall(self, request):
        returnVal = getattr(self, request.functionName)() # call function
        self.mainAction() # update drawing
        return PureFunctionCallResponse(returnVal) # send back returnVal