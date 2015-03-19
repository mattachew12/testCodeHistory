#!/usr/bin/env python
#-*- coding:utf-8 -*-

from PyQt4 import QtCore
from PyQt4.QtGui import *

class Highlight(QWidget):
    # initialize variables and set painting attributes
    def __init__(self, width, parent = None):
        super(Highlight, self).__init__(parent)
        palette = QPalette(self.palette())
        palette.setColor(palette.Background, QtCore.Qt.transparent)
        self.setPalette(palette)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.rectList = [] # list of clip percents to outline
        self.currentClipIndex = -1 # current clip is not yet in rectList

        # width of slider
        self.width = width-10

    # overwrites default paintEvent function of widget
    # event: event called by widget during updates, not called by programmer
    def paintEvent(self, event):
        painter = QPainter()
        painter.begin(self)
        for index in range(len(self.rectList)):
            # converts percents in rectList to pixels
            startPixel, width = self.percentToPixel(self.rectList[index])
            if index == self.currentClipIndex:
                rect = QtCore.QRectF(startPixel, 0, width, 15) # highlight box
                painter.fillRect(rect, QBrush(QColor(0, 255, 255, 100))) # blue highlight

            painter.drawRect(startPixel, 0, width, 15) # box outline all clips
        painter.setPen(QPen(QtCore.Qt.NoPen))

    # adds a saved clip to be outlined on the slider
    # startPercent: percent of slider to highlight from
    # endPercent: percent of slider to highlight to
    def addClipRect(self, (startPercent, endPercent)):
        self.rectList.append((startPercent, endPercent)) # outlined box
        self.update()

    # sets the clip to be highlighted
    # startPercent: percent of slider to highlight from
    # endPercent: percent of slider to highlight to
    def setCurrentClip(self, (startPercent, endPercent)):
        self.currentClipIndex = self.rectList.index((startPercent, endPercent))
        self.update()

    # removes the clip from the list of outlined boxes
    # startPercent: percent of slider to highlight from
    # endPercent: percent of slider to highlight to
    def removeClip(self, (startPercent, endPercent)):
        self.rectList.remove((startPercent, endPercent))
        self.update()

    # resets the current clip index to no clip
    def resetCurrentClip(self):
        self.currentClipIndex = -1 # index not in list
        self.update()

    # resets the list of saved clips
    def resetClips(self):
        self.rectList = []
        self.update() # repaints the slider

    # converts from percents to pixels
    # startPercent: percent of slider to highlight from
    # endPercent: percent of slider to highlight to
    # returns the pixels on the highlight widget corresponding to the percents
    def percentToPixel(self, (startPercent, endPercent)):
        startPixel = startPercent*self.width/100
        width = (endPercent*self.width/100) - startPixel
        return (startPixel+5, width) # shifted because slider doesn't start at 0