#!/usr/bin/python
import rospy
import os, sys
from os import listdir
from os.path import isfile, join

class Segmenter():

    def __init__(self, m_file, o_file, img_dir, drawer, split=None):
        self.m_file = m_file
        self.o_file = o_file
        self.drawer = drawer        
        
        if split == None:
            self.prefix = 0 
            self.drawer.load_file(self.m_file, self.o_file)           
        else:            
            self.prefix = split[0]
            self.drawer.load_range(self.m_file, self.o_file, split)

        self.max = len(self.drawer.frames)
        self.curr = 0        

        self.split = [0,0]
        self.splits = []

        self.images = []
        self.img_dir = img_dir
        self.curr_img = ""

    def change_frame(self, delta):
        self.curr += delta

        if self.curr < 0:
            self.curr = 0
        if self.curr >= self.max:
            print "Reached End Of File"
            self.curr = self.max-1             

        last = ""

        # Load image
        frame = self.drawer.get_frame(self.curr)
        curr_time = frame.get_time()

        for t, img in self.images:
            if t > curr_time:
                last = img
                break

        self.curr_img = os.path.join(self.img_dir, last)

    def load_images(self):
        images = [f for f in listdir(self.img_dir) if isfile(join(self.img_dir,f)) and ".png" in f]
        print "Num images loaded : ", len(images)
        times = []

        for img in images:
            sec, nsec = img.split("_")
            times.append(rospy.Time(int(sec), int(nsec.split(".")[0]) ).to_sec())

        tuple_list = zip(times, images)
        sorted_images = sorted(tuple_list, key = lambda p : p[0])
        self.images = sorted_images

    def save_splits(self):
        folder, path = os.path.split(self.m_file)
        split_outpath = os.path.join(folder, "Splits.csv")           
        with open(split_outpath, "a") as s_file:            
            print "saving splits to :", split_outpath
            for split in self.splits:
                out_str = ""
                for element in split:
                    out_str += str(element)+"," 
                print out_str
                s_file.write(out_str+"\n")
        
