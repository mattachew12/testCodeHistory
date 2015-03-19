#!/usr/bin/python

# Rafi Hayne

from openravepy import *
from MocapCommon import *
import transformation_helper
import csv
import sys


# TMP ROS STUFF
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from os import listdir
from os.path import isfile, join

class Human():

    def __init__(self, marker_list, object_list, elbow_pads, r_arm_only):
        self.markers = marker_list
        self.objects = object_list
        self.elbow_pads = elbow_pads
        self.r_arm_only = r_arm_only

    def get_center_points(self):
        # Get the center points
        Chest = (self.markers[0].array + self.markers[1].array)/2
        Sternum = (self.markers[2].array + self.markers[3].array)/2
        rShoulder = (self.markers[4].array + self.markers[5].array)/2
        if self.elbow_pads:
            rElbow = self.objects[2].array
            rWrist = (self.markers[6].array + self.markers[7].array)/2
            rPalm = self.markers[8].array
            if not self.r_arm_only:
                lShoulder = (self.markers[9].array + self.markers[10].array)/2
                lElbow = self.objects[3].array
                lWrist = (self.markers[11].array + self.markers[12].array)/2
                lPalm = self.markers[13].array
        else:
            rElbow = (self.markers[6].array + self.markers[7].array)/2
            rWrist = (self.markers[8].array + self.markers[9].array)/2
            rPalm = self.markers[10].array
            if not self.r_arm_only:
                lShoulder = (self.markers[11].array + self.markers[12].array)/2
                lElbow = (self.markers[13].array + self.markers[14].array)/2
                lWrist = (self.markers[15].array + self.markers[16].array)/2
                lPalm = self.markers[17].array

        Pelvis = self.objects[0].array
        Head = self.objects[1].array

        # Return the points
        centers = [Chest, Sternum, rShoulder, rElbow, rWrist, rPalm]
        if not self.r_arm_only:
            lArm = [lShoulder, lElbow, lWrist, lPalm]
            centers += lArm
        centers += [Pelvis, Head]

        return np.array(centers)

class Drawer():

    def __init__(self, nb_markers, nb_human, elbow_pads, r_arm_only):

        self.nb_human = nb_human
        self.elbow_pads = elbow_pads
        self.r_arm_only = r_arm_only
        self.nb_markers = nb_markers
        self.nb_objects = get_nb_objects(self.elbow_pads, self.r_arm_only)

        self.env = Environment()
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.SetViewer("qtcoin")

        self.handles = []
        self.frames = []


    def clear_frames(self):
        self.frames = []

    def load_file(self, m_filepath, o_filepath):
        #print "Trying to open file"
        # global NB_HUMAN # TODO fix global to be class member

        marker_file = []
        object_file = []

        with open(m_filepath, 'r') as m_file:
            with open(o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_frame = nb_lines

        self.frames = []

        for row in range(nb_lines):

            self.load_helper(row, marker_file, object_file)

    # Split is a tuple of the form (start_frame, end_frame)
    def load_range(self, m_filepath, o_filepath, split):
        #print "Trying to open split"
        # global NB_HUMAN # TODO fix global to be class member

        marker_file = []
        object_file = []

        with open(m_filepath, 'r') as m_file:
            with open(o_filepath, 'r') as o_file:

                marker_file = [row for row in csv.reader(m_file, delimiter=',')]
                object_file = [row for row in csv.reader(o_file, delimiter=',')]

        nb_lines = min(len(marker_file), len(object_file))
        self.last_frame = nb_lines

        for row in range(split[0], split[1]):
            self.load_helper(row, marker_file, object_file)
            
    def load_helper(self, row, marker_file, object_file):
        markers = []
        objects = []

        m_cells = marker_file[row]
        o_cells = object_file[row]

        # Load Objects
        count = int(o_cells[2])

        # Assuming only using Pelv/Head objects per person and nothing else in the scene
        # NB_HUMAN = count/2

        for i in range(3, count*9, 9):
            name = str(o_cells[i])
            occluded = int(o_cells[i+1])
            x = float(o_cells[i+2])
            y = float(o_cells[i+3])
            z = float(o_cells[i+4])
            r_x = float(o_cells[i+5])
            r_y = float(o_cells[i+6])
            r_z = float(o_cells[i+7])
            r_w = float(o_cells[i+8])

            object = Object( name, occluded, x, y, z, r_x, r_y, r_z, r_w )
            objects.append(object)

        # Load Markers
        sec = float(m_cells[0])
        nsec = float(m_cells[1])
        count = int(m_cells[2])

        for i in range(3, count*4, 4):
            id = m_cells[i]
            x = float(m_cells[i+1])
            y = float(m_cells[i+2])
            z = float(m_cells[i+3])
            marker = Marker(id, x, y, z)
            markers.append(marker)

        self.frames.append( Frame(sec, nsec, count, markers, objects))

    def play_raw(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        for i, frame in enumerate(self.frames):
            curr_time = frame.get_time()
            self.draw_frame_raw(frame)
            self.draw_frame_axes(frame)

            dt = curr_time - prev_time
            prev_time = curr_time
            time.sleep(dt)
            self.clear()


        return

    def play_skeleton(self):
        # for frame in self.frames:
        prev_time = self.frames[0].get_time()

        # IMAGE PUBLISHER
        folder = "/home/rafi/logging_ten/1"
        images = [ f for f in listdir(folder) if isfile(join(folder,f)) and '.png' in f ]
        #print "Num images : ", len(images)
        times = []

        for img in images:
            sec, nsec = img.split('_')
            times.append( rospy.Time(int(sec), int(nsec.split('.')[0]) ).to_sec() )

        tuple_list = zip(times, images)
        sorted_images = sorted(tuple_list, key = lambda p : p[0])

        for i, frame in enumerate(self.frames):
            curr_time = frame.get_time()

            self.draw_frame_skeleton(frame)

            dt = curr_time - prev_time
            # dt = 0.05
            prev_time = curr_time
            time.sleep(dt)


            if True:
            # if i % 20 == 0 :
                last = None

                try:
                    for t, img in sorted_images:
                        if t > curr_time:
                            # image_name = folder + "/" + img
                            break

                        last = img

                    if last:
                        image_name = folder + "/" + last
                        #print image_name

                        cv_image = cv2.imread( image_name, 1)
                        cv2.imshow('Camera Data', cv_image)
                        #print image_name
                except e:
                    pass
                    #print e


                if i == 0:
                    sys.stdin.readline()

            self.clear()

        return

    def isolate_humans(self, frame):
        humans = []

        for i in range(self.nb_human):
            # Ugly ;(
            temp_markers = frame.marker_list[i*self.nb_markers:i*self.nb_markers+self.nb_markers]
            temp_objects = frame.object_list[i*self.nb_objects:i*self.nb_objects+self.nb_objects]

            humans.append( Human(temp_markers, temp_objects, self.elbow_pads, self.r_arm_only) )

        return humans


    def get_frame(self, frame_id):
        return self.frames[frame_id]

    def draw_frame_skeleton(self, frame):
        humans = self.isolate_humans(frame)
        # self.draw_frame_axes(frame)
        self.draw_frame_raw(frame)

        for h in humans:
            centers = h.get_center_points()
            self.draw_skeleton(centers)

    def draw_frame_raw(self, frame):
        point_list = []

        for m in frame.marker_list:
            point_list.append(m.array)
        for o in frame.object_list:
            if o:
                point_list.append(o.array)

        self.draw_points(np.array(point_list))

    def draw_point_by_name(self, frame, name):
        point_list = []

        for m in frame.marker_list:
            if name in m.id:
                point_list.append(m.array)

        self.draw_point_isolated(np.array(point_list))

    def draw_point_isolated(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in range(len(point_list)):
            if n == 17:
                colors.append((1,0,0))
            else:
                colors.append((0,0,1))

        self.handles.append(self.env.plot3(points=point_list, pointsize=0.05, colors=array(colors), drawstyle=1))


    def draw_points(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            # colors.append((0,0,1))
            colors.append((float(n)*1, (1-float(n))*1, 0))

        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))


    def draw_skeleton(self, point_list):
        points = point_list

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        # Marker points
        self.handles.append(self.env.plot3(points=point_list, pointsize=0.02, colors=array(colors), drawstyle=1))

        if not self.r_arm_only:

            # Connect shoulders
            shoulders = np.array([point_list[2],point_list[6]])
            self.handles.append(self.env.drawlinestrip(points=shoulders, linewidth=3.0))

            # Connect head
            head = np.array( [ point_list[1], point_list[11] ] )
            self.handles.append(self.env.drawlinestrip(points=head, linewidth=3.0))

            # Connect right arm
            right_arm = np.array([point_list[2],point_list[3], point_list[4], point_list[5]])
            self.handles.append(self.env.drawlinestrip(points=right_arm, linewidth=3.0))

            # Left Arm
            left_arm = np.array( [point_list[6], point_list[7], point_list[8], point_list[9]] )
            self.handles.append(self.env.drawlinestrip(points=left_arm, linewidth=3.0))

            # Pelvis
            pelv = np.array( [point_list[2], point_list[10], point_list[6]] )
            self.handles.append(self.env.drawlinestrip(points=pelv, linewidth=3.0))

        else:
            # Connect Shoulders
            shoulders = np.array([point_list[2],point_list[1]])
            self.handles.append(self.env.drawlinestrip(points=shoulders, linewidth=3.0))

            # Connect head
            head = np.array( [ point_list[1], point_list[7] ] )
            self.handles.append(self.env.drawlinestrip(points=head, linewidth=3.0))

            # Connect right arm
            right_arm = np.array([point_list[2],point_list[3], point_list[4], point_list[5]])
            self.handles.append(self.env.drawlinestrip(points=right_arm, linewidth=3.0))

            # Pelvis
            pelv = np.array( [point_list[2], point_list[1], point_list[6], point_list[2]] )
            self.handles.append(self.env.drawlinestrip(points=pelv, linewidth=3.0))

    def draw_object_axes(self, object):
        # tf = MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([object.r_w, object.r_x, object.r_y, object.r_z]) )), transpose(matrix([object.x, object.y, object.z])) )

        tf = object.get_transform()
        self.handles.append(misc.DrawAxes( self.env, matrix(tf), 0.5 ))

    def draw_frame_axes(self, frame):
        for o in frame.object_list:
            if not o.is_occluded():
                self.draw_object_axes(o)

    def clear(self):
        del self.handles[:]