#!/usr/bin/python

# Rafi Hayne

from TransformMatrix import *
import openravepy
import collections
import numpy as np
import time
from openravepy import *
import transformation_helper
import rospy
import copy
import csv


global NB_HUMAN
global ELBOW_PADS
global RARM_ONLY
global NB_MARKERS

class Timer:
    def __enter__(self):
        self.start = time.clock()
        self.file_runtime = None
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start

class Marker:
    def __init__(self, id, x, y, z, name=''):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.array = np.array([self.x, self.y, self.z])
        self.times_dropped = 0

    def is_same_marker(self, a_marker, threshold):
        if self.get_dist(a_marker) > threshold:
            return False
        else:
            return True

    def numpy(self):
        return self.array

    def get_dist(self, other):
        diff = self.array - other.array
        return np.dot(diff, diff.conj())

    def get_true_dist(self, other):
        diff = self.array - other.array
        return np.sqrt(np.dot(diff,diff))

class Object:
    def __init__(self, id, occluded, x, y, z, r_x, r_y, r_z, r_w):
        self.id = id
        self.occluded = occluded
        self.x = x
        self.y = y
        self.z = z
        self.r_x = r_x
        self.r_y = r_y
        self.r_z = r_z
        self.r_w = r_w
        self.array = np.array([self.x, self.y, self.z])

    def is_occluded(self):
        return self.occluded

    def get_transform(self):

        # ''' TEST '''
        # raw_quaternion = array([self.r_w, self.r_x, self.r_y, self.r_z])
        # print "Raw quaternion: " + str(raw_quaternion)
        # raw_quaternion_norm = np.linalg.norm(raw_quaternion)
        # print "Raw norm: " + str(raw_quaternion_norm)
        # normalized_quaternion = raw_quaternion / raw_quaternion_norm
        # print "Normalized quaternion: " + str(normalized_quaternion)
        # normalized_quaternion_norm = np.linalg.norm(normalized_quaternion)
        # print "Normalized norm: " + str(normalized_quaternion_norm)
        # ''' /TEST '''

        mat =  MakeTransform( rotationMatrixFromQuat( array(transformation_helper.NormalizeQuaternion([self.r_w, self.r_x, self.r_y, self.r_z]) )), transpose(matrix([self.x, self.y, self.z])) )

        # TODO figure out if this works on all Pelvis frames.  If not, why?
        if 'Pelvis' in self.id:
            # x_dir = np.array(np.transpose(mat[:,0]).tolist()[0][:3])
            # y_dir = np.array(np.transpose(mat[:,1]).tolist()[0][:3])
            # z_dir = np.array(np.transpose(mat[:,2]).tolist()[0][:3])

            # new_x = -z_dir
            # new_x[2] = 0
            # new_x = new_x/np.linalg.norm(new_x)

            # new_z = np.array([0,0,1])

            # new_y = np.cross(new_x, new_z)
            # new_y = new_y/np.linalg.norm(new_y)
            # new_y = -new_y

            # mat[0][0, 0] = new_x[0]
            # mat[0][0, 1] = new_y[0]
            # mat[0][0, 2] = new_z[0]
            # mat[1][0, 0] = new_x[1]
            # mat[1][0, 1] = new_y[1]
            # mat[1][0, 2] = new_z[1]
            # mat[2][0, 0] = new_x[2]
            # mat[2][0, 1] = new_y[2]
            # mat[2][0, 2] = new_z[2]


            # ---------------------------------
            x_dir = np.array(np.transpose(mat[:,0]).tolist()[0][:3])
            y_dir = np.array(np.transpose(mat[:,1]).tolist()[0][:3])
            z_dir = np.array(np.transpose(mat[:,2]).tolist()[0][:3])

            new_z = np.array([0,0,1])
            new_x = np.cross(y_dir, new_z)
            new_y = np.cross(new_z, new_x)

            new_x = new_x/np.linalg.norm(new_x)
            new_y = new_y/np.linalg.norm(new_y)
            new_z = new_z/np.linalg.norm(new_z)

            t_rot = np.transpose( np.matrix([new_x, new_y, new_z]) )
            mat = MakeTransform( t_rot, np.matrix([self.x, self.y, self.z]))

        return mat

class Frame:
    def __init__(self, t_sec, t_nsec, nb_markers, marker_list, object_list):
        self.sec = t_sec
        self.nsec = t_nsec
        self.count = nb_markers
        self.marker_list = marker_list
        self.object_list = object_list

    # Can not have any duplicate ids
    def order_markers(self):
        self.marker_list.sort(key=lambda m: m.id, reverse=False)

    def get_marker_by_id(self, id):
        for marker in self.marker_list:
            if marker.id == id:
                return marker

        print "Couldn't find marker with id : ", id
        return None

    def get_marker_by_name(self, name):
        for m in self.marker_list:
            if m.name == name:
                return m

        print "Couldn't find marker with name : ", name
        return None

    def reorder_objects(self, nb_human, elbow_pads, r_arm_only):
        new_list = []
        for i in range(nb_human):
            new_list.append( self.get_object_by_id("Pelvis" + str(i)) )
            new_list.append( self.get_object_by_id("Head" + str(i)) )

            if elbow_pads:
                new_list.append( self.get_object_by_id("rElbow" + str(i)) )
                if not r_arm_only:
                    new_list.append( self.get_object_by_id("lElbow" + str(i)) )


        # HACK for messed up object ids in third and fourth logging data
        # new_list.append( self.get_object_by_id("Pelvis0") )
        # new_list.append( self.get_object_by_id("Head0") )
        # new_list.append( self.get_object_by_id("rElbow0") )
        # new_list.append( self.get_object_by_id("Pelvis1") )
        # new_list.append( self.get_object_by_id("Head2") )
        # new_list.append( self.get_object_by_id("rElbow1") )

        self.object_list = new_list

    def get_object_by_id(self, id):
        for obj in self.object_list:
            if obj.id == id:
                return obj

    def print_marker_ids(self):
        marker_string = '[ '

        for marker in self.marker_list:
            if marker is None:
                marker_string += 'None' + ' '
            else:
                marker_string += str(marker.id) + ' '
        marker_string += ']'
        print marker_string

    # def is_similar_frame(self, other):
    #     if self.count != other.count or self.count != N_MARKERS:
    #         return False

    #     for i in range(self.count):

    #         if self.marker_list[i] is None:
    #             return False

    #         if not self.marker_list[i].is_same_marker(other.marker_list[i], THRESHOLD):
    #             return False

    #     return True

    # Returns a list of tuples sorted by lowest value (distance)
    # [ ( key, val), ... etc
    def get_distances(self, other):
        dist = {}
        for marker in self.marker_list:
            # dist[marker.id] = random.random()
            dist[marker.id] = marker.get_dist(other)
        return sorted(dist.items(), key=lambda(k, v): v)

    def get_duplicate_ids(self):
        indices = []

        for marker in self.marker_list:
            if not marker == None:
                indices.append(marker.id)

        return [x for x, y in collections.Counter(indices).items() if y > 1]

    def get_true_threshold(self, id, thresh):
        marker = self.marker_list[id]

        return thresh + thresh*marker.times_dropped

    def get_new_config_by_distance(self, prev_frame, thresh):
        # print "i : ", prev_index+1
        new_markers = [None]*prev_frame.count
        shortest_found = [float('inf')]*prev_frame.count

        # self.print_marker_ids()

        for i, marker in enumerate(self.marker_list):

            dists = prev_frame.get_distances(marker)

            for closest in dists:
                closest_id = closest[0]
                closest_dist = closest[1]

                if shortest_found[closest_id] > closest_dist and closest_dist < prev_frame.get_true_threshold(closest_id, thresh) and closest_dist != 0:
                    new_markers[closest_id] = Marker(closest_id, marker.x, marker.y, marker.z, prev_frame.marker_list[closest_id].name)
                    new_markers[closest_id].times_dropped = 0
                    shortest_found[closest_id] = closest_dist
                    break

        for i in range(0, prev_frame.count):
            if new_markers[i] is None:  # Marker dropped!
                new_markers[i] = copy.deepcopy(prev_frame.get_marker_by_id(i))
                new_markers[i].times_dropped += 1

        # Fix occluded objects
        for i, o in enumerate(self.object_list):
            if o.is_occluded():
                self.object_list[i] = prev_frame.object_list[i]


        temp = Frame(self.sec, self.nsec, len(new_markers), new_markers, self.object_list)
        return temp

    def set_marker_names_by_id(self):
        for marker in self.marker_list:
            if marker.id == 0:
                marker.name = 'ChestFront'
            if marker.id == 1:
                marker.name = 'ChestBack'
            if marker.id == 2:
                marker.name = 'SternumFront'
            if marker.id == 3:
                marker.name = 'SternumBack'
            if marker.id == 4:
                marker.name = 'rShoulderFront'
            if marker.id == 5:
                marker.name = 'rShoulderBack'
            if marker.id == 6:
                marker.name = 'rElbowOuter'
            if marker.id == 7:
                marker.name = 'rElbowInner'
            if marker.id == 8:
                marker.name = 'rWristOuter'
            if marker.id == 9:
                marker.name = 'rWristInner'
            if marker.id == 10:
                marker.name = 'rPalm'
            if marker.id == 11:
                marker.name = 'lShoulderFront'
            if marker.id == 12:
                marker.name = 'lShoulderBack'
            if marker.id == 13:
                marker.name = 'lElbowOuter'
            if marker.id == 14:
                marker.name = 'lElbowInner'
            if marker.id == 15:
                marker.name = 'lWristOuter'
            if marker.id == 16:
                marker.name = 'lWristInner'
            if marker.id == 17:
                marker.name = 'lPalm'

    def set_marker_names(self, nb_human, nb_markers, elbow_pads, r_arm_only):
        true_count = nb_markers*nb_human
        if (self.count is not true_count):
            print "Something went wrong when setting marker names"
            return


        if elbow_pads:
            for i in range(0, true_count, nb_markers):
                self.marker_list[i].name = 'ChestFront'
                self.marker_list[i+1].name = 'ChestBack'
                self.marker_list[i+2].name = 'SternumFront'
                self.marker_list[i+3].name = 'SternumBack'
                self.marker_list[i+4].name = 'rShoulderFront'
                self.marker_list[i+5].name = 'rShoulderBack'
                self.marker_list[i+6].name = 'rWristOuter'
                self.marker_list[i+7].name = 'rWristInner'
                self.marker_list[i+8].name = 'rPalm'
                if not r_arm_only:
                    self.marker_list[i+9].name = 'lShoulderFront'
                    self.marker_list[i+10].name = 'lShoulderBack'
                    self.marker_list[i+11].name = 'lWristOuter'
                    self.marker_list[i+12].name = 'lWristInner'
                    self.marker_list[i+13].name = 'lPalm'
        else:
            for i in range(0, true_count, nb_markers):
                self.marker_list[i].name = 'ChestFront'
                self.marker_list[i+1].name = 'ChestBack'
                self.marker_list[i+2].name = 'SternumFront'
                self.marker_list[i+3].name = 'SternumBack'
                self.marker_list[i+4].name = 'rShoulderFront'
                self.marker_list[i+5].name = 'rShoulderBack'
                self.marker_list[i+6].name = 'rElbowOuter'
                self.marker_list[i+7].name = 'rElbowInner'
                self.marker_list[i+8].name = 'rWristOuter'
                self.marker_list[i+9].name = 'rWristInner'
                self.marker_list[i+10].name = 'rPalm'
                if not r_arm_only:
                    self.marker_list[i+11].name = 'lShoulderFront'
                    self.marker_list[i+12].name = 'lShoulderBack'
                    self.marker_list[i+13].name = 'lElbowOuter'
                    self.marker_list[i+14].name = 'lElbowInner'
                    self.marker_list[i+15].name = 'lWristOuter'
                    self.marker_list[i+16].name = 'lWristInner'
                    self.marker_list[i+17].name = 'lPalm'

    def get_n_closest_markers(self, pelv_frame, n):
        points = []
        dist = {}

        temp_pelv = np.transpose(pelv_frame[:,3])
        pelv = np.array([temp_pelv[0,0], temp_pelv[0,1], temp_pelv[0,2]])

        for marker in self.marker_list:
            # Only consider distance in x and y
            dist[marker.id] = sqrt( (marker.x-pelv[0])**2 + (marker.y-pelv[1])**2 )

        dist_list = sorted(dist.items(), key=lambda(k, v): v)

        # Return the first n points
        for pair in dist_list[:n]:
            # self.print_marker_ids()
            points.append( ( pair[0] ,self.marker_list[pair[0]].array) )

        return points


    def reorder_ids(self):
        for i, marker in enumerate(self.marker_list):
            marker.id = i

    def get_time(self):
        # return self.sec + (self.nsec/1e10)
        return rospy.Time(int(self.sec), int(self.nsec)).to_sec()

    def get_rostime(self):
        # return self.sec + (self.nsec/1e10)
        return rospy.Time(int(self.sec), int(self.nsec))

    def get_dist_between_frames(self, other):
        dist_list = []

        for i, marker in enumerate(self.marker_list):
            dist_list.append(other.marker_list[i].get_dist(marker))

        return dist_list


def get_nb_markers(elbow_pads, r_arm_only):
    if elbow_pads:
        if r_arm_only:
            nb_markers = 9
        else:
            nb_markers = 14
    else:
        if r_arm_only:
            nb_markers = 11
        else:
            nb_markers = 18

    return nb_markers

def get_nb_objects(elbow_pads, r_arm_only):
    if elbow_pads:
        if r_arm_only:
            nb_objects = 3

        else:
            nb_objects = 4
    else:
        nb_objects = 2

    return nb_objects

def read_setup(folder):
    setup = []

    with open(folder + 'Setup.csv', 'r') as s_file:
        matrix = [row for row in csv.reader(s_file, delimiter=',')]
        for line in matrix:
            setup.append( int(line[0]) )

    return setup

def set_constants(folder):
    setup = read_setup(folder)
    # splits = read_splits('/home/rafi/logging_ten/0/')

    NB_HUMAN    = setup[0]
    ELBOW_PADS  = setup[1]
    RARM_ONLY   = setup[2]
    NB_MARKERS = get_nb_markers(ELBOW_PADS, RARM_ONLY)

def read_splits(folder):
    splits = []

    with open(folder + 'Splits.csv', 'r') as s_file:
        matrix = [row for row in csv.reader(s_file, delimiter=',')]
        for line in matrix:
            splits.append( (int(line[0]), int(line[1])) )

    return splits
# a : a numpy array of [x,y,z]
# b : a numpy array of [x,y,z]
def interpolate( a, b, u ):
     out = copy.deepcopy(a)
     for i in range(len(out)):
         out[i] += u*(b[i]-a[i])
     return out
