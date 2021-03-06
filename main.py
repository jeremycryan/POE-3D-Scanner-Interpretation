#!/usr/local/bin/python

from serial import Serial, SerialException, serialutil
import time
import numpy as np
import io
from math import cos, sin, atan2, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def compile_data(serial_object):
    """ Returns a list of data points for 3D scan, in terms of two axes
    and ultrasonic sensor value.

    Returns list of data points as numpy array. """

    data = []
    points_count = 0

    print("Waiting for button press...")
    repr(serial_object.readline())
    while serial_object.readline() == "" or \
            serial_object.readline() == "begin\n":
        pass

    print("Connection established.")

    while True:
        #   Read the most recent serial output
        try:
            result = serial_object.readline();
            serial_object.flush()
        except:
            break

        #   Convert string into list of integers
        result_split = result.split()
        result_floats_raw = [float(item) for item in result_split]

        #   Take median of all distance measurements
        scans_per_distance = len(result_floats_raw) - 2
        angles = result_floats_raw[0:2]
        distance = np.median(result_floats_raw[2:])
        result_floats = angles + [distance]

        print result_floats
        #   Break out of loop if no data is received, scan is done
        if len(result_floats) <= 1:
            break

        #   Add to list of data if different than last data point
        if not len(data) or (len(data) and (result_floats != data[-1])):

            #   Print progress
            points_count += 1
            print("Points scanned: %s" % points_count)

            distance_threshold = 100
            #   Raw infrared readings less than threshold are removed
            if result_floats[-1] >= distance_threshold:
                data.append(result_floats)

        #   Don't run at max clock speed
        time.sleep(0.005)

    print("Scan complete!")

    #   Convert data to numpy array and return
    return np.asarray(data).T

def process_data(data):
    """ Takes data as a 3xN array, with the rows being servo positions and
    infrared output. Processes this data to represent servo angles in radians
    and distance in m. """

    rads_per_degree = pi/180.0
    processed_angles = data[0:2] * rads_per_degree

    dists = data[2]
    processed_distances = np.asarray([[volt_to_dist(volt) for volt in dists]])

    processed_data_matrix = np.concatenate([processed_angles,
                                            processed_distances],
                                            0)
    return processed_data_matrix

def data_to_3d(data, initial_position, initial_direction):
    """ Converts processed data matrix into a matrix of 3D points. """

    z_angles = data[0]
    x_angles = data[1]
    dists = data[2]

    points = np.zeros([3, len(dists)])

    for i, dist in enumerate(dists):
        vec = data[:, i]
        scan_pos = determine_position(vec, initial_position, initial_direction)
        scan_dir = determine_direction(vec, initial_direction)
        point = determine_coordinate(scan_pos, scan_dir, dist)
        points[0, i] = point[0][0]
        points[1, i] = point[1][0]
        points[2, i] = point[2][0]

    return points


def volt_to_dist(voltage):
    """ Converts infrared voltage signal to distances, based on
    a polynomial fitting of calibration data. """

    x = voltage
    a = -0.0000015744
    b = 0.0019
    c = -0.8718
    d = 168.6367
    return (a*x**3 + b*x**2 + c*x + d)/100.0


def rotate_x(vector, rad):
    """ Rotate a point (vector) about the positive y axis by rad
    radians, returns np array. """

    trans_mat = np.asarray([[1, 0, 0],
                            [0, cos(rad), -sin(rad)],
                            [0, sin(rad), cos(rad)]])
    return np.matmul(trans_mat, vector)

def rotate_z(vector, rad):
    """ Rotate a point (vector) about the positive z axis by rad
    radians, returns np array. """

    trans_mat = np.asarray([[cos(rad), -sin(rad), 0],
                            [sin(rad),  cos(rad), 0],
                            [       0,         0, 1]])
    return np.matmul(trans_mat, vector)

def determine_direction(processed_data_vector, initial_direction):
    """ Data vector should include the positions of each servo (rad) and the
    output of the infrared distance sensor (m). """

    z_difference = atan2(-initial_direction[0], initial_direction[1])
    corrected_for_z = rotate_z(initial_direction, -z_difference)
    after_x_corrected = rotate_x(corrected_for_z, processed_data_vector[1])
    after_x = rotate_z(after_x_corrected, z_difference)
    after_z = rotate_z(after_x, processed_data_vector[0])

    normed = after_z/np.linalg.norm(after_z)
    return normed

def determine_position(processed_data_vector, initial_position, initial_direction):
    """ Data vector should include the positions of each servo (rad) and the
    output of the infrared distance sensor (m). """

    minor_axis_height = 0.02769

    pos_to_secondary_pivot = processed_data_vector - minor_axis_height
    z_difference = atan2(-initial_direction[0], initial_direction[1])
    corrected_for_z = rotate_z(initial_position, -z_difference)
    after_x_corrected = rotate_x(corrected_for_z, processed_data_vector[1])
    after_x = rotate_z(after_x_corrected, z_difference)
    after_x += minor_axis_height
    after_z = rotate_z(after_x, processed_data_vector[0])

    return after_z

def determine_coordinate(position, direction, distance):
    point_position = direction * distance + position
    return point_position

def plot_points(pdata):
    """ Plots 3D points given their 3D coordinates. """

    #   Unpack data from pdata matrix
    xs = pdata[0]
    ys = pdata[1]
    zs = pdata[2]

    #   Plot data in matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs, ys, zs)

    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.set_zlabel("Z axis")

    plt.show()

if __name__ == '__main__':
    #   Establish serial connection
    try:
        cxn = Serial('/dev/ttyACM0', baudrate=9600, timeout = 2)
        sio = io.TextIOWrapper(io.BufferedRWPair(cxn, cxn))

        #   Read data from scan
        data_vec = compile_data(sio)

        #   Set some initial conditions
        unit_vec = np.asarray([[0], [1], [0]])
        initial_z_angle = -0.610865  #   in radians, counterclockwise
        initial_x_angle = pi/9
        initial_direction = rotate_x(unit_vec, -initial_x_angle)
        initial_direction = rotate_z(initial_direction, initial_z_angle)
        initial_pos = np.asarray([[0.00702], [0.0451], [0.04418]])

        #   Plot using matplotlib
        a = process_data(data_vec)
        b = data_to_3d(a, initial_pos, initial_direction)
        c = open("data.txt", "w")
        c.write(str(b))
        c.close()
        plot_points(a)
        plot_points(b)

    #   Print error statement if connection not established
    except serialutil.SerialException:
        raise serialutil.SerialException, "Arduino not connected, or port busy."
