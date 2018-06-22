#!/usr/bin/env python2
import numpy as np
import math
import matplotlib.pyplot as plt

from plotly import __version__
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
from plotly.graph_objs import Scatter, Figure, Layout

def normal_to_theta(normals):
    "normalToTheta: Assumes normals in the form x, y, z"
    # Parametrize normals with azimuth == phi
    # phi = arctan(y/x)
    x_values = normals[:, 0]
    y_values = normals[:, 1]
    theta = []
    for i in range(0, len(x_values)):
        theta.append(math.atan(y_values[i] / x_values[i]))

    return theta

def read_values(file_path):
    "readValues"
    with open(file_path, "r") as filehandle:
        matrix = [x.strip().split('\t') for x in filehandle]
        # Remove first line, which might contain only column definitions
        matrix = matrix[1:-1]
        return matrix

def main():
    "Main"
    matrix = read_values("./output_landmarks.txt")
    matrix = np.array(matrix, dtype=float)
    z_values = matrix[:, 2]
    print z_values
    delta_z = 0.01
    min_z = -0.5
    max_z = 2.0
    array_elements = int(math.ceil((max_z - min_z) / delta_z))

    # the histogram of the data
    # n, bins, patches = plt.hist(z_values, bins=array_elements, density=False,
                                # facecolor='g', alpha=0.75)
    n, bins, patches = plt.hist(z_values, bins=100, density=False,
                                facecolor='g', alpha=0.75)

    plt.xlabel('Z')
    plt.ylabel('Number of Landmarks')
    plt.title('Histogram of Landmarks by Z component')
    plt.axis([min_z, max_z, min(n), max(n)])
    plt.grid(True)
    plt.show()

    matrix_normals = read_values("./output_normals.txt")
    matrix_normals = np.array(matrix_normals, dtype=float)
    theta = normal_to_theta(matrix_normals)

    number_bins = 50
    n, bins, patches = plt.hist(theta, bins=number_bins, density=False,
                                facecolor='g', alpha=0.75)

    plt.xlabel('Theta[rad]')
    plt.ylabel('Number of normals')
    plt.title('Histogram of Normals by theta')
    plt.axis([-math.pi/2, math.pi/2, 0, max(n)+10])
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()

    # delta_z = 0.01
    # min_z = -0.5
    # max_z = 2.0
    # array_elements = int(math.ceil((max_z - min_z) / delta_z))
    # histogram = np.zeros(array_elements)
    # x_axis = [min_z]
    # for i in range(array_elements):
        # x_axis.append((i+1) * delta_z + min_z)

    # matrix = readValues("./output_landmarks.txt")
    # # Output.
    # values = []
    # # We loop over the rows...
    # for row in matrix:
        # # And we loop over all the items in the row...
        # for index, item in enumerate(row):
            # if (index == 3):
                # value = float(item)
                # values.append(value)
                # if ((value >= min_z) & (value <= max_z)):
                    # histogram[int(math.floor((value - min_z) / delta_z))] += 1
                    # #plot([Scatter(x=x_axis, y=histogram)])
