#!/usr/bin/env python3

"""
This file is used to plot RGB data collected from the color sensor.
It should be run on your computer, not on the robot.

Before running this script for the first time, you must install the dependencies
as explained in the README.md file.
"""

from ast import literal_eval
from math import sqrt, e, pi
from statistics import mean, stdev

from matplotlib import pyplot as plt
import numpy as np


COLOR_DATA = {
    "white": "color_data/white_data.csv",   # Hallway
    "purple": "color_data/purple_data.csv",    # Burning room
    "yellow": "color_data/yellow_data.csv",    # Room to avoid
    "green": "color_data/green_data.csv",     # Green Card
    "red": "color_data/red_data.csv",     # Red Card
    #"black": "black_data.csv",         # Grid lines
    "orange": "color_data/orange_data.csv"     # Entrance line
}

def normalized_rgb_of_color(filename):
    "Return the normalized R, G, B values from a csv file"
    red, green, blue = [], [], []

    with open(filename, "r") as f:
        for line in f.readlines():
            r,g,b = literal_eval(line)  # convert string to 3 floats
            # normalize the values to be between 0 and 1

            ### UNIT-VECTOR METHOD ###
            # denominator = sqrt(r ** 2 + g ** 2 + b ** 2)

            ### RATIO METHOD ##
            denominator = r + g + b

            red.append(r/denominator)
            green.append(g/denominator)
            blue.append(b/denominator)
        
    return red, green, blue


def gaussian(x, values):
    "Return a gaussian function from the given values."
    sigma = stdev(values)
    gaus_fct = (1 / (sigma * sqrt(2 * pi))) * e ** (-((x - mean(values)) ** 2) / (2 * sigma ** 2))
    #print(gaus_fct)
    return gaus_fct


# not used
def plot_color_graph(red, green, blue, graph_color):
    "Plot a graph of red, green, blue distribution for a given color "

    plt.figure(f"Color_sensor_{graph_color}")
    x_values = np.linspace(0, 1, 255)  # 255 evenly spaced values between 0 and 1
    plt.plot(x_values, gaussian(x_values, red), color="r")
    plt.plot(x_values, gaussian(x_values, green), color="g")
    plt.plot(x_values, gaussian(x_values, blue), color="b")
    plt.xlabel("Normalized intensity value")
    plt.ylabel("Normalized intensity PDF by color")
    plt.title(f"Color Sensor data for {graph_color} cube")
    plt.show()


def color_mean_std (color_name, color_data_file):
    "writes to color_stats/ the Mean & std_deviation of RGB values for the color"
    red, green, blue = normalized_rgb_of_color(color_data_file)

    f = open("color_mean_std/" + color_name + "_mean_std.csv", "w")

    # f.write("{color_name} \nMean, Standard deviation \nR\nG\nB \n\n")
    f.write("{:.5f}".format(mean(red))+", "+"{:.5f}".format(stdev(red))+"\n")
    f.write("{:.5f}".format(mean(green))+", "+"{:.5f}".format(stdev(green))+"\n")
    f.write("{:.5f}".format(mean(blue))+", "+"{:.5f}".format(stdev(blue)))

    f.close


def write_mean_std_for_all_colors():
    "writes statistics for all colors in COLOR_DATA using color_statistics()"
    for color in COLOR_DATA:
        color_data_path = COLOR_DATA[color]
        color_mean_std(color, color_data_path)


if __name__ == "__main__":
    write_mean_std_for_all_colors()
   
