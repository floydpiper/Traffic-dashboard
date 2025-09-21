#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import requests
import json
import polyline as pl
import tkdesigner
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage
from decimal import Decimal 
import datetime

# Define screen size of LED
width = 500
height = 400
window = Tk()

# --------- Define GUI ------ #
canvas = Canvas(
    window,
    bg = "#2A2D39",
    height = 600,
    width = 1024,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
first_rect = canvas.create_rectangle(
    20.0,
    20.0,
    502.0,
    374.0,
    fill="#30333E",
    outline="")

second_rect = canvas.create_rectangle(
    522.0,
    394.0,
    1004.0,
    580.0,
    fill="#30333E",
    outline="")

canvas.create_rectangle(
    522.0,
    20.0,
    1004.0,
    374.0,
    fill="#30333E",
    outline="")

canvas.create_rectangle(
    20.0,
    394.0,
    502.0,
    580.0,
    fill="#30333E",
    outline="")

first_arrival = canvas.create_text(
    199.0,
    529.0,
    anchor="nw",
    text="Arrival: ",
    fill="#FFFFFF",
    font=("Inter", 14 * -1)
)

second_arrival = canvas.create_text(
    701.0,
    529.0,
    anchor="nw",
    text="Arrival: ",
    fill="#FFFFFF",
    font=("Inter", 14 * -1)
)

first_eta = canvas.create_text(
    98.0,
    413.0,
    anchor="nw",
    text="----------",
    fill="#F7F7F7",
    font=("Inter", 96 * -1)
)

second_eta = canvas.create_text(
    600.0,
    413.0,
    anchor="nw",
    text="----------",
    fill="#F7F7F7",
    font=("Inter", 96 * -1)
)

canvas.create_text(
    260.0,
    30.0,
    anchor="n",
    text="SCHOOL",
    fill="#7A7A7A",
    font=("Inter", 20, "bold")
)

canvas.create_text(
    760.0,
    30.0,
    anchor="n",
    text="WORK",
    fill="#7A7A7A",
    font=("Inter", 20, "bold")
)
# ----------------------------------- #

def convert_seconds_to_minutes(seconds):
    """ This function converts seconds to minutes 

    Args:
        seconds (string): The second in a string format. EX: 345s

    Returns:
        minutes: The newly converted minutes
    """
    # Eliminate the s "###s"
    second = seconds[: -1]
    minutes = Decimal(second)/60

    return round(minutes)

def decode_polyline(polyline):
    """ This function takes the polyline from the JSON object and decodes it

    Args:
        polyline (_type_): The polyline from the JSON object

    Returns:
        decoded polyline: The decoded polyline
    """
    return pl.decode(polyline)

def latlon_to_xy(lat, lon, min_lat, max_lat, min_lon, max_lon, x_min, y_min, x_max, y_max):
    """ Converts lat/lon coordinates into screen (x, y) coordinates that fit inside the screen

    Args:
        lat (float): Lat of the point 
        lon (float): Lon of the point 
        min_lat (float): Min lat of the bounding box
        max_lat (float): Max lat of the bounding box
        min_lon (float): Min lon of the bounding box
        max_lon (float): Max lon of the bounding box
        x_min (int): Min x pixel coordinate 
        y_min (int): Min y pixel coordinate 
        x_max (int): Max x pixel coordinate 
        y_max (int): Max y pixel coordinate 

    Returns:
        x, y: Pixel (x, y) position corresponding to the lat/lon
    """
    rect_width = x_max - x_min
    rect_height = y_max - y_min


    x = int(((lon - min_lon) / (max_lon - min_lon + 1e-6)) * rect_width + x_min)
    y = int(((max_lat - lat) / (max_lat - min_lat + 1e-6)) * rect_height + y_min)

    return x, y


def get_arrival_time(eta):
    """ This function takes the eta and calculates the arrival time

    Args:
        eta: Estimated time unti arrival

    Returns:
        arrival time: The arrival time in a specified format 
    """
    time = datetime.datetime.now()
    minutes = datetime.timedelta(minutes=eta)   
    arrival = time + minutes

    return arrival.strftime("%H:%M")

def start_gui():
    """ Starts up the GUI
    """
    window.geometry("1024x600")
    window.configure(bg="#2A2D39")
    window.resizable(False, False)
    window.mainloop()

def callback(msg):
    """ ROS subscriber callback that updates GUI based on new route data

    Args:
        msg: JSON encoded message
    """
    # Retrieve JSON information 
    data = json.loads(msg.data)

    # ---------------- Route 1 (upper left box) ---------------- #
    # Extract information from JSON
    eta1 = data['route1']['eta']
    minutes1 = convert_seconds_to_minutes(eta1)
    color1 = data['route1']['color']  
    arrival1 = get_arrival_time(minutes1)

    # Update GUI to reflect new eta and arrival time
    canvas.itemconfigure(first_eta, text=str(minutes1) + " min", fill=color1)
    canvas.itemconfigure(first_arrival, text="Arrival: " + str(arrival1))

    # Decode polyline and draw route on screen
    points1 = decode_polyline(data['route1']['polyline'])
    lats1 = [pt[0] for pt in points1]
    lons1 = [pt[1] for pt in points1]
    min_lat1, max_lat1 = min(lats1), max(lats1)
    min_lon1, max_lon1 = min(lons1), max(lons1)

    xy_points1 = []
    for lat, lon in points1:
        x, y = latlon_to_xy(lat, lon, min_lat1, max_lat1, min_lon1, max_lon1,
                            x_min=40, y_min=70, x_max=490, y_max=360)  
        xy_points1.extend([x, y])

    canvas.create_line(xy_points1, width=7, fill=color1, smooth=True, splinesteps=36)

    # ---------------- Route 2 (upper right box) ---------------- #
    # Extract information from JSON
    eta2 = data['route2']['eta']
    minutes2 = convert_seconds_to_minutes(eta2)
    color2 = data['route2']['color']   # <-- use color from Google data
    arrival2 = get_arrival_time(minutes2)

    # Update GUI to reflect new eta and arrival time
    canvas.itemconfigure(second_eta, text=str(minutes2) + " min", fill=color2)
    canvas.itemconfigure(second_arrival, text="Arrival: " + str(arrival2))

    # Decode polyline and draw route on screen
    points2 = decode_polyline(data['route2']['polyline'])
    lats2 = [pt[0] for pt in points2]
    lons2 = [pt[1] for pt in points2]
    min_lat2, max_lat2 = min(lats2), max(lats2)
    min_lon2, max_lon2 = min(lons2), max(lons2)

    xy_points2 = []
    for lat, lon in points2:
        x, y = latlon_to_xy(lat, lon, min_lat2, max_lat2, min_lon2, max_lon2,
                            x_min=542, y_min=70, x_max=984, y_max=340) 
        xy_points2.extend([x, y])

    canvas.create_line(xy_points2, width=7, fill=color2, smooth=True, splinesteps=36)



if __name__ == '__main__':

    # Start ROS
    rospy.init_node('display_gui')
    rospy.Subscriber('/route_data', String, callback)

    start_gui()
