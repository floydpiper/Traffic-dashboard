#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import requests
import json

# Set coordinates and Google API key
API = 'YOUR KEY HERE' 
# First destination
start1 = ['LAT', 'LON']
end1 = ['LAT', 'LON']
# Second destination 
start2 = ['LAT', 'LON']
end2   = ['LAT', 'LON']


def grab_route_color(delay):
    """ This function returns the color of the route based on traffic 

    Args:
        delay: Traffic delay
    """
     # < 5 min delay
    if delay < 300:        
        color = "#26a269" 
    # < 15 min delay  
    elif delay < 900:       
        color = "#F2994A"  
    # < 30 min delay
    elif delay < 1800:      
        color = "#F53F35"   
    else:
        color = "#8B0000"   

    return color


def fetch_route_data(start, end):
    """ This function gets the Google Route for a specific start and end destination.

    Args:
        start: (lat,lon) of start coordinate
        end: (lat,lon) of end coordinate

    Returns:
        json object: Json object that contains eta, polyline of route, and route color
    """
    # Build json request
    url = 'https://routes.googleapis.com/directions/v2:computeRoutes'
    headers = {
        'Content-Type': 'application/json',
        'X-Goog-Api-Key': API,
        'X-Goog-FieldMask': 'routes.duration,routes.distanceMeters,routes.polyline.encodedPolyline,routes.travelAdvisory'
    }
    body = {
        "origin": {"location": {"latLng": {"latitude": start[0], "longitude": start[1]}}}, 
        "destination": {"location": {"latLng": {"latitude": end[0], "longitude": end[1]}}}, 
        "travelMode": "DRIVE",
        "routingPreference": "TRAFFIC_AWARE_OPTIMAL"
    }

    response = requests.post(url, headers=headers, json=body)
    data = response.json()

    # Filter out information
    route = data['routes'][0]
    duration = route['duration']
    encoded_polyline = route['polyline']['encodedPolyline']

    # Extract route delay
    delay = 0
    travel = route.get("travelAdvisory")
    if travel:
        delay_str = travel.get("delay")
        if delay_str:
            delay = int(delay_str[:-1])

    color = grab_route_color(delay)

    # Return only necessary information 
    return {
        'eta': duration, 
        'polyline': encoded_polyline, 
        'color': color
    }



def main():
    """ Main function 
    """
    # Set ROS nodes
    rospy.init_node('route_grabber')
    pub = rospy.Publisher('/route_data', String, queue_size=10)
    rate = rospy.Rate(1)  

    # Grab and concat route information
    while not rospy.is_shutdown():
        route1 = fetch_route_data(start1, end1)
        route2 = fetch_route_data(start2, end2)

        # Combine both into one JSON message
        msg = json.dumps({
            'route1': route1,
            'route2': route2
        })

        pub.publish(msg)
        print("Published both routes")
        rate.sleep()


if __name__ == '__main__':
    main()











