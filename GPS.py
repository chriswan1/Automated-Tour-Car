import math
import geographiclib as geo
import numpy
import rospy as ros
###
Get the message from the GPS extracting the latitude and longitude (lat and lon)
###
def get_GPS_Info():
    #origin_pose is the original pose that holds the latitude and longitude
    origin_pose = ros.wait_for_message('local_xy_origin', PoseStamped)
    lat = origin_pose.pose.position.y
    lon = origin_pose.pose.position.x
    return lat, lon

###
Converting extracted lat and lon to decimal format
###
def DMS_to_Dec(lat,lon):
    # Check for degrees, minutes, seconds format and convert to decimal
    if ',' in lat:
        degrees, minutes, seconds = lat.split(',')
        degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
        if lat[0] == '-':  # check for negative sign
            minutes = -minutes
            seconds = -seconds
        lat = degrees + minutes/60 + seconds/3600
    if ',' in lon:
        degrees, minutes, seconds = lon.split(',')
        degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
        if lon[0] == '-':  # check for negative sign
            minutes = -minutes
            seconds = -seconds
        lon = degrees + minutes/60 + seconds/3600

    lat = float(lat)
    lon = float(lon)
    ros.loginfo('Given GPS goal: lat %s, long %s.' % (lat, lon))
    return lat, lon


