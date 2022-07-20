import rospy
import ros_numpy
from geometry_msgs.msg import Point, PointStamped

import numpy as np

AVGAMOUNT = 100

lastrealworldx = []
lastrealworldy = []
lastrealworldz = []
realworldi = 0
lastcamx = []
lastcamy = []
cami = 0

for i in range(AVGAMOUNT):
    lastrealworldx.append(0)
    lastrealworldy.append(0)
    lastrealworldz.append(0)
    lastcamx.append(0)
    lastcamy.append(0)

def printcoords():
    global lastrealworldx, lastrealworldy, lastrealworldz, lastcamx, lastcamy, AVGAMOUNT
    realworldavgx = sum(lastrealworldx) / AVGAMOUNT
    realworldavgy = sum(lastrealworldy) / AVGAMOUNT
    realworldavgz = sum(lastrealworldz) / AVGAMOUNT
    camavgx = sum(lastcamx) / AVGAMOUNT
    camavgy = sum(lastcamy) / AVGAMOUNT

    print('\nmeters [', round(realworldavgx, 2), ' ', round(realworldavgy, 2), ' ', round(realworldavgz, 2), ']')
    print('pixels [', round(camavgx), ' ', round(camavgy), ']\n')


def callback_realworld(coord):
    global lastrealworldx, lastrealworldy, lastrealworldz, realworldi, AVGAMOUNT
    pnt = ros_numpy.numpify(coord.point)
    
    lastrealworldx[realworldi] = pnt[0]
    lastrealworldy[realworldi] = pnt[1]
    lastrealworldz[realworldi] = pnt[2]

    realworldi += 1
    if realworldi == AVGAMOUNT:
        realworldi = 0

    printcoords()

def callback_cam(coord):
    global lastcamx, lastcamy, cami, AVGAMOUNT
    pnt = ros_numpy.numpify(coord)
    
    lastcamx[cami] = pnt[0]
    lastcamy[cami] = pnt[1]

    cami += 1
    if cami == AVGAMOUNT:
        cami = 0

if __name__ == '__main__':
    rospy.init_node("coordread", anonymous=True)
    rospy.loginfo('coord read node started')

    rospy.Subscriber('/realworld_coords', PointStamped, callback_realworld) 
    rospy.Subscriber("/position_2d", Point, callback_cam)

    while not rospy.is_shutdown():
        rospy.spin()