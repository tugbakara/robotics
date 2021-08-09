#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math 

def scanCallback(scanData):
    minValue,minIndex = minRangeIndex(scanData.ranges)
    print "\nthe min range value is: ",minValue
    print "the min range index is: ",minIndex
    maxValue,maxIndex = maxRangeIndex(scanData.ranges)
    print"\nthe max range value is: ",maxValue
    print "the max range index is: ",maxIndex
    averageValue = averageRange(scanData.ranges)
    print "\nthe aveange value is: ",averageValue
    #In the averageBtwnIndices function 2 and 7 represent between 2nd and 7th indices, respectlively.
    averageValueIndices = averageBtwnIndices(scanData.ranges,2,7)
    print "\nthe average between two indices is: ", averageValueIndices    
    print "the field of view: ",fieldOfView(scanData)
    
def fieldOfView(scanData):
    return ((scanData.angle_max - scanData.angle_min)*180.0/3.14)    
def minRangeIndex(ranges):
    # to avoid and get the clear solutions ranges are filtered below in every method.
    ranges = [x for x in ranges if not math.isnan(x)]
    return ( min(ranges),ranges.index(min(ranges)))
def maxRangeIndex(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (max(ranges),ranges.index(max(ranges)))
def averageRange(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (sum(ranges)/float(len(ranges)))
def averageBtwnIndices(ranges,i,j):
    ranges = [x for x in ranges if not math.isnan(x)]
    sliceOfArray = ranges[i:j+1]
    return(sum(sliceOfArray)/float(len(sliceOfArray)))


if __name__ == '__main__':
    # initializing the node 
    nodeName = "scanNode"
    rospy.init_node(nodeName,anonymous = True)
    # creating subscriber with topic /scan with the function to take current scan messages.
    rospy.Subscriber("scan",LaserScan,scanCallback)
    # spin() keeps the python from exitting untill this node is stopped.
    rospy.spin()
