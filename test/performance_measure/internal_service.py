#!/usr/bin/env python3

import rospy
import time
import csv
import math
from ros_web_client.srv import RTTSrv,RTTSrvRequest,RTTSrvResponse


start_time=0
end_time=0
rtt_time=0
sizes=[]
rtts=[]


def callback(req):
    return RTTSrvResponse(req.message)



def stop_timer():
    global start_time,end_time,time,rtts
    end_time=time.time()
    rtt_time = end_time-start_time
    rtts.append(rtt_time*1000)
    print(rtt_time)

def start_timer():
    global start_time
    start_time=time.time()

def generate_message(logsize):
    global sizes
    size=math.floor(math.pow(10,logsize))
    sizes.append(size)
    message=""
    for i in range(1,size+1):
        message+="A"
    return message

def print_csv(sizes,rtts):
    with open('rtt_internal_service.csv', mode='w') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for i in range(0,len(sizes)):
            writer.writerow([sizes[i], rtts[i]])


if __name__ == '__main__':
    rospy.init_node('test_rtt_internal', anonymous=False)
    s = rospy.Service('testservice', RTTSrv, callback)
    rospy.wait_for_service('testservice')
    
    i=1
    while i<=6: 
        message=generate_message(i)
        start_timer()
        testservice = rospy.ServiceProxy('testservice', RTTSrv)
        res = testservice(message)
        stop_timer()
        i+=0.25

    print_csv(sizes,rtts)
    print("done")

    
    while not rospy.is_shutdown():
        rospy.spin()
    
