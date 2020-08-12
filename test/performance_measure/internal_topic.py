#!/usr/bin/env python3

import rospy
import time
import csv
import math
from std_msgs.msg import String


start_time=0
end_time=0
rtt_time=0
sizes=[]
rtts=[]


def callback(msg):
    stop_timer()

def stop_timer():
    global start_time,end_time,rtt_time,rtts
    end_time=time.time()
    rtt_time = end_time-start_time
    rtts.append(rtt_time*2000)
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
    with open('rtt_internal_topic.csv', mode='w') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for i in range(0,len(sizes)):
            writer.writerow([sizes[i], rtts[i]])


if __name__ == '__main__':
    #Publisher
    pub = rospy.Publisher('testtopic', String, queue_size=10)
    rospy.init_node('test_rtt_internal', anonymous=False)
    rate = rospy.Rate(0.5)
    #Subscriber
    rospy.Subscriber("testtopic", String, callback)
    rate.sleep()

    i=1
    while i<=6:
        message=generate_message(i)
        start_timer()
        pub.publish(message)
        rate.sleep()
        i+=0.25

    print_csv(sizes,rtts)
    print("done")

    
    while not rospy.is_shutdown():
        rospy.spin()
    
