#!/usr/bin/env python

import rospy
import time
import csv
import math

import roslibpy


start_time=0
end_time=0
rtt_time=0
sizes=[]
rtts=[]


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
    for i in range(1,int(size+1)):
        message+="A"
    return message

def print_csv(sizes,rtts):
    with open('rtt_rosbridge_topic.csv', mode='w') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for i in range(0,len(sizes)):
            writer.writerow([sizes[i], rtts[i]])

def callback(message):
    stop_timer()


if __name__ == '__main__':
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    test = roslibpy.Topic(client, '/test', 'std_msgs/String')
    test.advertise()
    test_back = roslibpy.Topic(client, '/test_back', 'std_msgs/String')

    test_back.subscribe(callback)
    time.sleep(1)
    i=1
    while i<=6: 
        message=generate_message(i)
        start_timer()
        test.publish(roslibpy.Message({'data': message}))
        time.sleep(1)
        i+=0.25

    print_csv(sizes,rtts)
    print("done")