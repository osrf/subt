#!/USSR/bin/env python

import rospy
import struct

import sys

from std_msgs.msg import String

from subt_example.srv import CreatePeer

import numpy as np

class CentralizedTester:
    def __init__(self, send_rate, send_count, name1, name2, payload_size=1024):
        self.send_rate = send_rate
        self.send_count = send_count
        self.outgoing_data = []
        self.incoming_data = []
        self.done = False
        self.send = False

        self.name1 = name1
        self.name2 = name2

        # Tell the subt_example_node to setup peer-connections topics
        service1 = '/' + name1 + '_control/create_peer'
        service2 = '/' + name2 + '_control/create_peer'
        print("Waiting for services {} {}".format(service1, service2))

        rospy.wait_for_service(service1)
        rospy.wait_for_service(service2)
        print("Calling service {}".format(service1))
        rospy.ServiceProxy(service1, CreatePeer).call(name2)
        print("Calling service {}".format(service2))
        rospy.ServiceProxy(service2, CreatePeer).call(name1)

        # Set topic names based on client names
        self.topic1 = '/' + name1 + '_control/' + name2 + "/send"
        self.topic2 = '/' + name2 + '_control/' + name1 + "/recv"
        print('Subscribing to [{}], publishing to [{}]'.format(self.topic2, self.topic1))

        self.send_pub = rospy.Publisher('%s' % self.topic1, String, queue_size=100)
        self.recv_sub = rospy.Subscriber('%s' % self.topic2, String, self.OnStringRecv)

        self.r = None

        self.seq_number = 0

        self.payload_size = payload_size

    def OnStringRecv(self, data):
        seq = int(data.data)
        self.incoming_data.append((rospy.Time.now(), data, seq))

    def PrintStatistics(self):
        if len(self.outgoing_data) != 0:
            perc = float(len(self.incoming_data))/float(len(self.outgoing_data))
            print('{} ({}/{})'.format(perc, len(self.incoming_data), len(self.outgoing_data)))

    def UpdateRate(self, send_rate):
        self.send_rate = send_rate
        self.r = rospy.Rate(self.send_rate)

    def UpdateSize(self, payload_size):
        self.payload_size = payload_size

    def GetLatency(self):
        window = 100

        if len(self.incoming_data) == 0 or len(self.outgoing_data) == 0:
            return np.Infinity

        incoming = {}
        outgoing = {}
        for d in self.incoming_data[-window:]:
            incoming[d[2]] = d[0].to_sec()
        for d in self.outgoing_data[-window:]:
            outgoing[d[2]] = d[0].to_sec()

        latencies = []
        for (seq, rx) in incoming.items():
            try:
                tx = outgoing[seq]
                latencies.append(rx - tx)
            except Exception, e:
                continue
        #print(latencies)

        if len(latencies) > 0:
            #print(np.mean(latencies))
            return np.mean(latencies)

        return np.Infinity

    def GetStatistics(self):
        if len(self.outgoing_data) != 0:
            perc = float(len(self.incoming_data))/float(len(self.outgoing_data))
        else:
            perc = 0.0
        return [perc, len(self.incoming_data), len(self.outgoing_data)]

    def GetRate(self, queue):
        if len(queue) == 0:
            return 0.0

        window = 10
        data = queue[-window:]
        dt = 0.0
        try:
            dt = (data[-1][0] - data[0][0]).to_sec()
        except Exception, e:
            print('ERROR: {}'.format(e))
            return 0.0

        if(abs(dt) < 0.0001):
            return 0.0

        total_data = 0
        for d in data:
            total_data += len(d[1].data)

        return total_data/dt

    def GetRXRate(self):
        return self.GetRate(self.incoming_data)

    def GetTXRate(self):
        return self.GetRate(self.outgoing_data)

    def ResetStatistics(self):
        self.incoming_data = []
        self.outgoing_data = []

    def SendBurst(self, size_in_bytes):
        # Send sequence of packets of size self.payload_size such that
        # total bytes is 'size_in_bytes'
        N = int(size_in_bytes/self.payload_size)+1

        for i in range(0, N):
            test_msg = String()
            test_msg.data = "{}".format(self.seq_number)
            test_msg.data = test_msg.data + ' '*(max(self.payload_size - len(test_msg.data), 0))

            self.send_pub.publish(test_msg)
            self.outgoing_data.append((rospy.Time.now(), test_msg, self.seq_number))
            self.seq_number = self.seq_number + 1

    def Spin(self):
        test_msg = String()
        if self.r is None:
            self.r = rospy.Rate(self.send_rate)
        while not rospy.is_shutdown() and not self.done:
            if self.send:
                if self.send_count < 0 or self.send_count > 0:
                    test_msg.data = "{}".format(self.seq_number)
                    test_msg.data = test_msg.data + ' '*(max(self.payload_size - len(test_msg.data), 0))

                    self.send_pub.publish(test_msg)
                    self.outgoing_data.append((rospy.Time.now(), test_msg, self.seq_number))
                    self.seq_number = self.seq_number + 1

                    self.send_count = self.send_count - 1
                    self.r.sleep()
                    #self.PrintStatistics()
