#!/usr/bin/env python3
import rospy
import json
from heron_msgs.msg import Drive, Course
from sensor_msgs.msg import JointState, NavSatFix
from geometry_msgs.msg import Vector3Stamped
from time import sleep
import yaml
import rosnode
import math
import matplotlib.pyplot as plt

class WaterDrone:
    def __init__(self):
        nodes = rosnode.get_node_names()
        rospy.init_node("water_drone", anonymous=False)
        self.my_lat = 0
        self.my_lon = 0
        #self.way_coord = [{'lat': 49.8987564727, 'lon': 8.90169695784}, {'lat': 49.898914397634286, 'lon': 8.9015597651453}, {'lat': 49.89881918695902, 'lon': 8.901497385692466}, {'lat': 49.89907232256857, 'lon': 8.901422572450606}, {'lat': 49.89888190121803, 'lon': 8.901297813544971}, {'lat': 49.89923024750286, 'lon': 8.901285379755905}, {'lat': 49.89894461547705, 'lon': 8.901098241397449}, {'lat': 49.89938817243714, 'lon': 8.901148187061203}, {'lat': 49.899007329736065, 'lon': 8.900898669249955}, {'lat': 49.89954609737143, 'lon': 8.901010994366509}, {'lat': 49.89907004399508, 'lon': 8.900699097102432}, {'lat': 49.89970402230571, 'lon': 8.900873801671807}, {'lat': 49.8991327582541, 'lon': 8.90049952495491}, {'lat': 49.89986194724, 'lon': 8.900736608977105}, {'lat': 49.899195472513114, 'lon': 8.900299952807416}, {'lat': 49.899925637, 'lon': 8.90068128035}, {'lat': 49.89925818677213, 'lon': 8.900100380659893}, {'lat': 49.8999203868244, 'lon': 8.90047215226491}, {'lat': 49.899320901031146, 'lon': 8.899900808512399}, {'lat': 49.899915136648794, 'lon': 8.900263024179822}, {'lat': 49.89938361529016, 'lon': 8.899701236364876}, {'lat': 49.8999117653, 'lon': 8.900128734630016}, {'lat': 49.89944632954918, 'lon': 8.899501664217354}, {'lat': 49.89990660837607, 'lon': 8.899919604224579}, {'lat': 49.899509043808195, 'lon': 8.89930209206986}, {'lat': 49.899901451452145, 'lon': 8.899710473819141}, {'lat': 49.8995632732, 'lon': 8.899129520849982}, {'lat': 49.89989629452822, 'lon': 8.899501343413476}, {'lat': 49.89964448571179, 'lon': 8.89893673431446}, {'lat': 49.89989113760429, 'lon': 8.899292213008039}, {'lat': 49.89972569822358, 'lon': 8.89874394777894}, {'lat': 49.89988598068037, 'lon': 8.899083082602601}, {'lat': 49.899806910735364, 'lon': 8.898551161243418}, {'lat': 49.89988082375644, 'lon': 8.898873952196936}, {'lat': 49.8998692169, 'lon': 8.898403255589997}]
        self.way_coord = [{'lat': 49.8987564727, 'lon': 8.90169695784}, {'lat': 49.898914397634286, 'lon': 8.9015597651453}, {'lat': 49.89881918695902, 'lon': 8.901497385692466}, {'lat': 49.89907232256857, 'lon': 8.901422572450606}, {'lat': 49.89888190121803, 'lon': 8.901297813544971}, {'lat': 49.89923024750286, 'lon': 8.901285379755905}, {'lat': 49.89894461547705, 'lon': 8.901098241397449}, {'lat': 49.89938817243714, 'lon': 8.901148187061203}, {'lat': 49.899007329736065, 'lon': 8.900898669249955}, {'lat': 49.89954609737143, 'lon': 8.901010994366509}, {'lat': 49.89907004399508, 'lon': 8.900699097102432}, {'lat': 49.89970402230571, 'lon': 8.900873801671807}, {'lat': 49.8991327582541, 'lon': 8.90049952495491}, {'lat': 49.89986194724, 'lon': 8.900736608977105}, {'lat': 49.899195472513114, 'lon': 8.900299952807416}, {'lat': 49.899925637, 'lon': 8.90068128035}, {'lat': 49.89925818677213, 'lon': 8.900100380659893}, {'lat': 49.8999203868244, 'lon': 8.90047215226491}, {'lat': 49.899320901031146, 'lon': 8.899900808512399}, {'lat': 49.899915136648794, 'lon': 8.900263024179822}, {'lat': 49.89938361529016, 'lon': 8.899701236364876}, {'lat': 49.8999117653, 'lon': 8.900128734630016}, {'lat': 49.89944632954918, 'lon': 8.899501664217354}, {'lat': 49.89990660837607, 'lon': 8.899919604224579}, {'lat': 49.899509043808195, 'lon': 8.89930209206986}, {'lat': 49.899901451452145, 'lon': 8.899710473819141}, {'lat': 49.8995632732, 'lon': 8.899129520849982}, {'lat': 49.89989629452822, 'lon': 8.899501343413476}, {'lat': 49.89964448571179, 'lon': 8.89893673431446}, {'lat': 49.89989113760429, 'lon': 8.899292213008039}, {'lat': 49.89972569822358, 'lon': 8.89874394777894}]
        self.current_line = {'k': 0, 'b': 0, 'angle': 0}
        self.course_pub = rospy.Publisher("cmd_course", Course, queue_size=10)
        #rospy.spin()
        rospy.Subscriber("/navsat/fix", NavSatFix, self.get_coord)
        self.k_lon = 111320
        self.k_lat = 111319*math.cos(self.way_coord[0]['lat']*math.pi/180)
        self.err_m = 1
        self.err_lat = self.err_m/self.k_lat
        self.err_lon = self.err_m/self.k_lon
    # def look_herons(self):
    #     nodes = rosnode.get_node_names()
    #     for node in nodes:
    #         if node == "water_drone":
    #             self.herons.append(1)
    def get_coord(self, data):
        self.my_lat = data.latitude
        self.my_lon = data.longitude
    def line(self, target_lat, target_lon):
        # lon = k*lat + b
        self.current_line['k'] = (target_lon - self.my_lon)/(target_lat - self.my_lat)
        self.current_line['b'] = target_lon - self.current_line['k']*target_lat
        self.current_line['angle'] = math.atan2(target_lon - self.my_lon, target_lat - self.my_lat)
        print(f'angle = {self.current_line["angle"]}')
    def go(self):
        try:
            sleep(1)
            plt.figure()
            plt.ion()
            plt.show()
            f1 = open('coord-err-1m', 'w')
            f2 = open('coord-final-err-1m', 'w')
            final_coord = [[],[]]
            for target in self.way_coord:
                print(f'New target {target}')
                self.line(target['lat'], target['lon'])
                course_mess = Course()
                course_mess.yaw = -self.current_line['angle'] + math.pi/2
                course_mess.speed = 0
                self.course_pub.publish(course_mess)
                sleep(10)
                while (abs(self.my_lat - target['lat']) > self.err_lat) and (abs(self.my_lon - target['lon']) > self.err_lon):
                    self.line(target['lat'], target['lon'])
                    #print(f'my_lat = {self.my_lat}, my_lon = {self.my_lon}')
                    course_mess = Course()
                    course_mess.yaw = -self.current_line['angle'] + math.pi/2
                    course_mess.speed = 1
                    self.course_pub.publish(course_mess)
                    plt.plot(self.my_lat, self.my_lon, 'ro')
                    plt.draw()
                    plt.pause(0.001)
                    f1.write(f'{self.my_lat}; {self.my_lon}\n')
                    sleep(2)
                course_mess.speed = 0
                self.course_pub.publish(course_mess)
                final_coord[0].append(self.my_lat)
                final_coord[1].append(self.my_lon)
                f2.write(f'{self.my_lat}; {self.my_lon}\n')
                sleep(2)
            print(f'final_coord = {final_coord}')
            f1.close()
            f2.close()
        except KeyboardInterrupt:
            exit()
                
WaterDrone().go()

