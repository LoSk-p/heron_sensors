#!/usr/bin/env python3
import rospy
import json
from heron_msgs.msg import Drive, Course
from sensor_msgs.msg import JointState, NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
import time
import yaml
import rosnode
import math
import matplotlib.pyplot as plt
import os
from ast import literal_eval

class WaterDrone:
    def __init__(self, boat_nomber):
        self.with_pollution_looking = False
        self.boat_nomber = boat_nomber
        if self.boat_nomber == 0:
            self.prefix = ''
        else:
            self.prefix = f'drone{self.boat_nomber}/'
        #nodes = rosnode.get_node_names()
        self.path = os.path.realpath(__file__)[:-20]
        rospy.init_node(f"water_drone{self.boat_nomber}", anonymous=False)
        if self.with_pollution_looking:
            f = open(f'{self.path}utils/pollution_borders', 'w')
            time.sleep(2)
            f.close()
            x0 = 49.89973114614826
            y0 = 8.900325679552948
            r = 0.0002
            with open(f'{self.path}utils/map') as f:
                for line in f:
                    js = literal_eval(line)
                    lat = js["lat"]
                    lon = js["lon"]
                    if r**2 >= ((lat - x0)**2 + (lon-y0)**2):
                        plt.plot(lat, lon, 'bo')
                        plt.draw()
                        plt.pause(0.001)
        self.my_lat = 0
        self.my_lon = 0
        self.way_coord = []
        with open(f'{self.path}utils/ways/way{self.boat_nomber}') as f:
            for line in f:
                self.way_coord.append(literal_eval(line))
        #self.way_coord = [{'lat': 49.8987564727, 'lon': 8.90169695784}, {'lat': 49.89878805768686, 'lon': 8.901669519301059}, {'lat': 49.89876901555181, 'lon': 8.901657043410466}, {'lat': 49.898819642673715, 'lon': 8.901642080762123}, {'lat': 49.89878155840361, 'lon': 8.901617128980973}, {'lat': 49.89885122766057, 'lon': 8.90161464222318}, {'lat': 49.89879410125542, 'lon': 8.901577214551452}, {'lat': 49.89888281264743, 'lon': 8.901587203684244}, {'lat': 49.898806644107225, 'lon': 8.901537300121959}, {'lat': 49.898914397634286, 'lon': 8.9015597651453}, {'lat': 49.89881918695903, 'lon': 8.901497385692437}, {'lat': 49.89894598262114, 'lon': 8.901532326606365}, {'lat': 49.89883172981084, 'lon': 8.901457471262916}, {'lat': 49.898977567608, 'lon': 8.901504888067421}, {'lat': 49.89884427266264, 'lon': 8.901417556833422}, {'lat': 49.89900915259486, 'lon': 8.901477449528485}, {'lat': 49.89885681551445, 'lon': 8.9013776424039}, {'lat': 49.899040737581714, 'lon': 8.901450010989542}, {'lat': 49.898869358366255, 'lon': 8.90133772797438}, {'lat': 49.89907232256857, 'lon': 8.901422572450606}, {'lat': 49.89888190121806, 'lon': 8.901297813544886}, {'lat': 49.89910390755543, 'lon': 8.901395133911663}, {'lat': 49.89889444406987, 'lon': 8.901257899115365}, {'lat': 49.899135492542285, 'lon': 8.901367695372727}, {'lat': 49.89890698692167, 'lon': 8.901217984685843}, {'lat': 49.89916707752914, 'lon': 8.901340256833784}, {'lat': 49.89891952977348, 'lon': 8.90117807025635}, {'lat': 49.899198662516, 'lon': 8.90131281829484}, {'lat': 49.898932072625286, 'lon': 8.901138155826828}, {'lat': 49.89923024750286, 'lon': 8.901285379755905}, {'lat': 49.89894461547709, 'lon': 8.901098241397335}, {'lat': 49.899261832489714, 'lon': 8.901257941216961}, {'lat': 49.8989571583289, 'lon': 8.901058326967814}, {'lat': 49.89929341747657, 'lon': 8.901230502678025}, {'lat': 49.898969701180704, 'lon': 8.901018412538292}, {'lat': 49.89932500246343, 'lon': 8.901203064139082}, {'lat': 49.89898224403251, 'lon': 8.900978498108799}, {'lat': 49.899356587450285, 'lon': 8.901175625600146}, {'lat': 49.898994786884316, 'lon': 8.900938583679277}, {'lat': 49.89938817243714, 'lon': 8.901148187061203}, {'lat': 49.89900732973612, 'lon': 8.900898669249756}, {'lat': 49.899419757424, 'lon': 8.901120748522267}, {'lat': 49.89901987258793, 'lon': 8.900858754820263}, {'lat': 49.899451342410856, 'lon': 8.901093309983324}, {'lat': 49.899032415439734, 'lon': 8.900818840390741}, {'lat': 49.89948292739771, 'lon': 8.901065871444388}, {'lat': 49.89904495829154, 'lon': 8.90077892596122}, {'lat': 49.89951451238457, 'lon': 8.901038432905445}, {'lat': 49.899057501143346, 'lon': 8.900739011531726}, {'lat': 49.89954609737143, 'lon': 8.901010994366509}, {'lat': 49.89907004399515, 'lon': 8.900699097102205}, {'lat': 49.899577682358284, 'lon': 8.900983555827565}, {'lat': 49.89908258684696, 'lon': 8.900659182672683}, {'lat': 49.89960926734514, 'lon': 8.900956117288622}, {'lat': 49.899095129698765, 'lon': 8.90061926824319}, {'lat': 49.899640852332, 'lon': 8.900928678749686}, {'lat': 49.89910767255057, 'lon': 8.900579353813669}, {'lat': 49.899672437318856, 'lon': 8.900901240210743}, {'lat': 49.89912021540238, 'lon': 8.900539439384175}, {'lat': 49.89970402230571, 'lon': 8.900873801671807}, {'lat': 49.89913275825418, 'lon': 8.900499524954654}, {'lat': 49.89973560729257, 'lon': 8.900846363132864}, {'lat': 49.89914530110599, 'lon': 8.900459610525132}, {'lat': 49.89976719227943, 'lon': 8.900818924593928}, {'lat': 49.899157843957795, 'lon': 8.90041969609564}, {'lat': 49.899798777266284, 'lon': 8.900791486054985}, {'lat': 49.8991703868096, 'lon': 8.900379781666118}, {'lat': 49.89983036225314, 'lon': 8.900764047516049}, {'lat': 49.89918292966141, 'lon': 8.900339867236596}, {'lat': 49.89986194724, 'lon': 8.900736608977105}, {'lat': 49.89919547251321, 'lon': 8.900299952807103}, {'lat': 49.899893532226855, 'lon': 8.90070917043817}, {'lat': 49.89920801536502, 'lon': 8.900260038377581}, {'lat': 49.89992511721371, 'lon': 8.900681731899226}, {'lat': 49.899220558216825, 'lon': 8.90022012394806}, {'lat': 49.899925637, 'lon': 8.90068128035}, {'lat': 49.89923310106863, 'lon': 8.900180209518567}, {'lat': 49.89992458696488, 'lon': 8.900639454732755}, {'lat': 49.89924564392044, 'lon': 8.900140295089045}, {'lat': 49.899923536929755, 'lon': 8.900597629115737}, {'lat': 49.899258186772244, 'lon': 8.900100380659524}, {'lat': 49.89992248689463, 'lon': 8.900555803498492}, {'lat': 49.89927072962405, 'lon': 8.90006046623003}, {'lat': 49.89992143685951, 'lon': 8.900513977881474}, {'lat': 49.899283272475856, 'lon': 8.900020551800509}, {'lat': 49.899920386824384, 'lon': 8.900472152264228}, {'lat': 49.89929581532766, 'lon': 8.899980637371016}, {'lat': 49.89991933678926, 'lon': 8.90043032664721}, {'lat': 49.89930835817947, 'lon': 8.899940722941494}, {'lat': 49.899918286754136, 'lon': 8.900388501029965}, {'lat': 49.899320901031274, 'lon': 8.899900808511973}, {'lat': 49.89991723671901, 'lon': 8.900346675412948}, {'lat': 49.89933344388308, 'lon': 8.89986089408248}, {'lat': 49.89991618668389, 'lon': 8.900304849795702}, {'lat': 49.899345986734886, 'lon': 8.899820979652958}, {'lat': 49.899915136648765, 'lon': 8.900263024178685}, {'lat': 49.89935852958669, 'lon': 8.899781065223436}, {'lat': 49.89991408661364, 'lon': 8.90022119856144}, {'lat': 49.8993710724385, 'lon': 8.899741150793943}, {'lat': 49.89991303657852, 'lon': 8.900179372944422}, {'lat': 49.899383615290304, 'lon': 8.899701236364422}, {'lat': 49.899911986543394, 'lon': 8.900137547327176}, {'lat': 49.89939615814211, 'lon': 8.8996613219349}, {'lat': 49.8999117653, 'lon': 8.900128734630016}, {'lat': 49.89940870099392, 'lon': 8.899621407505407}, {'lat': 49.899910313539706, 'lon': 8.900086921029242}, {'lat': 49.89942124384572, 'lon': 8.899581493075885}, {'lat': 49.899908861779416, 'lon': 8.900045107428696}, {'lat': 49.89943378669753, 'lon': 8.899541578646392}, {'lat': 49.899907410019125, 'lon': 8.900003293827922}, {'lat': 49.899446329549335, 'lon': 8.89950166421687}, {'lat': 49.899905958258834, 'lon': 8.899961480227148}, {'lat': 49.89945887240114, 'lon': 8.89946174978735}, {'lat': 49.899904506498544, 'lon': 8.899919666626602}, {'lat': 49.89947141525295, 'lon': 8.899421835357856}, {'lat': 49.89990305473825, 'lon': 8.899877853025828}, {'lat': 49.89948395810475, 'lon': 8.899381920928334}, {'lat': 49.89990160297796, 'lon': 8.899836039425281}, {'lat': 49.89949650095656, 'lon': 8.899342006498813}, {'lat': 49.89990015121767, 'lon': 8.899794225824508}, {'lat': 49.899509043808365, 'lon': 8.89930209206932}, {'lat': 49.89989869945738, 'lon': 8.899752412223734}, {'lat': 49.89952158666017, 'lon': 8.899262177639798}, {'lat': 49.89989724769709, 'lon': 8.899710598623187}, {'lat': 49.89953412951198, 'lon': 8.899222263210277}, {'lat': 49.8998957959368, 'lon': 8.899668785022413}, {'lat': 49.89954667236378, 'lon': 8.899182348780784}, {'lat': 49.89989434417651, 'lon': 8.89962697142164}, {'lat': 49.89955921521559, 'lon': 8.899142434351262}, {'lat': 49.89989289241622, 'lon': 8.899585157821093}, {'lat': 49.8995632732, 'lon': 8.899129520849982}, {'lat': 49.89989144065593, 'lon': 8.89954334422032}, {'lat': 49.89959691197201, 'lon': 8.899104642790746}, {'lat': 49.89988998889564, 'lon': 8.899501530619773}, {'lat': 49.89963055074402, 'lon': 8.899079764731503}, {'lat': 49.89988853713535, 'lon': 8.899459717018999}, {'lat': 49.89966418951603, 'lon': 8.899054886672253}, {'lat': 49.899887085375056, 'lon': 8.899417903418225}, {'lat': 49.89969782828804, 'lon': 8.89903000861301}, {'lat': 49.899885633614765, 'lon': 8.899376089817679}, {'lat': 49.89973146706005, 'lon': 8.89900513055376}, {'lat': 49.899884181854475, 'lon': 8.899334276216905}, {'lat': 49.89976510583206, 'lon': 8.89898025249451}, {'lat': 49.899882730094184, 'lon': 8.899292462616359}, {'lat': 49.89979874460407, 'lon': 8.898955374435268}, {'lat': 49.89988127833389, 'lon': 8.899250649015585}, {'lat': 49.89983238337608, 'lon': 8.898930496376018}, {'lat': 49.8998798265736, 'lon': 8.899208835414811}, {'lat': 49.89986602214809, 'lon': 8.898905618316775}, {'lat': 49.89987837481331, 'lon': 8.899167021814264}, {'lat': 49.8998692169, 'lon': 8.89890325559}]
        self.current_line = {'k': 0, 'b': 0, 'angle': 0}
        self.course_pub = rospy.Publisher(f"{self.prefix}cmd_course", Course, queue_size=10)
        self.thrust_pub = rospy.Publisher(f"{self.prefix}cmd_drive", Drive, queue_size=10)
        #rospy.spin()
        rospy.Subscriber(f"/{self.prefix}navsat/fix", NavSatFix, self.get_coord)
        rospy.Subscriber(f"/{self.prefix}imu/data", Imu, self.get_orientation)
        self.k_lon = 111320
        self.k_lat = 111319*math.cos(self.way_coord[0]['lat']*math.pi/180)
        self.err_m = 1             # Ошибка GPS м
        self.err_lat = self.err_m/self.k_lat
        self.err_lon = self.err_m/self.k_lon
        self.err_temp_coord = 0.00002     # Ошибка GPS для поиска температуры по файлу
        self.err_temp = 2 # in grad
        self.normal_temp = 20
        self.temperature = 20
        self.current_angle = 0
        self.angle_step = 25*math.pi/180 # угол пооворота при нахождении загрязения (рад)
        self.prev_temp = 0
        self.prev_time = 0
        self.u_0 = 0.5

    # def look_herons(self):
    #     nodes = rosnode.get_node_names()
    #     for node in nodes:
    #         if node == "water_drone":
    #             self.herons.append(1)

    def get_coord(self, data):
        self.my_lat = data.latitude
        self.my_lon = data.longitude

    def get_temperature(self):
        with open(f'{self.path}utils/with_temp') as map_temp:
            for line in map_temp:
                js = literal_eval(line)
                if (abs(self.my_lat - js['lat']) < self.err_temp_coord) and (abs(self.my_lon - js['lon']) < self.err_temp_coord):
                    self.temperature = js['temp']
                    print(f'get_temperature, temperature = {self.temperature}')
                    break

    def line(self, target_lat, target_lon):
        # lon = k*lat + b
        self.current_line['k'] = (target_lon - self.my_lon)/(target_lat - self.my_lat)
        self.current_line['b'] = target_lon - self.current_line['k']*target_lat
        self.current_line['angle'] = math.pi/2 - math.atan2(target_lon - self.my_lon, target_lat - self.my_lat)
        # print(f'angle = {self.current_line["angle"]}')

    def go_pollution(self):
        initial_coord = {"lat": self.my_lat, "lon": self.my_lon}
        course_mess = Course()
        course_mess.speed = -1
        self.course_pub.publish(course_mess)
        time.sleep(3)
        count = 0
        right_angle = False
        while not right_angle:
            self.current_angle = self.current_angle - self.angle_step
            course_mess.speed = 0
            course_mess.yaw = self.current_angle
            self.course_pub.publish(course_mess)
            time.sleep(2)
            course_mess.speed = 0.8
            self.course_pub.publish(course_mess)
            time.sleep(5)
            plt.plot(self.my_lat, self.my_lon, 'ro')
            plt.draw()
            plt.pause(0.001)
            self.get_temperature()
            if abs(self.temperature - self.normal_temp) > self.err_temp:
                course_mess.speed = -1
                self.course_pub.publish(course_mess)
                time.sleep(5)
                plt.plot(self.my_lat, self.my_lon, 'ro')
                plt.draw()
                plt.pause(0.001)
            else:
                right_angle = True
                course_mess.speed = 0
                self.course_pub.publish(course_mess)
        while ((abs(self.my_lat - initial_coord["lat"]) > self.err_temp_coord/10) or (abs(self.my_lon - initial_coord["lon"]) > self.err_temp_coord/10)) or (count < 5):
            print(f'my lat - init {abs(self.my_lat - initial_coord["lat"])}, err {self.err_temp_coord/2},my lon - init {abs(self.my_lon - initial_coord["lon"])}, err {self.err_temp_coord/2}')
            self.get_temperature()
            print(f'temerature in pollution {self.temperature}')
            if abs(self.temperature - self.normal_temp) > self.err_temp:
                self.current_angle = self.current_angle - self.angle_step
                course_mess.yaw = self.current_angle
            else:
                self.current_angle = self.current_angle + self.angle_step*2
                course_mess.yaw = self.current_angle
            print(f'course {course_mess}')
            course_mess.speed = 0
            self.course_pub.publish(course_mess)
            count += 1
            time.sleep(8)
            course_mess.speed = 0.5
            self.course_pub.publish(course_mess)
            time.sleep(3)
            plt.plot(self.my_lat, self.my_lon, 'ro')
            plt.draw()
            plt.pause(0.001)
            with open(f'{self.path}utils/pollution_borders', 'a') as f4:
                data = {"lat": self.my_lat, "lon": self.my_lon, "temp": self.temperature}
                f4.write(f'{data} \n')

    def sign(self, a):
        if a > 0:
            return 1
        elif a < 0:
            return -1
        else:
            return 0

    def delta_temp(self):

        delta_temp = (current_temp - self.prev_temp)/(time.time()-self.prev_time)
        self.prev_temp = self.temperature
        self.prev_time = time.time()
        return delta_temp
    
    def get_orientation(self, data):
        self.current_angle = 2*math.acos(data.orientation.w)
        

    def inpollution_control(self):
        mu = 1
        speed_max = 1
        speed_min = -1
        drive_msg = Drive()
        self.get_temperature()
        self.u_0 = 0.5
        delta = self.delta_temp
        sigma = -self.sign((delta - mu*math.tanh(self.temperature - self.looking_value_temp)))
        drive_msg.right= 0.5*0.25*(1 - sigma) + u_0
        drive_msg.left = 0.5*0.25*(1 + sigma) + u_0
        self.thrust_pub.publish(drive_msg)



    def go_targets(self):
        time.sleep(1)
        #plt.figure()
        #plt.ion()
        #plt.show()
        f1 = open(f'{self.path}/utils/{self.boat_nomber}coord-err-1m', 'w')
        f2 = open(f'{self.path}/utils/{self.boat_nomber}coord-final-err-1m', 'w')
        final_coord = [[],[]]
        coords = []
        inPollution = False
        drive_msg = Drive()
        for target in self.way_coord:
            print(f'New target {target}')
            self.line(target['lat'], target['lon'])
            # self.current_angle = -self.current_line['angle'] + math.pi/2
            # course_mess = Course()
            # course_mess.yaw = -self.current_line['angle'] + math.pi/2
            # course_mess.speed = 0
            # self.course_pub.publish(course_mess)

            
            while (abs(self.my_lat - target['lat']) > self.err_lat) and (abs(self.my_lon - target['lon']) > self.err_lon):
                self.get_temperature()
                #print(f'temperature = {self.temperature}')
                if self.with_pollution_looking:
                    if abs(self.temperature - self.normal_temp) > self.err_temp:
                        try:
                            with open(f'{self.path}utils/pollution_borders') as f3:
                                for line in f3:
                                    line = literal_eval(line)
                                    #print(f'line = {line}')
                                    if (abs(line["lat"] - self.my_lat) < self.err_temp_coord*4) and (abs(line["lon"] - self.my_lon) < self.err_temp_coord*4):
                                        inPollution = True
                                        break
                        except FileNotFoundError:
                            pass

                        if not inPollution:
                            print(f'go pollution')
                            self.looking_value_temp = self.temperature
                            self.inpollution_control()
                    else:
                        inPollution = False

                self.line(target['lat'], target['lon'])
                drive_msg.left = self.u_0 + 0.01*(self.current_angle - self.current_line['angle'])
                drive_msg.right = self.u_0 - 0.01*(self.current_angle - self.current_line['angle'])
                print(f"req angle: {self.current_line['angle']}")
                print(f"curr angl : {self.current_angle}")
                print(f"pub left {drive_msg.left} right {drive_msg.right}")
                
                self.thrust_pub.publish(drive_msg)

                    
                    

                #print(f'my_lat = {self.my_lat}, my_lon = {self.my_lon}')
                # course_mess = Course()
                # course_mess.yaw = -self.current_line['angle'] + math.pi/2
                # course_mess.speed = 1
                # print(course_mess)
                # self.course_pub.publish(course_mess)
                plt.plot(self.my_lat, self.my_lon, 'ro')
                plt.draw()
                plt.pause(0.001)
                f1.write(f'{self.my_lat}; {self.my_lon}\n')
                coords.append({'lat':self.my_lat, 'lon': self.my_lon})
                time.sleep(2)
            # course_mess.speed = 0
            # self.course_pub.publish(course_mess)
            final_coord[0].append(self.my_lat)
            final_coord[1].append(self.my_lon)
            f2.write(f'{self.my_lat}; {self.my_lon}\n')
            time.sleep(2)
        print(f'final_coord = {final_coord}')
        f1.close()
        f2.close()
        f = open('final_coords_slovar', 'w')
        f.write(str(coords))
        f.close()
        #rospy.spin()

                
if __name__ == '__main__':  
    try:  
        drone = WaterDrone(0)
        drone.go_targets()
    except KeyboardInterrupt:
        print("exception")
        exit()

