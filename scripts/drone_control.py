#!/usr/bin/env python3
import rospy
import json
from heron_msgs.msg import Drive, Course, Helm
from sensor_msgs.msg import JointState, NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
from heron_sensors.msg import DronesStatus
from mavros_msgs.msg import OverrideRCIn
import time
import math
import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np
from datetime import datetime
import random
import rospkg

class WaterDrone:
    def __init__(self, boat_number):
        self.real = False       # Control real boat
        self.waves = True      # Generate waves
        self.looking_value_temp = 8.4
        self.with_pollution_looking = True      # Look for izoline
        self.boat_number = boat_number
        if self.boat_number == 0:
            self.prefix = ''
        else:
            self.prefix = f'drone{self.boat_number}/'
        #nodes = rosnode.get_node_names()
        rospack = rospkg.RosPack()
        rospack.list() 
        self.path = rospack.get_path('heron_sensors')
        rospy.loginfo(self.path)

        self.log_file = open(f'{self.path}/utils/logs/{datetime.now()}_drone_numbers_{self.boat_number}', 'w')
        rospy.init_node(f"water_drone{self.boat_number}", anonymous=False)

        x0 = 49.8988
        self.x_min = x0
        y0 = 8.89844
        self.y_min = y0
        x1 = 49.8998995
        y1 = 8.90168012
        shape_x = 413 + 1
        shape_y = 1248 + 1
        step_x = (x1 - x0)/shape_x
        self.x_step = step_x
        step_y = (y1 - y0)/shape_y
        self.y_step = step_y
        with open(f'{self.path}/utils/map/for_painting') as map_temp:
            lat = []
            lon = []
            temp = []
            r = 0.0002
            lat = np.arange(x0, x1, step_x)   # 49.8998995
            lon = np.arange(y0, y1, step_y)    # 8.9016801     
            rospy.loginfo(f'lat {lat}')
            for line in map_temp:
                line = line.split(';')
                #rospy.loginfo(line)
                temp.append(float(line[2]))

        Y, X = np.meshgrid(lon, lat)
        rospy.loginfo(f'X, Y {X}, {Y}')
        temp = np.array(temp)
        rospy.loginfo(f"temp {temp}")
        temp = temp.reshape((shape_y, shape_x))
        rospy.loginfo(f"temp reshaped {temp}")
        temp = np.transpose(temp)
        self.fig, self.ax = plt.subplots()
        rospy.loginfo(f'x shape {X.shape} z shape {temp.shape}')
        CS_common = self.ax.contour(X, Y, temp, 25)
        CS_lvl = self.ax.contour(X, Y, temp, levels=[self.looking_value_temp], colors='b')

        self.ax.clabel(CS_common, inline=True, fontsize=10)
        self.ax.clabel(CS_lvl, inline=True, fontsize=10)
        self.ax.set_title('Isoline follow')
        self.ax.set_xlabel("Latitude, °")
        self.ax.set_ylabel("Longitude, °")
        plt.draw()
        plt.pause(0.01)
            
        self.my_lat = 0
        self.my_lon = 0
        self.way_coord = []
        with open(f'{self.path}/utils/ways/way{self.boat_number}') as f:
            for line in f:
                self.way_coord.append(literal_eval(line))
        #self.way_coord = [{'lat': 49.8987564727, 'lon': 8.90169695784}, {'lat': 49.89878805768686, 'lon': 8.901669519301059}, {'lat': 49.89876901555181, 'lon': 8.901657043410466}, {'lat': 49.898819642673715, 'lon': 8.901642080762123}, {'lat': 49.89878155840361, 'lon': 8.901617128980973}, {'lat': 49.89885122766057, 'lon': 8.90161464222318}, {'lat': 49.89879410125542, 'lon': 8.901577214551452}, {'lat': 49.89888281264743, 'lon': 8.901587203684244}, {'lat': 49.898806644107225, 'lon': 8.901537300121959}, {'lat': 49.898914397634286, 'lon': 8.9015597651453}, {'lat': 49.89881918695903, 'lon': 8.901497385692437}, {'lat': 49.89894598262114, 'lon': 8.901532326606365}, {'lat': 49.89883172981084, 'lon': 8.901457471262916}, {'lat': 49.898977567608, 'lon': 8.901504888067421}, {'lat': 49.89884427266264, 'lon': 8.901417556833422}, {'lat': 49.89900915259486, 'lon': 8.901477449528485}, {'lat': 49.89885681551445, 'lon': 8.9013776424039}, {'lat': 49.899040737581714, 'lon': 8.901450010989542}, {'lat': 49.898869358366255, 'lon': 8.90133772797438}, {'lat': 49.89907232256857, 'lon': 8.901422572450606}, {'lat': 49.89888190121806, 'lon': 8.901297813544886}, {'lat': 49.89910390755543, 'lon': 8.901395133911663}, {'lat': 49.89889444406987, 'lon': 8.901257899115365}, {'lat': 49.899135492542285, 'lon': 8.901367695372727}, {'lat': 49.89890698692167, 'lon': 8.901217984685843}, {'lat': 49.89916707752914, 'lon': 8.901340256833784}, {'lat': 49.89891952977348, 'lon': 8.90117807025635}, {'lat': 49.899198662516, 'lon': 8.90131281829484}, {'lat': 49.898932072625286, 'lon': 8.901138155826828}, {'lat': 49.89923024750286, 'lon': 8.901285379755905}, {'lat': 49.89894461547709, 'lon': 8.901098241397335}, {'lat': 49.899261832489714, 'lon': 8.901257941216961}, {'lat': 49.8989571583289, 'lon': 8.901058326967814}, {'lat': 49.89929341747657, 'lon': 8.901230502678025}, {'lat': 49.898969701180704, 'lon': 8.901018412538292}, {'lat': 49.89932500246343, 'lon': 8.901203064139082}, {'lat': 49.89898224403251, 'lon': 8.900978498108799}, {'lat': 49.899356587450285, 'lon': 8.901175625600146}, {'lat': 49.898994786884316, 'lon': 8.900938583679277}, {'lat': 49.89938817243714, 'lon': 8.901148187061203}, {'lat': 49.89900732973612, 'lon': 8.900898669249756}, {'lat': 49.899419757424, 'lon': 8.901120748522267}, {'lat': 49.89901987258793, 'lon': 8.900858754820263}, {'lat': 49.899451342410856, 'lon': 8.901093309983324}, {'lat': 49.899032415439734, 'lon': 8.900818840390741}, {'lat': 49.89948292739771, 'lon': 8.901065871444388}, {'lat': 49.89904495829154, 'lon': 8.90077892596122}, {'lat': 49.89951451238457, 'lon': 8.901038432905445}, {'lat': 49.899057501143346, 'lon': 8.900739011531726}, {'lat': 49.89954609737143, 'lon': 8.901010994366509}, {'lat': 49.89907004399515, 'lon': 8.900699097102205}, {'lat': 49.899577682358284, 'lon': 8.900983555827565}, {'lat': 49.89908258684696, 'lon': 8.900659182672683}, {'lat': 49.89960926734514, 'lon': 8.900956117288622}, {'lat': 49.899095129698765, 'lon': 8.90061926824319}, {'lat': 49.899640852332, 'lon': 8.900928678749686}, {'lat': 49.89910767255057, 'lon': 8.900579353813669}, {'lat': 49.899672437318856, 'lon': 8.900901240210743}, {'lat': 49.89912021540238, 'lon': 8.900539439384175}, {'lat': 49.89970402230571, 'lon': 8.900873801671807}, {'lat': 49.89913275825418, 'lon': 8.900499524954654}, {'lat': 49.89973560729257, 'lon': 8.900846363132864}, {'lat': 49.89914530110599, 'lon': 8.900459610525132}, {'lat': 49.89976719227943, 'lon': 8.900818924593928}, {'lat': 49.899157843957795, 'lon': 8.90041969609564}, {'lat': 49.899798777266284, 'lon': 8.900791486054985}, {'lat': 49.8991703868096, 'lon': 8.900379781666118}, {'lat': 49.89983036225314, 'lon': 8.900764047516049}, {'lat': 49.89918292966141, 'lon': 8.900339867236596}, {'lat': 49.89986194724, 'lon': 8.900736608977105}, {'lat': 49.89919547251321, 'lon': 8.900299952807103}, {'lat': 49.899893532226855, 'lon': 8.90070917043817}, {'lat': 49.89920801536502, 'lon': 8.900260038377581}, {'lat': 49.89992511721371, 'lon': 8.900681731899226}, {'lat': 49.899220558216825, 'lon': 8.90022012394806}, {'lat': 49.899925637, 'lon': 8.90068128035}, {'lat': 49.89923310106863, 'lon': 8.900180209518567}, {'lat': 49.89992458696488, 'lon': 8.900639454732755}, {'lat': 49.89924564392044, 'lon': 8.900140295089045}, {'lat': 49.899923536929755, 'lon': 8.900597629115737}, {'lat': 49.899258186772244, 'lon': 8.900100380659524}, {'lat': 49.89992248689463, 'lon': 8.900555803498492}, {'lat': 49.89927072962405, 'lon': 8.90006046623003}, {'lat': 49.89992143685951, 'lon': 8.900513977881474}, {'lat': 49.899283272475856, 'lon': 8.900020551800509}, {'lat': 49.899920386824384, 'lon': 8.900472152264228}, {'lat': 49.89929581532766, 'lon': 8.899980637371016}, {'lat': 49.89991933678926, 'lon': 8.90043032664721}, {'lat': 49.89930835817947, 'lon': 8.899940722941494}, {'lat': 49.899918286754136, 'lon': 8.900388501029965}, {'lat': 49.899320901031274, 'lon': 8.899900808511973}, {'lat': 49.89991723671901, 'lon': 8.900346675412948}, {'lat': 49.89933344388308, 'lon': 8.89986089408248}, {'lat': 49.89991618668389, 'lon': 8.900304849795702}, {'lat': 49.899345986734886, 'lon': 8.899820979652958}, {'lat': 49.899915136648765, 'lon': 8.900263024178685}, {'lat': 49.89935852958669, 'lon': 8.899781065223436}, {'lat': 49.89991408661364, 'lon': 8.90022119856144}, {'lat': 49.8993710724385, 'lon': 8.899741150793943}, {'lat': 49.89991303657852, 'lon': 8.900179372944422}, {'lat': 49.899383615290304, 'lon': 8.899701236364422}, {'lat': 49.899911986543394, 'lon': 8.900137547327176}, {'lat': 49.89939615814211, 'lon': 8.8996613219349}, {'lat': 49.8999117653, 'lon': 8.900128734630016}, {'lat': 49.89940870099392, 'lon': 8.899621407505407}, {'lat': 49.899910313539706, 'lon': 8.900086921029242}, {'lat': 49.89942124384572, 'lon': 8.899581493075885}, {'lat': 49.899908861779416, 'lon': 8.900045107428696}, {'lat': 49.89943378669753, 'lon': 8.899541578646392}, {'lat': 49.899907410019125, 'lon': 8.900003293827922}, {'lat': 49.899446329549335, 'lon': 8.89950166421687}, {'lat': 49.899905958258834, 'lon': 8.899961480227148}, {'lat': 49.89945887240114, 'lon': 8.89946174978735}, {'lat': 49.899904506498544, 'lon': 8.899919666626602}, {'lat': 49.89947141525295, 'lon': 8.899421835357856}, {'lat': 49.89990305473825, 'lon': 8.899877853025828}, {'lat': 49.89948395810475, 'lon': 8.899381920928334}, {'lat': 49.89990160297796, 'lon': 8.899836039425281}, {'lat': 49.89949650095656, 'lon': 8.899342006498813}, {'lat': 49.89990015121767, 'lon': 8.899794225824508}, {'lat': 49.899509043808365, 'lon': 8.89930209206932}, {'lat': 49.89989869945738, 'lon': 8.899752412223734}, {'lat': 49.89952158666017, 'lon': 8.899262177639798}, {'lat': 49.89989724769709, 'lon': 8.899710598623187}, {'lat': 49.89953412951198, 'lon': 8.899222263210277}, {'lat': 49.8998957959368, 'lon': 8.899668785022413}, {'lat': 49.89954667236378, 'lon': 8.899182348780784}, {'lat': 49.89989434417651, 'lon': 8.89962697142164}, {'lat': 49.89955921521559, 'lon': 8.899142434351262}, {'lat': 49.89989289241622, 'lon': 8.899585157821093}, {'lat': 49.8995632732, 'lon': 8.899129520849982}, {'lat': 49.89989144065593, 'lon': 8.89954334422032}, {'lat': 49.89959691197201, 'lon': 8.899104642790746}, {'lat': 49.89988998889564, 'lon': 8.899501530619773}, {'lat': 49.89963055074402, 'lon': 8.899079764731503}, {'lat': 49.89988853713535, 'lon': 8.899459717018999}, {'lat': 49.89966418951603, 'lon': 8.899054886672253}, {'lat': 49.899887085375056, 'lon': 8.899417903418225}, {'lat': 49.89969782828804, 'lon': 8.89903000861301}, {'lat': 49.899885633614765, 'lon': 8.899376089817679}, {'lat': 49.89973146706005, 'lon': 8.89900513055376}, {'lat': 49.899884181854475, 'lon': 8.899334276216905}, {'lat': 49.89976510583206, 'lon': 8.89898025249451}, {'lat': 49.899882730094184, 'lon': 8.899292462616359}, {'lat': 49.89979874460407, 'lon': 8.898955374435268}, {'lat': 49.89988127833389, 'lon': 8.899250649015585}, {'lat': 49.89983238337608, 'lon': 8.898930496376018}, {'lat': 49.8998798265736, 'lon': 8.899208835414811}, {'lat': 49.89986602214809, 'lon': 8.898905618316775}, {'lat': 49.89987837481331, 'lon': 8.899167021814264}, {'lat': 49.8998692169, 'lon': 8.89890325559}]
        self.current_line = {'k': 0, 'b': 0, 'angle': 0}

        ##################
        ### Real drone ###
        ##################

        if self.real:
            self.rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
            rospy.Subscriber(f"/mavros/imu/data", Imu, self.get_orientation)
            rospy.Subscriber(f"/mavros/global_position/global", NavSatFix, self.get_coord)

        #############
        ### Heron ###
        #############

        else:
            rospy.Subscriber(f"/{self.prefix}navsat/fix", NavSatFix, self.get_coord)
            #rospy.Subscriber(f"/{self.prefix}gps/fix", NavSatFix, self.get_coord)
            rospy.Subscriber(f"/{self.prefix}imu/data", Imu, self.get_orientation)
            self.course_pub = rospy.Publisher(f"{self.prefix}cmd_course", Course, queue_size=10)
            self.thrust_pub = rospy.Publisher(f"{self.prefix}cmd_drive", Drive, queue_size=10)
            self.helm_pub = rospy.Publisher(f"{self.prefix}cmd_helm", Helm, queue_size=10)

        self.status_pub = rospy.Publisher(f"drones_status", DronesStatus, queue_size=10)
        rospy.Subscriber(f"/drones_status", DronesStatus, self.get_drones_status)
        self.k_lon = 111320
        self.k_lat = 111319*math.cos(self.way_coord[0]['lat']*math.pi/180)
        self.err_m = 4             # Ошибка GPS м
        self.err_lat = self.err_m/self.k_lat
        self.err_lon = self.err_m/self.k_lon
        self.err_temp_coord = 0.000002     # Ошибка GPS для поиска температуры по файлу
        self.err_temp = 2 # in grad
        self.normal_temp = 20
        self.temperature = 20
        self.current_angle = 0
        self.angle_step = 25*math.pi/180 # угол пооворота при нахождении загрязения (рад)
        self.prev_temp = 0
        self.prev_time = 0
        self.u_0 = 0.3
        #self.f1 = plt.figure(1)
        #self.f2 = plt.figure(2)
        self.arr_x = []
        self.arr_y = []
        self.drones_status = {}
        self.risk = []
        self.risk_zone = 1.5   # м, зона где дроны отходят друг от друга
        self.current_angle_coord = 0  # текущий угол вычисленный по координатам
        self.stopped = False
        self.noise = 0
        self.wave = 0
        self.time = 0

    def _euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    def get_coord(self, data):
        if self.real:
            plus_lat = 0
            plus_lon = 0
        else:
            plus_lat = 0
            plus_lon = 0
        if self.waves:
            self.wave = 0.000008*(math.sin(1000*(data.latitude + plus_lat)) + math.sin(1000*(data.longitude + plus_lon))) + 0.000008*0.01*math.sin(0.087)*math.cos(0.087)*self.time
            self.time += 1
            #rospy.loginfo(f"Wave: {self.wave}")
        else: 
            self.wave = 0
        dif_lat = data.latitude - self.my_lat + plus_lat
        dif_lon = data.longitude - self.my_lon + plus_lon
        self.my_lat = data.latitude + plus_lat + self.wave
        self.my_lon = data.longitude + plus_lon + self.wave
        
        angle = math.pi/2 - math.atan2(dif_lon, dif_lat)
        if angle < 0:
            self.current_angle_coord = angle + 2*math.pi
        else:
            self.current_angle_coord = angle
    
    def change_angle(self, prev_angle, step):
        next_angle = prev_angle + step
        if next_angle > 2*math.pi:
            next_angle = next_angle - 2*math.pi 
        elif next_angle < 0:
            next_angle = next_angle + 2*math.pi
        return next_angle

    def compare_angles(self, angle1, angle2):       # returns True if angle1 > angle2 (против часовой до 180 от первого угла)
        if abs(angle2 - angle1) < math.pi:
            return angle1 > angle2
        else:
            return angle2 > angle1

    def collision_control(self, boat_num, file=None):
        print('collision control')
        angle_collision = 0.72
        angle = self.line(self.drones_status[f'{boat_num}']['lat'], self.drones_status[f'{boat_num}']['lon'])
        print(f'collision control angle: {angle}, current angle {self.current_angle}')
        min_angle = self.change_angle(self.current_angle, -angle_collision/2)
        max_angle = self.change_angle(self.current_angle, angle_collision/2)
        boat_is_in_angle = False
        if max_angle > min_angle:
            if (angle > min_angle) and (angle < max_angle):
                boat_is_in_angle = True
        else:
            if (angle < max_angle) or (angle > min_angle):
                boat_is_in_angle = True

        if boat_is_in_angle:
            print('boat collision is forward')
            self.stop()
            if self.compare_angles(self.current_angle, angle):
                print('right')
                req_angle = self.change_angle(self.current_angle, 1.4)
                self.turn(req_angle, 0, file=file)
                for i in range(2):
                    req_angle = self.change_angle(self.current_angle, -0.7)
                    self.turn(req_angle, 0.4, file=file)
            else:
                print('left')
                req_angle = self.change_angle(self.current_angle, -1.4)
                self.turn(req_angle, 0, file=file)
                for i in range(2):
                    req_angle = self.change_angle(self.current_angle, 0.7)
                    self.turn(req_angle, 0.4, file=file)
        print('collision control end')

    def is_risk(self, number, lat, lon):
        if number != self.boat_number:
            print(f'is risk my_lat - lat: {self.my_lat - lat} er: {self.risk_zone/self.k_lat}, lon: {self.my_lon - lon}, er: {self.risk_zone*10/self.k_lon}')
            if (abs(self.my_lat - lat) < self.risk_zone/self.k_lat) and (abs(self.my_lon - lon) < self.risk_zone*10/self.k_lon):
                if number not in self.risk:
                    self.risk.append(number)
                    print(f'risk append {self.risk}')
            else:
                if number in self.risk:
                    self.risk.remove(number)
                    print(f'risk remove {self.risk}')


    def get_drones_status(self, data):
        self.is_risk(data.boat_number, data.latitude, data.longitude)
        if f'{data.boat_number}' in self.drones_status:
            self.drones_status[f'{data.boat_number}']['lat'] = data.latitude
            self.drones_status[f'{data.boat_number}']['lon'] = data.longitude
            self.drones_status[f'{data.boat_number}']['temp'] = data.temperature
        else:
            self.drones_status[f'{data.boat_number}'] = {'lat': data.latitude, 'lon': data.longitude, 'temp': data.temperature}

    def get_temperature(self):
        self.noise = random.gauss(0, 0.01)
        rospy.loginfo(f"temp noise = {self.noise}")
        n = 4
        temp_sum = 0
        temp_num = 0
        for i in range(n):
            with open(f'{self.path}/utils/map/for_control') as map_temp:
                rospy.loginfo(f'temp my lat {self.my_lat}, lon {self.my_lon}')
                y = 0
                for line in map_temp:
                    x = 0
                    #col = line.split(';')
                    col = line.split()
                    for temp in col:
                        lat = float(x)*self.x_step + self.x_min
                        lon = float(y)*self.y_step + self.y_min
                        if (abs(self.my_lat - lat) < self.err_temp_coord) and (abs(self.my_lon - lon) < self.err_temp_coord):
                            temp_sum += float(temp) + self.noise
                            temp_num += 1
                            #self.temperature = float(temp) + self.noise
                            rospy.loginfo(f'get_temperature, temperature = {self.temperature}')
                            break
                            #return
                        x += 1
                    y += 1
        try:
            self.temperature = temp_sum/temp_num
        except:
            self.temperature = 0
                
    def line(self, target_lat, target_lon):
        # lon = k*lat + b
        self.current_line['k'] = (target_lon - self.my_lon)/(target_lat - self.my_lat)
        self.current_line['b'] = target_lon - self.current_line['k']*target_lat
        #self.current_line['angle'] = math.pi/2 - math.atan2(target_lon - self.my_lon, target_lat - self.my_lat)
        angle = math.pi/2 - math.atan2(target_lon - self.my_lon, target_lat - self.my_lat)
        # if angle < 0:
        #     angle = angle + 2*math.pi
        # elif angle > 2*math.pi:
        #     angle = angle - 2*math.pi
        # else:
        #     angle = angle

        print(f'angle = {self.current_line["angle"]}')
        return angle
       
    def sign(self, a):
        if a > 0:
            return 1
        elif a < 0:
            return -1
        else:
            return 0

    def delta_temp(self):

        delta_temp = (self.temperature - self.prev_temp)/(time.time()-self.prev_time)
        delta_time = time.time()-self.prev_time
        self.prev_time = time.time()
        rospy.loginfo(f'delta prev temp {self.prev_temp}, temp {self.temperature} time {time.time()-self.prev_time}')
        self.prev_temp = self.temperature
        return delta_temp
    
    def get_orientation(self, data):
        roll, pitch, yaw = self._euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.current_angle = yaw
        # if self.real:
        #     if (data.orientation.w < -0.7) and (data.orientation.w > -1) and (data.orientation.z > 0):
        #         angle = 2*math.acos(-data.orientation.w)
        #         self.current_angle = self.change_angle(angle, 0.5)
        #     else:
        #         angle = 2*math.acos(data.orientation.w)
        #         self.current_angle = self.change_angle(angle, 0.5)
        #     print(f'get orientation {self.current_angle}')
        # else:
        #     self.current_angle = 2*math.acos(data.orientation.w)
    

    def inpollution_control(self):
        mu = 0.02
        helm_msg = Helm()
        rcin_msg = OverrideRCIn()
        self.get_temperature()
        u_max = -0.15
        u_min = -u_max
        th = 0.15
        angle = 0
        prev_time = time.time()
        # n = 0
        while True:
            self.get_temperature()
            delta = self.delta_temp()
            sigma = -self.sign((delta + mu*math.tanh(self.temperature - self.looking_value_temp)))
            #sigma = -self.sign((delta))
            u = 0.5*((1 - sigma)*u_min + (1 + sigma)*u_max)
            if self.real:
                turn = 1469 + 481*u   # left - 985 - 1951 - right
                go = 1467 + 481*th   # 985 - 1953
                rcin_msg.channels = [turn, 1466, go, 1469, 1467, 1954, 1467, 1502]
                self.rc_pub.publish(rcin_msg)
            else:
                helm_msg.thrust = th
                helm_msg.yaw_rate = u
                self.helm_pub.publish(helm_msg)
            rospy.loginfo(f"pub yaw rate {helm_msg.yaw_rate}")
            rospy.loginfo(f"looking_temp {self.looking_value_temp}")
            rospy.loginfo(f"temp: {self.temperature}, delta: {delta}, sigma: {sigma}")
            # self.ax[0].plot(self.my_lat, self.my_lon, 'ro')
            self.ax.plot(self.my_lat, self.my_lon, 'ro', markersize=1)
            plt.draw()
            plt.pause(0.01)
            self.write_log()

    def stop(self):
        print('stop')
        if self.real:
            rcin_msg = OverrideRCIn()
            if not self.stopped:
                turn = 1469   # left - 985 - 1951 - right
                go = 1467 - 100   # 985 - 1953
                rcin_msg.channels = [turn, 1466, go, 1469, 1467, 1954, 1467, 1502]
                self.rc_pub.publish(rcin_msg)
                time.sleep(0.7)
            go = 1467   # 985 - 1953
            turn = 1469
            rcin_msg.channels = [turn, 1466, go, 1469, 1467, 1954, 1467, 1502]
            self.rc_pub.publish(rcin_msg)

        else:
            drive_msg = Drive()
            if not self.stopped:
                drive_msg.left = -0.3
                drive_msg.right = -0.3
                self.thrust_pub.publish(drive_msg)
                time.sleep(0.7)
            drive_msg.left = 0
            drive_msg.right = 0
            time.sleep(0.7)
            self.thrust_pub.publish(drive_msg)
        self.stopped = True
        self.pub_status()

    def pub_status(self):
        status_msg = DronesStatus()
        status_msg.boat_number = self.boat_number
        status_msg.latitude = self.my_lat
        status_msg.longitude = self.my_lon
        status_msg.temperature = self.temperature
        self.status_pub.publish(status_msg)

    def write_log(self):
        log = {"lat": self.my_lat, "lon": self.my_lon, "value": self.temperature, "noise": self.noise, "wave": self.wave}
        self.log_file.write(f'{log}\n')


    def turn(self, angle, forward):
        self.stopped = False
        status_msg = DronesStatus()
        drive_msg = Drive()
        rcin_msg = OverrideRCIn()
        angle_err = 0.2
        while abs(self.current_angle - angle) > angle_err:
            time.sleep(0.8)

            if self.real:
                turn = int(1469 - 150*math.tanh(self.current_angle - angle))   # left - 985 - 1951 - right
                go = int(1467 + forward*486)    # 985 - 1953
                print(f'turn real turn: {turn}, go: {go}, cur - angle {self.current_angle - angle}')
                print(f'angle {angle}, cur angle {self.current_angle}')
                rcin_msg.channels = [turn, 1466, go, 1469, 1467, 1954, 1467, 1502]
                self.rc_pub.publish(rcin_msg)

            else:
                drive_msg.left = forward + 0.1*(self.current_angle - angle)
                drive_msg.right = forward - 0.1*(self.current_angle - angle)
                print(f"req angle in while: {angle}")
                print(f"curr angle in while : {self.current_angle}")
                print(f"pub left {drive_msg.left} right {drive_msg.right}")
                self.thrust_pub.publish(drive_msg)
            self.ax.plot(self.my_lat, self.my_lon, 'ro')
            plt.draw()
            plt.pause(0.001)
            self.write_log()

            self.pub_status()


    def go_targets(self):
        #time.sleep(0.1)
        inPollution = False
        angle_err = 0.05
        drive_msg = Drive()
        helm_msg = Helm()
        rcin_msg = OverrideRCIn()
        for target in self.way_coord:
            rospy.loginfo(f'New target {target}')
            self.current_line['angle'] = self.line(target['lat'], target['lon'])
            self.turn(self.current_line["angle"], 0)
            
            while (abs(self.my_lat - target['lat']) > self.err_lat) and (abs(self.my_lon - target['lon']) > self.err_lon):
                self.get_temperature()
                print(f'temperature = {self.temperature}')

                ######################################
                ### Проверка на нужную температуру ###
                ######################################

                if self.with_pollution_looking:
                    #if abs(self.temperature - self.normal_temp) > self.err_temp:
                    if abs(self.temperature - self.looking_value_temp) < 0.02:
                        try:
                            with open(f'{self.path}/utils/pollution_borders') as f3:
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
                            #self.looking_value_temp = self.temperature
                            self.inpollution_control()
                    else:
                        inPollution = False

                self.current_line['angle'] = self.line(target['lat'], target['lon'])

                ####################################################
                ### Проверка на столкновение с другими дронами ###
                ####################################################

                if len(self.risk) > 0:
                    rospy.loginfo(f'In risk with {self.risk} boats')
                    try:
                        if max(self.risk) < self.boat_number:
                            self.collision_control(self.risk[0], file=f1)
                        else:
                            self.stop()
                            continue
                    except Exception as e:
                        print(e)

                self.stopped = False

                if self.real:
                    turn = int(1469 - 150*math.tanh(self.current_angle - self.current_line['angle']))   # left - 985 - 1951 - right
                    go = 1467 + 200    # 985 - 1953
                    print(f'go targets angle line {self.current_line["angle"]}, cur angle {self.current_angle}')
                    print(f'go targets go: {go}, turn {turn}')
                    rcin_msg.channels = [turn, 1466, go, 1469, 1467, 1954, 1467, 1502]
                    self.rc_pub.publish(rcin_msg)
                else:
                    helm_msg.thrust = self.u_0
                    helm_msg.yaw_rate = 0.2*(self.current_angle - self.current_line['angle'])
                    self.helm_pub.publish(helm_msg)
                    print(f"req angle: {self.current_line['angle']}")
                    print(f"curr angl : {self.current_angle}")
                    print(f"pub th {helm_msg.thrust} yaw_rate {helm_msg.yaw_rate}")

                rospy.loginfo(f'my_lat = {self.my_lat}, my_lon = {self.my_lon}')
                self.pub_status()
                self.ax.plot(self.my_lat, self.my_lon, 'ro')
                plt.draw()
                plt.pause(0.001)
                time.sleep(2)
            self.write_log()
            time.sleep(2)

                
if __name__ == '__main__':  
    try:  
        drone = WaterDrone(0)
        # drone.go_targets()
        drone.inpollution_control()
    except KeyboardInterrupt:
        rospy.loginfo("exception")
        exit()