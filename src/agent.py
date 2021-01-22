#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2021 Ashwin Rajesh
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys

sys.path.append("/HDD/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg")

import carla
import numpy as np
import time

def deg_to_rad(val):
    return val * np.pi / 180

class agent:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.world  = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.actor_list = []

    def spawn_vehicle(self):
        car_bp = np.random.choice(self.bp_lib.filter("vehicle.*.*"))
        car_tf = np.random.choice(self.world.get_map().get_spawn_points())

        self.vehicle = self.world.spawn_actor(car_bp, car_tf)
        self.actor_list.append(self.vehicle)
        self.vehicle.set_autopilot(True)

        print(" Spawning vehicle at location : %d, %d, %d"%(car_tf.location.x, car_tf.location.y, car_tf.location.z))
        print(" Vehicle model : %s %s"%(self.vehicle.type_id.split('.')[1], self.vehicle.type_id.split('.')[2]))

    def spawn_imu(self, period=0.1, length=500):
        if(self.vehicle == None):
            print(" Error : spawn car first")
            return

        imu_bp = self.bp_lib.find("sensor.other.imu")
        imu_bp.set_attribute('sensor_tick', '0.1')
        imu_tf = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))

        self.imu = self.world.spawn_actor(imu_bp, imu_tf, attach_to=self.vehicle)
        self.actor_list.append(self.imu)
        self.imu_time = 0
        self.imu_per  = period
        self.imu_len  = length
        self.imu_list = []
        self.imu.listen(self.imu_listen)

    def imu_listen(self, data):
        if(data.timestamp - self.imu_time < self.imu_per):
            return
        self.imu_time = data.timestamp
        self.imu_data = data

        if(len(self.imu_list) < self.imu_len):
            self.imu_list.append(data)

    def imu_reset(self):
        self.imu_list = []

    def spawn_gnss(self, period=0.1, length=500):
        if(self.vehicle == None):
            print(" Error : spawn car first")
            return

        gnss_bp = self.bp_lib.find("sensor.other.gnss")
        gnss_bp.set_attribute('sensor_tick', '0.1')
        gnss_tf = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))

        map_geo = self.world.get_map().transform_to_geolocation(carla.Location(0,0,0))
        self.geo_centre = {'lat': deg_to_rad(map_geo.latitude), 'lon': deg_to_rad(map_geo.longitude), 'alt':map_geo.altitude}
        self.geo_rad_y  = 6.357e6
        self.geo_rad_x  = 6.378e6

        self.gnss = self.world.spawn_actor(gnss_bp, gnss_tf, attach_to=self.vehicle)
        self.actor_list.append(self.gnss)
        self.gnss_time = 0
        self.gnss_per  = period
        self.gnss_len  = length
        self.gnss_list = []
        self.rpos_list = []
        self.gnss.listen(self.gnss_listen)
        
    def gnss_listen(self, data):
        if(data.timestamp - self.gnss_time < self.gnss_per):
            return
        self.gnss_time = data.timestamp
        self.gnss_data = data

        lat = deg_to_rad(data.latitude)
        lon = deg_to_rad(data.longitude)
        alt = data.altitude 
        
        x = (lon - self.geo_centre['lon']) * np.cos(self.geo_centre['lat']) * self.geo_rad_x
        y = (self.geo_centre['lat'] - lat) * self.geo_rad_y
        z = alt - self.geo_centre['alt']

        loc = self.vehicle.get_location()
        
        self.gnss_pos = [x, y, z]
        self.real_pos = [loc.x, loc.y, loc.z]
        self.gnss_err = [self.gnss_pos[i] - self.real_pos[i] for i in range(3)]
        
        if(len(self.gnss_list) < self.gnss_len):
            self.gnss_list.append((self.gnss_pos, data.timestamp))
            self.rpos_list.append((self.real_pos, data.timestamp))

    def gnss_reset(self):
        self.gnss_list = []
        self.rpos_list = []
        
    def __del__(self):
        for a in self.actor_list:
            a.destroy()
            self.actor_list.remove(a)
