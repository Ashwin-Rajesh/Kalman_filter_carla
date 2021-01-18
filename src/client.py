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

class client:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.world  = client.get_world()
        self.bp_lib = world.get_blueprint_library()

    def spawn_car(self):
        car_bp = np.choice(self.bp_lib.filter("vehicle.*.*"))
        car_tf = np.choice(self.world.get_map().get_spawn_points())

        self.car = self.world.spawn_actor(car_bp, car_tf)

        print(" Spawning car at location : %d, %d, %d"%(car_tf.rotation.x, car_tf.rotation.y, car_tf.rotation.z))

    def spawn_imu(self):
        if(self.car == None):
            print(" Error : spawn car first")
            return

        imu_bp = self.bp_lib.find("sensor.other.imu")
        imu_tf = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))

        self.imu = self.world.spawn_actor(imu_bp, imu_tf, attach_to=self.car)

        self.imu.listen(self.imu_listen)

    def imu_listen(self, data):
        self.data = data
