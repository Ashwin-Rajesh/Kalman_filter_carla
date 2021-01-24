# Kalman_filter_carla

Kalman filter for self driving cars using gnss and imu data. Data was collected from the CARLA open source simulator.

---

## Software used

- Ubuntu 18.04
- Carla 0.9.11
- Python 3.7 API
- Python 3.6 interpreter
- VS Code for editing
- IPython 5.5.0

---

## Data collection (CARLA agent)

- A class, ```agent``` is created that handles data collection from carla. [Code](src/agent.py)
- Creating the class creates the carla ```client```, ```blueprint library``` and ```world``` objects. See the carla documentation for details.
- ```spawn_vehicle``` spawns a vehicle at a location, based on the arguments. The vehicle is named ```vehicle```
  - argument : index    : The index of pre-defined location in the world at which to spawn the vehicle. If not mentioned, a random location is choosen.
- ```spawn_gnss``` spawns a gnss sensor, named ```gnss_sensor```
  - argument : ```per```        : Period for refreshing gnss data
  - argument : ```std_dev```    : Standard deviation of gnss data (in degrees for latitude and longitude)
- ```reg_gnss_callback``` registers the passed function as a callback that will be executed when the gnss data is available.
  - argument : ```callback```   : The function that will be called with the gnss data as the argument, when data is ready.
- ```spawn_imu``` spawns an IMU sensor, named ```imu_sensor```
  - arguemnt : ```per```        : Period for refreshing imu data
  - argument : ```accel_std_dev``` : Standard deviation for accelerometer
  - argument : ```gyro_std_dev```   : Standard deviation for gyroscope data.
- ```reg_imu_callback``` registers the passed function as a callback that will be executed when the imu data is available.
  - argument : ```callback```   : The function that will be called with the imu data as the argument, when data is ready.
- ```gnss_to_xyz``` converts gnss data passed into xyz coordinates in the current map, by comparing to the geographic coordinates(latitude, longitude) of the map centre to geographic coordinates in the passed gnss data.
  - argument : ```data```       : The gnss data to be converted
  - returns  : ```[x, y, z]```  : Standard cartesian coordinates


---

## References
1) [Carla documentation](https://carla.readthedocs.io/en/latest/python_api/)
2) [GNSS latitude and longitude to x-y coordinate conversion](https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y#:~:text=latitude%20%3D%20Math.,and%20y%20xPos%20%3D%20\(app.)
