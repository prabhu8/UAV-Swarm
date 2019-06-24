import time

import cflib.crtp
import csv
import json
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from typing import List, Dict
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
import signal
import sys

# Change uris and sequences according to your setup
DRONE0 = 'radio://0/70/2M/E7E7E7E701'
DRONE1 = 'radio://0/70/2M/E7E7E7E702'
DRONE2 = 'radio://0/70/2M/E7E7E7E703'
DRONE3 = 'radio://1/70/2M/E7E7E7E704'
DRONE4 = 'radio://1/70/2M/E7E7E7E705'
DRONE5 = 'radio://1/70/2M/E7E7E7E706'
DRONE6 = 'radio://2/80/2M/E7E7E7E707'
DRONE7 = 'radio://2/80/2M/E7E7E7E708'
DRONE8 = 'radio://2/80/2M/E7E7E7E709'
DRONE9 = 'radio://2/80/2M/E7E7E7E710'
DRONE10 = 'radio://1/80/2M/E7E7E7E711'
DRONE11 = 'radio://2/90/2M/E7E7E7E712'
DRONE12 = 'radio://2/90/2M/E7E7E7E713'
DRONE13 = 'radio://2/90/2M/E7E7E7E714'
DRONE14 = 'radio://2/90/2M/E7E7E7E715'
DRONE15 = 'radio://2/90/2M/E7E7E7E716'
DRONE16 = 'radio://3/100/2M/E7E7E7E717'
DRONE17 = 'radio://3/100/2M/E7E7E7E718'
DRONE18 = 'radio://3/100/2M/E7E7E7E719'
DRONE19 = 'radio://3/100/2M/E7E7E7E720'



# FIGURE8 = [
#     [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
#     [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
# ]

# List of URIs, comment the one you do not want to fly
#DRONE4 ## Faulty Drone // Does not work
trajectory_assigment = {
    0: DRONE0,
    1: DRONE1,
    2: DRONE2,
    3: DRONE3,
    #4: DRONE4,
    5: DRONE5,
    6: DRONE6,
    7: DRONE7,
    8: DRONE8,
    9: DRONE9,
    # 10: DRONE10,
    # 11: DRONE11,
    # 12: DRONE12,
    # 13: DRONE13,
    # 14: DRONE14,
    # 15: DRONE15,
    # 16: DRONE16,
    # 17: DRONE17,
    # 18: DRONE18,
}



class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def wait_for_position_estimator(scf: SyncCrazyflie):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def wait_for_param_download(scf: SyncCrazyflie):
    print("param")
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

def reset_estimator(scf: SyncCrazyflie):
    print("reset")
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def upload_trajectory(cf: Crazyflie, trajectory_id: int, trajectory: List):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration

def upload_sequence(scf: Crazyflie,trajectory: List, duration: float):
    try:
        print("upload")
        cf = scf.cf # type: Crazyflie
        trajectory_id = 1
        cf.param.set_value('commander.enHighLevel','1')
        upload_trajectory(cf, trajectory_id, trajectory)
        reset_estimator(scf)
    except Exception as e:
        print(e)

def go_sequence(scf: Crazyflie,trajectory: List, duration: float):
    try:
        cf = scf.cf # type: Crazyflie
        trajectory_id = 1       
        commander = cf.high_level_commander # type: cflib.HighLevelCOmmander
        print("go")
        commander.takeoff(1.0, 2.0)
        time.sleep(10.0)
        #relative = False
        #commander.start_trajectory(trajectory_id, 2.0, relative)
        #time.sleep(duration*2)
    except Exception as e:
        print(e)

def land_sequence(scf: Crazyflie,trajectory: List, duration: float):
    try:
        cf = scf.cf # type: Crazyflie
        commander = cf.high_level_commander # type: cflib.HighLevelCOmmander        
        print("land")
        time.sleep(3)
        commander.land(0.0, 2.0)
        time.sleep(2)
        commander.stop()
    except Exception as e:
        print(e)



if __name__ == '__main__':

    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    uris = {trajectory_assigment[key] for key in trajectory_assigment.keys()}
    print("uris:", uris)
    with open('formP.json', 'r') as f:
        traj_list = json.load(f)
    
    #building arguments list in swarm
    swarm_args = {}
    for key in trajectory_assigment.keys():
        print("key", key)
        trajectory = traj_list[str(key)]
        print("trajectory assigned")
        duration = 0
        print("duration assigned")
        for leg in trajectory:
            duration += leg[0]
        swarm_args[trajectory_assigment[key]] = [trajectory, duration]
        print("trajectory URI:", trajectory_assigment[key])
        print("duration:", duration)
                


    with Swarm(uris, factory=factory) as swarm:
        def signal_handler(sig, frame):
            print('You pressed Ctrl+C!')
            swarm.parallel(land_sequence, args_dict=swarm_args)
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)
        print('Press Ctrl+C')
    
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        print('Resetting estimators...')
        #swarm.parallel(reset_estimator)

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)

        #print('Setting up trajectories...')
        #swarm.parallel(setup_trajectory, args_dict = swarm_args)

        print('Running trajectory...')
        swarm.parallel(upload_sequence, args_dict=swarm_args)
        swarm.parallel(start_position_printing, args_dict = swarm_args)
        swarm.parallel(go_sequence, args_dict=swarm_args)
        swarm.parallel(land_sequence, args_dict=swarm_args)
