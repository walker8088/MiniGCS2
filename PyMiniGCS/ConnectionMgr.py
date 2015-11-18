
#coding: utf-8

import sys, os, struct, math, time, socket
import fnmatch, errno, threading

from pymavlink import *

from ParamMgr import *

class Vector3(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
def get_usec():
    '''time since 1970 in microseconds'''
    return int(time.time() * 1.0e6)

def report_altitude(altitude):
    '''possibly report a new altitude'''
    master = self.master()
    if getattr(self.console, 'ElevationMap', None) is not None and self.settings.basealt != 0:
        lat = master.field('GLOBAL_POSITION_INT', 'lat', 0)*1.0e-7
        lon = master.field('GLOBAL_POSITION_INT', 'lon', 0)*1.0e-7
        alt1 = self.console.ElevationMap.GetElevation(lat, lon)
        alt2 = self.settings.basealt
        altitude += alt2 - alt1
    self.status.altitude = altitude
    if (int(self.settings.altreadout) > 0 and
        math.fabs(self.status.altitude - self.status.last_altitude_announce) >= int(self.settings.altreadout)):
        self.status.last_altitude_announce = self.status.altitude
        rounded_alt = int(self.settings.altreadout) * ((self.settings.altreadout/2 + int(self.status.altitude)) / int(self.settings.altreadout))
        say("height %u" % rounded_alt, priority='notification')

def periodic_tasks():
    '''run periodic checks'''
    if self.status.setup_mode:
        return

    if self.settings.heartbeat != 0:
        heartbeat_period.frequency = self.settings.heartbeat

    if heartbeat_period.trigger() and self.settings.heartbeat != 0:
        self.status.counters['MasterOut'] += 1
        for master in self.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    if battery_period.trigger():
        battery_report()


#----------------------------------------------------------#
class MasterStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        
        self.gps         = None
        
        self.msgs = {}
        self.msg_count = {}
        
        self.counters = {'MasterIn' : 0, 'MasterOut' : 0}
        
        self.target_system = -1
        self.target_component = -1
        
        self.altitude = 0
        self.last_altitude_announce = 0.0
        self.last_distance_announce = 0.0
        
        self.last_battery_announce = 0
        self.last_avionics_battery_announce = 0
        self.battery_level = -1
        self.voltage_level = -1
        self.avionics_battery_level = -1
        
        self.exit = False
        
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        
        self.logdir = None
        
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = True
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.highest_msec = 0
        
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        
        self.watch = None
        
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        
        self.last_seq = 0
        
        self.armed = False
        
        self.last_msg = {}
        
        self.last_sysinfo = {}
        self.last_sensors = {}
        self.last_io = {}
        self.last_nav = {}
        
        self.mav_param = mavparm.MAVParmDict()
         
    def show(self, f, pattern=None):
        '''write status to status.txt'''
        if pattern is None:
            f.write('Counters: ')
            for c in self.counters:
                f.write('%s:%s ' % (c, self.counters[c]))
            f.write('\n')
            f.write('MAV Errors: %u\n' % self.mav_error)
            f.write(str(self.gps)+'\n')
        for m in sorted(self.msgs.keys()):
            if pattern is not None and not fnmatch.fnmatch(str(m).upper(), pattern.upper()):
                continue
            f.write("%u: %s\n" % (self.msg_count[m], str(self.msgs[m])))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()

#----------------------------------------------------------#
class ConnectionManager():
    def __init__(self,  port,  baud, observer):
        self.port  = port
        self.baud = baud
        self.observer = observer
        
        self.conn = None
        self.read_lock = threading.Lock()
        self.running = False
        
        self.param_mgr = None 
        
    def start_request_data_stream(self,  master): 
        master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 0, 10, 1)
        
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 1, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 2, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 3, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 4, 1, 1)
        #master.mav.request_data_stream_send(self.status.target_system, self.status.target_component, 6, 1, 1)
        
        """
        # MAV_DATA_STREAM
        MAV_DATA_STREAM_ALL = 0 # Enable all data streams
        MAV_DATA_STREAM_RAW_SENSORS = 1 # Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
        MAV_DATA_STREAM_EXTENDED_STATUS = 2 # Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
        MAV_DATA_STREAM_RC_CHANNELS = 3 # Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
        MAV_DATA_STREAM_RAW_CONTROLLER = 4 # Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT,
                # NAV_CONTROLLER_OUTPUT.
        MAV_DATA_STREAM_POSITION = 6 # Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.

        target_system             : The target requested to send the message stream. (uint8_t)
        target_component          : The target requested to send the message stream. (uint8_t)
        req_stream_id             : The ID of the requested data stream (uint8_t)
        req_message_rate          : The requested interval between two messages of this type (uint16_t)
        start_stop                : 1 to start sending, 0 to stop sending. (uint8_t
        """ 
    
    def on_msg_heartbeat(self, m, master):
        
        if (self.status.target_system != m.get_srcSystem() or self.status.target_component != m.get_srcComponent()):
            self.status.target_system = m.get_srcSystem()
            self.status.target_component = m.get_srcComponent()
            self.start_request_data_stream(master)
               
        self.status.last_heartbeat = time.time()
        #master.last_heartbeat =self.status.last_heartbeat
        
        """
        armed = master.motors_armed()
        if armed != self.status.armed:
            self.status.armed = armed
            if armed:
                self.observer.info("ARMED")
            else:
                self.observer.info("DISARMED")
        """
        
        if m.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING]:
            master.vehicle_type = 'plane'
            master.vehicle_name = 'ArduPlane'
        elif m.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                        mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                        mavutil.mavlink.MAV_TYPE_SUBMARINE]:
            master.vehicle_type = 'rover'
            master.vehicle_name = 'APMrover2'
        elif m.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                        mavutil.mavlink.MAV_TYPE_COAXIAL,
                        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                        mavutil.mavlink.MAV_TYPE_TRICOPTER,
                        mavutil.mavlink.MAV_TYPE_HELICOPTER]:
            master.vehicle_type = 'copter'
            master.vehicle_name = 'ArduCopter'
        else:
            print "type error"
            
    def on_msg_sensor_offsets(self, m, master):
        self.status.accel_offsets = Vector3(m.accel_cal_x, m.accel_cal_y, m.accel_cal_z)
        self.status.gyro_offsets = Vector3(m.gyro_cal_x, m.gyro_cal_y, m.gyro_cal_z)
        self.status.mag_offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)#, m.mag_declination)
        self.status.press = (m.raw_press, m.raw_temp)
        
    def on_msg_scaled_imu2(self, m, master):
        accv3 = Vector3(m.xacc, m.yacc, m.zacc)
        gyrov3 = Vector3(m.xgyro, m.ygyro, m.zgyro)
        magv3 = Vector3(m.xmag, m.ymag, m.zmag)
        
        #print accv3, gyrov3, magv3 
    
    def on_msg_scaled_pressure(self, m, master):
        press = (m.press_abs, m.press_diff, m.temperature)
        #print press
        
    def on_msg_sys_status(self, m, master):
        pass
        #self.status.last_status[m.get_type()] = m
        """
        onboard_control_sensors_present : 6355983, 
        onboard_control_sensors_enabled : 6331407, 
        onboard_control_sensors_health : 2161679, 
        load : 158, 
        voltage_battery : 0, 
        current_battery : -1, 
        battery_remaining : -1, 
        drop_rate_comm : 0, 
        errors_comm : 0, 
        errors_count1 : 0, 
        errors_count2 : 0, 
        errors_count3 : 0, 
        errors_count4 : 0
        """
    
    def on_msg_power_status(self, m, master):
        #battery_update(m)
        #self.status.last_status[m.get_type()] = m
        pass
        
    def on_msg_meminfo(self, m, master):
        #self.status.last_status[m.get_type()] = m
        pass
        
    def on_msg_system_time(self, m, master):
        #self.status.last_status[m.get_type()] = m
        pass
        
    def on_msg_hwstatus(self, m, master):
        #self.status.last_status[m.get_type()] = m
        pass
        
    def on_msg_statustext(self, m, master):
        severitys = ("EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG")
        self.observer.info(severitys[m.severity] +": " + m.text)
        
    def on_msg_rc_channels_raw(self, m, master):
        self.status.raw_rc_chanels = (m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw)
        
    def on_msg_servo_output_raw(self, m, master):
        self.status.raw_servo_outs = (m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw,m.servo5_raw, m.servo6_raw, m.servo7_raw, m.servo8_raw)
        
    def on_msg_gps_raw_int(self, m, master):
        pass
        """
        fix_type : 0, 
        lat : 0, 
        lon : 0, 
        alt : 0, 
        eph : 0, 
        epv : 65535, 
        vel : 0, 
        cog : 0, 
        satellites_visible : 0
        """
    
    def on_msg_gps_raw(self, m, master):
        pass
        
    def on_msg_raw_imu(self, m, master):
        accv3 = Vector3(m.xacc, m.yacc, m.zacc)
        gyrov3 = Vector3(m.xgyro, m.ygyro, m.zgyro)
        magv3 = Vector3(m.xmag, m.ymag, m.zmag)
        
        #print accv3, gyrov3, magv3 
        
    def on_msg_attitude(self, m,  master):
        self.observer.update_attitude(m.pitch, m.roll, m.yaw) 
        """
        time_boot_ms : 318067, 
        roll : 0.00354599650018, 
        pitch : 0.0179140623659, 
        yaw : 1.61865353584, 
        rollspeed : 8.43061134219e-05, 
        pitchspeed : -0.000376928946935, 
        yawspeed : -0.000310299918056}
        """
        
    def on_msg_vfr_hud(self, m,  msster):
        pass
        """
        airspeed : 0.0, 
        groundspeed : 0.0, 
        heading : 92, 
        throttle : 0, 
        alt : 0.479999989271, 
        climb : 0.199999988079}
        """
        
    def on_msg_ahrs(self, m,  msster):
        pass
        """
        omegaIx : -2.38564134634e-05, 
        omegaIy : 0.00019565311959, 
        omegaIz : -0.00018073352112, 
        accel_weight : 0.0, 
        renorm_val : 0.0, 
        error_rp : 0.000493810279295, 
        error_yaw : 0.00441460032016}
        """
    
    def on_msg_param_value(self, m, master):
        #print "on_msg_param_value", m 
        self.param_mgr.handle_mavlink_packet(m,  master)
        
    def on_msg_global_position_int(self, m, master):
        #self.observer.info("sys_status" + str(m)) 
        #print m
        pass
        """
        lat : 0, lon : 0, alt : 0, relative_alt : 190, vx : 0, vy : 0, vz : 0, hdg 1
        """
        
    def on_msg_mission_current(self, m, master):
        pass
        
    def on_msg_nav_controller_output(self, m, master):
        #self.observer.info("sys_status" + str(m)) 
        pass
        
        """
        nav_roll : 0.0, 
        nav_pitch : 0.0, 
        nav_bearing : 175, 
        target_bearing : 0, 
        wp_dist : 0, 
        alt_error : -0.808967292309, 
        aspd_error : 0.0, 
        xtrack_error : 0.0
        NAV_CONTROLLER_OUTPUT
        """
    
    def on_send_callback(self, m, master):
        '''called on sending a message'''
        
        mtype = m.get_type()
        
        if mtype == 'HEARTBEAT':
            return
            
        #self.observer.info("send: %s" % mtype)
        
    def mav_set_param(self, name, value, retries=3):
        '''set a parameter on a mavlink connection'''
        got_ack = False
        with self.read_lock :                                
                while retries > 0 and not got_ack:
                    retries -= 1
                    self.conn.param_set_send(name.upper(), float(value))
                    tstart = time.time()
                    while time.time() - tstart < 1:
                        self.conn.recv_msg()
                        ack = self.conn.recv_match(type='PARAM_VALUE', blocking=True)
                        if ack == None:
                            time.sleep(0.1)
                            continue
                        if str(name).upper() == str(ack.param_id).upper():
                            got_ack = True
                            break
                if not got_ack:
                    print("timeout setting %s to %f" % (name, float(value)))
                    return False
                return True

    def on_recv_callback(self, m, master):

        mtype = m.get_type()
        
        if mtype == "BAD_DATA":
                self.bad_count += 1
                return
        
        #print mtype
        
        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)
            self.status.counters['MasterIn'] += 1

        #if getattr(m, 'time_boot_ms', None) is not None:
            # update link_delayed attribute
        #    self.handle_msec_timestamp(m, master)
        
        msg_handler_func_name = "on_msg_" + mtype.lower()
        
        msg_handler = getattr(self, msg_handler_func_name,  None)
        if msg_handler :
            msg_handler(m, master) 
        else :
            #print "msg handler not found", msg_handler_func_name
            pass
            
        if mtype not in ["PARAM_VALUE", "STATUSTEXT"]:
            self.status.last_msg[mtype] = m  
            msg = ' '.join(str(m).split()[1:])[1:-1]
            self.observer.show_msg(mtype, msg)
                
    def open(self):
        
        if self.conn :
                return False
                
        self.bad_count = 0
        self.status = MasterStatus()
        self.param_mgr = ParamManager(self.status.mav_param, self.observer)
        
        self.conn = mavutil.mavlink_connection(self.port, autoreconnect=True, baud=self.baud)
        
        self.conn.mav.set_callback(self.on_recv_callback, self.conn)
        self.conn.mav.set_send_callback(self.on_send_callback, self.conn)
        
        '''
        for mtype in ["SYSTEM_TIME", "MEMINFO", "HWSTATUS", "POWER_STATUS", "STATUSTEXT", "SYS_STATUS", "HEARTBEAT"]:
            self.observer.show_msg(mtype, "")
        for mtype in ["SENSOR_OFFSETS", "RAW_IMU", "SCALED_IMU2", "GPS_RAW_INT", "SCALED_PRESSURE"]:
            self.observer.show_msg(mtype, "")
        for mtype in ["SERVO_OUTPUT_RAW", "RC_CHANNELS_RAW"]:
            self.observer.show_msg(mtype, "")
        for mtype in ["GLOBAL_POSITION_INT", "AHRS", "ATTITUDE", "MISSION_CURRENT", "NAV_CONTROLLER_OUTPUT", "VFR_HUD"]:
            self.observer.show_msg(mtype, "")
        '''    
        
        #self.msg_period = mavutil.periodic_event(1.0/15)
        self.heartbeat_period = mavutil.periodic_event(1)
        #self.battery_period = mavutil.periodic_event(0.1)
        #self.heartbeat_check_period = mavutil.periodic_event(0.33)

        # run main loop as a thread
        self.loop_thread = threading.Thread(target = self.loop_run)
        self.loop_thread.daemon = True
        self.running = True
        self.loop_thread.start()
        
        return True
        
    def close(self):
        if not self.conn:
            return
        self.running = False    
        while self.conn :
                time.sleep(0.2)
        
    def loop_run(self):
        while self.running : 
            with self.read_lock :        
                self.conn.recv_msg()
            if self.heartbeat_period.trigger() :
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.param_mgr.fetch_check(self.conn)                                        
        #end while
        self.conn.close()
        self.conn = None 
