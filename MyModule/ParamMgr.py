#coding utf-8

import time, os, fnmatch

from pymavlink import mavutil, mavparm

class ParamManager:
    '''this class is separated to make it possible to use the parameter
       functions on a secondary connection'''
    def __init__(self, mav_param, observer):
        self.mav_param_set = set()
        self.mav_param_count = 0
        self.param_period = mavutil.periodic_event(1)
        self.fetch_one = 0
        self.mav_param = mav_param
        self.observer = observer
        
    def handle_mavlink_packet(self, m, master):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'PARAM_VALUE':
            #print m    
            param_id = "%.16s" % m.param_id
            if m.param_index != -1 and m.param_index not in self.mav_param_set:
                added_new_parameter = True
                self.mav_param_set.add(m.param_index)
            else:
                added_new_parameter = False
            if m.param_count != -1:
                self.mav_param_count = m.param_count
            if m.param_type <= 8:
                self.mav_param[str(param_id)] = int(m.param_value)
            else :
                self.mav_param[str(param_id)] = m.param_value
            
            self.observer.show_param(m.param_count, m.param_index,  m.param_id,  int(m.param_type),  str(self.mav_param[str(param_id)]) )
            
            if self.fetch_one > 0:
                self.fetch_one -= 1
                #print("%s = %f" % (param_id, m.param_value))
            if added_new_parameter and len(self.mav_param_set) == m.param_count:
                self.observer.params_done()
               
    def fetch_check(self, master):
        '''check for missing parameters periodically'''
        if self.param_period.trigger():
            if len(self.mav_param_set) == 0:
                master.param_fetch_all()
            elif self.mav_param_count != 0 and len(self.mav_param_set) != self.mav_param_count:
                if master.time_since('PARAM_VALUE') >= 1:
                    diff = set(range(self.mav_param_count)).difference(self.mav_param_set)
                    count = 0
                    while len(diff) > 0 and count < 10:
                        idx = diff.pop()
                        master.param_fetch_one(idx)
                        count += 1
