#!/usr/bin/env python3
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from typing import List

class MotorControlCommandNames:
    motor_control_group: str
    enable: str
    m1: str 
    m2: str 
    m3: str
    m4: str
    motors: List[str]

    def __init__(self) -> None:
        self.motor_control_group = "motorPowerSet"
        self.enable = "enable"
        self.m1 = "m1"
        self.m2 = "m2"
        self.m3 = "m3"
        self.m4 = "m4"
        self.motors = [self.m1, self.m2, self.m3, self.m4]


class MotorControl:
    _synced_cf: SyncCrazyflie
    _command_names: MotorControlCommandNames
    _debug_logs: bool

    def __init__(self, synced_cf: SyncCrazyflie) -> None:
        self._synced_cf = synced_cf
        self._command_names = MotorControlCommandNames()
        self._debug_logs = True
        self._setup_parameters()
       
    def _motor_enable_callback(self, name, value):
        if self._debug_logs:
            print(name, " is set to ", value)
    
    def _m_callback(self, name, value):
        if self._debug_logs:
            print(name, " is set to ", value) 


    def _setup_parameters(self):
        group = self._command_names.motor_control_group

        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=self._command_names.enable,
                        cb=self._motor_enable_callback) 

        for motor in self._command_names.motors:
             self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=motor,
                        cb=self._m_callback) 


    def set_custom_callback(self, callback):
        group = self._command_names.motor_control_group

        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=self._command_names.enable,
                        cb=callback) 

        for motor in self._command_names.motors:
             self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=motor,
                        cb=callback) 
    
    def set_custom_callback_enable_motor_level_control(self, callback):
        group = self._command_names.motor_control_group

        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=self._command_names.enable,
                        cb=callback) 

    def set_custom_callback_set_motors(self, motor_idx: int, callback):
        """
        motor_idx \in {0,1,2,3} corresponding to m1, m2, m3, m4
        """
        group = self._command_names.motor_control_group
        name = self._command_names.motors[motor_idx]
        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=name,
                        cb=callback) 

    def enable_motor_control(self):
        full_name = self._command_names.motor_control_group + "." + self._command_names.enable
        self._synced_cf.cf.param.set_value(full_name, 1)

    def disable_motor_control(self):
        full_name = self._command_names.motor_control_group + "." + self._command_names.enable
        self._synced_cf.cf.param.set_value(full_name, 0)

    def set_motors(self, velocities: List[int]):
        assert len(velocities) == 4
        for i in range(4):
            full_name = self._command_names.motor_control_group + "." + self._command_names.motors[i]
            self._synced_cf.cf.param.set_value(full_name, velocities[i])


        