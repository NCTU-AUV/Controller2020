#!/usr/bin/python3
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :PID.py
# description     :python pid controller
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7
# ==============================================================================

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P_roll=0.2, I_roll=0.0, D_roll=0.0, P_pitch=0.2, I_pitch=0.0, D_pitch=0.0):

        self.Kp_roll = P_roll
        self.Ki_roll = I_roll
        self.Kd_roll = D_roll

        self.Kp_pitch = P_pitch
        self.Ki_pitch = I_pitch
        self.Kd_pitch = D_pitch

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint_roll = 0.0
        self.SetPoint_pitch = 0.0

        self.PTerm_roll = 0.0
        self.ITerm_roll = 0.0
        self.DTerm_roll = 0.0
        self.PTerm_pitch = 0.0
        self.ITerm_pitch = 0.0
        self.DTerm_pitch = 0.0

        self.last_error_roll = 0.0
        self.last_error_pitch = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output_roll = 0.0
        self.output_pitch = 0.0

    def update_RollandPitch(self, feedback_value_roll, feedback_value_pitch):
        """
        Calculates PID value for given eference feedback

        .. math::
            u(t) = K_p e(t) + K_i / int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error_roll = self.SetPoint_roll - feedback_value_roll
        error_pitch = self.SetPoint_pitch - feedback_value_pitch

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time

        if (delta_time >= self.sample_time):
            self.PTerm_roll = self.Kp_roll * error_roll
            self.PTerm_pitch = self.Kp_pitch * error_pitch

            self.ITerm_roll += error_roll * delta_time
            self.ITerm_pitch += error_pitch * delta_time

            if (self.ITerm_roll < -self.windup_guard):
                self.ITerm_roll = -self.windup_guard
            elif (self.ITerm_roll > self.windup_guard):
                self.ITerm_roll = self.windup_guard
            
            if (self.ITerm_pitch < -self.windup_guard):
                self.ITerm_pitch = -self.windup_guard
            elif (self.ITerm_pitch > self.windup_guard):
                self.ITerm_pitch = self.windup_guard

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error_roll = error_roll
            self.last_error_pitch = error_pitch

            self.output_roll = self.PTerm_roll + (self.Ki_roll * self.ITerm_roll) + (self.Kd_roll * self.DTerm_roll)
            self.output_pitch = self.PTerm_pitch + (self.Ki_pitch * self.ITerm_pitch) + (self.Kd_pitch * self.DTerm_pitch)
            
        return [self.output_roll, self.PTerm_pitch]
        
    def setDTerm_roll(self, Dterm):
        self.DTerm_roll = Dterm
    
    def setDTerm_pitch(self, Dterm):
        self.DTerm_pitch = Dterm

    def setAllCoeff(self, k):
        self.Kp_roll = k[0]
        self.Ki_roll = k[1]
        self.Kd_roll = k[2]
        self.Kp_pitch = k[3]
        self.Ki_pitch = k[4]
        self.Kd_pitch = k[5]

    def setKp_roll(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp_roll = proportional_gain

    def setKi_roll(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki_roll = integral_gain

    def setKd_roll(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd_roll = derivative_gain

    def setKp_pitch(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp_pitch = proportional_gain

    def setKi_pitch(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki_pitch = integral_gain

    def setKd_pitch(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd_pitch = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

    def setSetPoint_roll(self, set_point_roll):
        self.SetPoint_roll = set_point_roll
    
    def setSetPoint_pitch(self, set_point_pitch):
        self.SetPoint_pitch = set_point_pitch