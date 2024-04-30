# Copyright 2021 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# See https://nrel.github.io/wind-hybrid-open-controller for documentation

import zmq

from whoc.controllers.wind_farm_power_tracking_controller import POWER_SETPOINT_DEFAULT
from whoc.interfaces.interface_base import InterfaceBase

# Code copied from ROSCO; consider just importing and using that code
# directly??


class ROSCO_ZMQInterface(InterfaceBase):
    def __init__(
        self, network_address="tcp://*:5555", identifier="0", timeout=600.0, verbose=False
    ):
        """Python implementation of the ZeroMQ server side for the ROSCO
        ZeroMQ wind farm control interface. This class makes it easy for
        users to receive measurements from ROSCO and then send back control
        setpoints (generator torque, nacelle heading and/or blade pitch
        angles).
        Args:
            network_address (str, optional): The network address to
                communicate over with the desired instance of ROSCO. Note that,
                if running a wind farm simulation in SOWFA or FAST.Farm, there
                are multiple instances of ROSCO and each of these instances
                needs to communicate over a unique port. Also, for each of those
                instances, you will need an instance of zmq_server. Defaults to
                "tcp://*:5555".
            identifier (str, optional): Turbine identifier. Defaults to "0".
            timeout (float, optional): Seconds to wait for a message from
                the ZeroMQ server before timing out. Defaults to 600.0.
            verbose (bool, optional): Print to console. Defaults to False.
        """
        super().__init__()

        self.network_address = network_address
        self.identifier = identifier
        self.timeout = timeout
        self.verbose = verbose
        
        # Set up a dictionary to hold the rosco measurements
        self.n_turbines = 2 # TODO: Where can I get this information easily?
        self.rosco_measurements = {}

        # Establish the dictionaries to hold the measurements and setpoints
        yaw_angles_initial = {
            "yaw_angle_{0:03d}".format(id):0.0 for id in range(1, self.n_turbines+1)
        }
        vane_angles_initial = {
            "vane_{0:03d}".format(id):0.0 for id in range(1, self.n_turbines+1)
        }
        powers_initial = {
            "power_{0:03d}".format(id):0.0 for id in range(1, self.n_turbines+1)
        }
        self.rosco_measurements = {
            **yaw_angles_initial,
            **vane_angles_initial,
            **powers_initial
        }
        self.rosco_setpoints = {
            "yaw_setpoint_{0:03d}".format(id):0.0 for id in range(1, self.n_turbines+1)
        }
        self.rosco_time = 0.0 # Initialize?

    def get_measurements(self, hercules_dict=None):
        """
        Reads the inputs from the place where the wfc_controller method sets them
        """
        #print("ROSCO time in get_measurements():", self.rosco_time)

        # Handle external signals
        if (hercules_dict is not None
            and "external_signals" in hercules_dict
            and "wind_power_reference" in hercules_dict["external_signals"]):
            wind_power_reference = hercules_dict["external_signals"]["wind_power_reference"]
        else:
            wind_power_reference = POWER_SETPOINT_DEFAULT

        # Process individual measurements into lists
        wind_directions = [
            self.rosco_measurements["yaw_angle_{0:03d}".format(id)] 
            + self.rosco_measurements["vane_{0:03d}".format(id)]
            for id in range(1, self.n_turbines+1)
        ]
        turbine_powers = [
            self.rosco_measurements["power_{0:03d}".format(id)] for id in range(1, self.n_turbines+1)
        ]

        # Combine into dict
        measurements = {
            "time": self.rosco_time,
            "wind_directions": wind_directions,
            "turbine_powers": turbine_powers,
            "wind_power_reference": wind_power_reference
        }

        return measurements


    def rosco_wfc_zmq_server_wfc_controller(self, id, current_time, measurements):

        # Save new measurements
        #        self.rosco_measurements_all[ = measurements
        self.rosco_time = current_time
                                    
        # Check id and measurements[0] match
        if id != measurements["ZMQ_ID"]:
            raise ValueError("Inconsistent turbine ID.")

        self.rosco_measurements["status_{0:03d}".format(id)] = measurements["iStatus"]
        self.rosco_measurements["time_{0:03d}".format(id)] = measurements["Time"]
        self.rosco_measurements["mech_power_{0:03d}".format(id)] = measurements["VS_MechGenPwr"]
        self.rosco_measurements["power_{0:03d}".format(id)] = measurements["VS_GenPwr"]
        self.rosco_measurements["gen_speed_{0:03d}".format(id)] = measurements["GenSpeed"]
        self.rosco_measurements["rot_speed_{0:03d}".format(id)] = measurements["RotSpeed"]
        self.rosco_measurements["gen_torque_{0:03d}".format(id)] = measurements["GenTqMeas"]
        # TODO: check whether NacelleHeading is absolute or relative (presumably absolute?)
        self.rosco_measurements["yaw_angle_{0:03d}".format(id)] = measurements["NacHeading"]
        self.rosco_measurements["vane_{0:03d}".format(id)] = measurements["NacVane"]
        self.rosco_measurements["wind_speed_{0:03d}".format(id)] = measurements["HorWindV"]
        self.rosco_measurements["root_MOOP1_{0:03d}".format(id)] = measurements["rootMOOP(1)"]
        self.rosco_measurements["root_MOOP2_{0:03d}".format(id)] = measurements["rootMOOP(2)"]
        self.rosco_measurements["root_MOOP3_{0:03d}".format(id)] = measurements["rootMOOP(3)"]
        self.rosco_measurements["FA_accel_{0:03d}".format(id)] = measurements["FA_Acc"]
        self.rosco_measurements["nac_imu_FA_accel_{0:03d}".format(id)] = measurements["NacIMU_FA_Acc"]
        self.rosco_measurements["azimuth_angle_{0:03d}".format(id)] = measurements["Azimuth"]

        # Extract control setpoints
        setpoints = {}
        setpoints["ZMQ_YawOffset"] = self.rosco_setpoints["yaw_setpoint_{0:03d}".format(id)]
        setpoints["ZMQ_PitOffset(1)"] = 0 # Not currently controlled
        setpoints["ZMQ_PitOffset(2)"] = 0 # Not currently controlled
        setpoints["ZMQ_PitOffset(3)"] = 0 # Not currently controlled
        #setpoints["ZMQ_PowerReference"] = 0 # Not currently implemented

        #print("wfc_controller:", self.rosco_setpoints)

        return setpoints

    def check_controls(self, controls_dict):
        available_controls = [
            "yaw_angles",
            "power_setpoints",
        ]

        for k in controls_dict.keys():
            if k not in available_controls:
                raise ValueError("Setpoint " + k + " is not available in this configuration")

    def send_controls(self, _, yaw_angles=None, power_setpoints=None):
        """
        Send controls to ROSCO .dll ffor individual turbine control

        Parameters:
        -----------
        genTorques: float
            Generator torque setpoint
        nacelleHeadings: float
            Nacelle heading setpoint
        bladePitchAngles: List (len=3)
            Blade pitch angle setpoint
        """
        # Create a message with controls to send to ROSCO

        # Handle None inputs
        if yaw_angles is None:
            yaw_angles = [0] * self.n_turbines
        if power_setpoints is None:
            power_setpoints = [POWER_SETPOINT_DEFAULT] * self.n_turbines

        # Set up the controls dict as per turbine ID
        for id in range(1, self.n_turbines+1):
            self.rosco_setpoints["yaw_setpoint_{0:03d}".format(id)] = yaw_angles[id-1]
            self.rosco_setpoints["power_setpoint_{0:03d}".format(id)] = power_setpoints[id-1]
                
        return None
