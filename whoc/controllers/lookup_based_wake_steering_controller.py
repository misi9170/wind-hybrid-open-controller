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

import numpy as np
import pandas as pd

from whoc.controllers.controller_base import ControllerBase
from whoc.design_tools.wake_steering_design import get_yaw_angles_interpolant
from whoc.interfaces.interface import InterfaceBase


class LookupBasedWakeSteeringController(ControllerBase):
    def __init__(
            self,
            interface: InterfaceBase,
            input_dict: dict,
            df_yaw: pd.DataFrame | None = None,
            hysteresis_dict: dict | None = None,
            verbose: bool = False
        ):
        """
        Constructor for LookupBasedWakeSteeringController.

        Args:
            interface (InterfaceBase): Interface object for communicating with the plant.
            input_dict (dict): Dictionary of input parameters.
            df_yaw (pd.DataFrame): DataFrame of yaw offsets. May be produced using tools in 
                whoc.design_tools.wake_steering_design. Defaults to None.
            hysteresis_dict (dict): Dictionary of hysteresis zones. May be produced using
                compute_hysteresis_zones function in whoc.design_tools.wake_steering_design.
                Defaults to None.
            verbose (bool): Verbosity flag.
        """
        super().__init__(interface, verbose=verbose)

        self.dt = input_dict["dt"]  # Won't be needed here, but generally good to have
        self.n_turbines = input_dict["controller"]["num_turbines"]
        self.turbines = range(self.n_turbines)

        # Handle yaw optimizer object
        if df_yaw is None:
            if hysteresis_dict is not None:
                raise ValueError(
                    "Hysteresis zones provided without yaw offsets. "
                    "Please provide yaw offsets."
                )
            if self.verbose:
                print("No offsets received; assuming nominal aligned control.")
            self.wake_steering_interpolant = None
        else:
            self.wake_steering_interpolant = get_yaw_angles_interpolant(df_yaw)

        self.hysteresis_dict = hysteresis_dict

        # Set initial conditions
        yaw_IC = input_dict["controller"]["initial_conditions"]["yaw"]
        if hasattr(yaw_IC, "__len__"):
            if len(yaw_IC) == self.n_turbines:
                self.controls_dict = {"yaw_angles": yaw_IC}
            else:
                raise TypeError(
                    "yaw initial condition should be a float or "
                    + "a list of floats of length num_turbines."
                )
        else:
            self.controls_dict = {"yaw_angles": [yaw_IC] * self.n_turbines}

        # For startup
        self.wd_store = [270.]*self.n_turbines # TODO: update this?


    def compute_controls(self):
        self.wake_steering_angles()

    def wake_steering_angles(self):
        
        # Handle possible bad data
        wind_directions = self.measurements_dict["wind_directions"]
        wind_speeds = [8.0]*self.n_turbines # TODO: enable extraction of wind speed in Hercules
        if not wind_directions: # Received empty or None
            if self.verbose:
                print("Bad wind direction measurement received, reverting to previous measurement.")
            wind_directions = self.wd_store
        else:
            self.wd_store = wind_directions

        # look up wind direction
        if self.wake_steering_interpolant is None:
            yaw_setpoint = wind_directions
        else:
            interpolated_angles = self.wake_steering_interpolant(
                wind_directions,
                wind_speeds,
                None
            )
            yaw_offsets = np.diag(interpolated_angles)
            yaw_setpoint = (np.array(wind_directions) - yaw_offsets).tolist()

        # Apply hysteresis
        if self.hysteresis_dict is not None:
            raise NotImplementedError("Hysteresis not yet implemented.")

        self.controls_dict = {"yaw_angles": yaw_setpoint}

        return None
