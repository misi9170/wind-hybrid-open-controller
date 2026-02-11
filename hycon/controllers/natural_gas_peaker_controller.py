import numpy as np

from hycon.controllers.controller_base import ControllerBase


class NaturalGasPeakerController(ControllerBase):
    """
    Controller for natural gas peaker plants (single cycle gas turbines).

    Controller switches the plant on based on price signals for electricity
    and natural gas.
    """

    def __init__(self, interface, input_dict, controller_parameters={}, verbose=False):
        super().__init__(interface, verbose=verbose)

        self.cname = "gas_peaker"

        # Check that parameters are not specified both in input file
        # and in controller_parameters
        if "controller" in input_dict:
            for cp in controller_parameters.keys():
                if cp in input_dict["controller"]:
                    raise KeyError(
                        'Found key "' + cp + '" in both input_dict["controller"] and'
                        " in controller_parameters."
                    )
            controller_parameters = controller_parameters | input_dict["controller"]
        self.set_controller_parameters(**controller_parameters)

        # Initialize memory
        self.hourly_plan = np.zeros(24, dtype=bool)  # 24-hour plan

    def compute_controls(self, measurements_dict):
        real_time_lmp = measurements_dict["RT_LMP"]
        gas_price = measurements_dict["natural_gas_price"]

        # Calculate the marginal cost of generation for the gas peaker plant
        marginal_cost = (
            self.plant_parameters[self.cname]["heat_rate"] * gas_price
            + self.plant_parameters[self.cname]["variable_operating_cost"]
        )  # in $/MWh

        # Create daily plan based on day-ahead economics, assuming persistence in marginal cost
        if measurements_dict["time"] % (3600 * 24) == 0:  # At the start of a new day
            self.hourly_plan = self.generate_hourly_plan(
                np.array(measurements_dict["DA_LMP_24hours"]),
                marginal_cost
            )

        # Make real-time decision based on current hour's plan
        current_hour = int((measurements_dict["time"] % (3600 * 24)) // 3600)
        if not self.hourly_plan[current_hour]:
            power_setpoint = 0.0
        elif real_time_lmp > marginal_cost:
            power_setpoint = self.plant_parameters[self.cname]["rated_capacity"]
        else:
            power_setpoint = (
                self.plant_parameters[self.cname]["min_stable_load"]
                * self.plant_parameters[self.cname]["rated_capacity"]
            )

        return {"power_setpoint": power_setpoint}
    
    def generate_hourly_plan(self, day_ahead_lmps, marginal_cost):
        """
        Generate an hourly on/off plan for the gas peaker plant based on
        day-ahead LMPs and marginal cost of generation.

        Based on a simple heuristic that turns the plant on when cost thresholds
        are met.

        Inputs:
            day_ahead_lmps: Array of day-ahead LMPs for the next 24 hours
            marginal_cost: Marginal cost of generation for the plant
        Outputs:
            hourly_plan: Array of 1s and 0s indicating on/off status for each hour
        """
        naive_plan = np.where(day_ahead_lmps > marginal_cost, 1, 0)

        # Basic second pass to see if still worth operating considering start-up/shut-down costs
        switching_costs = np.zeros_like(naive_plan)
        naive_plan_switch = np.diff(np.concatenate((self.hourly_plan[-1], naive_plan)))
        switching_costs += (naive_plan_switch > 0) * self.plant_parameters[self.cname][
            "start_up_cost"
        ]
        switching_costs += (naive_plan_switch < 0) * self.plant_parameters[self.cname][
            "shut_down_cost"
        ]

        hourly_plan = np.where(day_ahead_lmps > marginal_cost + switching_costs, True, False)

        return hourly_plan
    

class RealTimeGasController(ControllerBase):
    """
    Controller for natural gas peaker plants (single cycle gas turbines) that
    makes decisions based solely on real-time price signals with no foresight.
    """

    def __init__(self, interface, input_dict, controller_parameters={}, verbose=False):
        super().__init__(interface, verbose=verbose)

        # Check that parameters are not specified both in input file
        # and in controller_parameters
        if "controller" in input_dict:
            for cp in controller_parameters.keys():
                if cp in input_dict["controller"]:
                    raise KeyError(
                        'Found key "' + cp + '" in both input_dict["controller"] and'
                        " in controller_parameters."
                    )
            controller_parameters = controller_parameters | input_dict["controller"]
        self.set_controller_parameters(**controller_parameters)

    def set_controller_parameters(
        self,
        price_threshold=20.0, # $/MWh threshold for turning on the plant
        **_,  # <- Allows arbitrary additional parameters to be passed, which are ignored
    ):
        self.price_threshold = price_threshold

    def compute_controls(self, measurements_dict):
        real_time_lmp = measurements_dict["RT_LMP"]
        #gas_price = measurements_dict["natural_gas_price"]

        # Calculate the marginal cost of generation for the gas peaker plant
        # TODO: Create more sophisticated real-time logic that considers start-up/shut-down costs
        # and minimum up/down times, rather than just a price threshold
        # marginal_cost = (
        #     self.plant_parameters[self.cname]["heat_rate"] * gas_price
        #     + self.plant_parameters[self.cname]["variable_operating_cost"]
        # )  # in $/MWh
        marginal_cost = self.price_threshold

        if real_time_lmp > marginal_cost:
            power_setpoint = self.plant_parameters["thermal"]["rated_capacity"]
        else:
            power_setpoint = 0.0

        return {"power_setpoint": power_setpoint}