import multiprocessing as mp
import os
import pandas as pd

import matplotlib.pyplot as plt
import numpy as np
from rosco.toolbox.control_interface import wfc_zmq_server
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.fast_io import output_processing
from whoc.controllers.lookup_based_wake_steering_controller import LookupBasedWakeSteeringController
from whoc.interfaces.rosco_zmq_interface_2 import ROSCO_ZMQInterface

this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, "examples_out")
os.makedirs(example_out_dir, exist_ok=True)
TIME_CHECK = 20
DESIRED_YAW_OFFSET = [-10, 10]


def run_zmq(interface, logfile=None):
    """Start the ZeroMQ server for wind farm control"""

    # Start the server at the following address
    network_address = "tcp://*:5555"
    server = wfc_zmq_server(network_address, timeout=60.0, verbose=False, logfile = logfile)

    print("Running open-loop controller...")
    #controller = WindFarmPowerDistributingController(interface)

    # Provide the wind farm control algorithm as the wfc_controller method of the server
    server.wfc_controller = interface.rosco_wfc_zmq_server_wfc_controller

    # Run the server to receive measurements and send setpoints
    server.runserver()

    # Do I get here? NO---only after finished. Damn.... so, where does that leave me?
    print("I'm here! run_zmq")

def sim_openfast_1():
    """Run the first OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 25,
    }
    run_dir = os.path.join(example_out_dir, "17b_zeromq_OF1")
    r.controller_params = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"] = {}
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 1
    r.save_dir = run_dir
    r.run_FAST()
    print("I'm here! sim_openfast_1")


def sim_openfast_2():
    """Run the second OpenFAST simulation with ZeroMQ enabled"""
    r = run_FAST_ROSCO()
    r.tuning_yaml = "NREL5MW.yaml"
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts = {
        "U": [8],
        "TMax": 25,
    }
    run_dir = os.path.join(example_out_dir, "17b_zeromq_OF2")
    r.save_dir = run_dir
    r.controller_params = {}
    r.controller_params["DISCON"] = {}
    r.controller_params["LoggingLevel"] = 2
    r.controller_params["DISCON"]["ZMQ_Mode"] = 1
    r.controller_params["DISCON"]["ZMQ_ID"] = 2
    r.run_FAST()
    print("I'm here! sim_openfast_2")

def sim_wfc(interface):
    """Run the wind farm control algorithm"""
    df_opt = pd.read_pickle("yaw_offsets.pkl")
    df_opt.yaw_angles_opt = [[12.0, 6.0] for _ in range(len(df_opt))] # Temp test
    input_dict = {
        "dt":0.5,
        "controller":{
            "num_turbines":2,
            "initial_conditions":{"yaw":270.0}
        }
     }
    controller = LookupBasedWakeSteeringController(interface, input_dict, df_yaw=df_opt)
    
    for t in range(20):
        controller.step()

if __name__ == "__main__":
    
    # Instantiate the interface
    interface = ROSCO_ZMQInterface()

    # Start wind farm control server and two openfast simulation
    # as separate processes
    logfile = os.path.join(example_out_dir,os.path.splitext(os.path.basename(__file__))[0]+'.log')
    p0 = mp.Process(target=run_zmq,args=(interface,logfile))
    p1 = mp.Process(target=sim_openfast_1)
    p2 = mp.Process(target=sim_openfast_2)
    p3 = mp.Process(target=sim_wfc,args=(interface,))

    p0.start()
    p1.start()
    p2.start()
    p3.start()

    p0.join()
    p1.join()
    p2.join()
    p3.join()

    ## Run tests
    # Check that info is passed to ROSCO for first simulation
    op1 = output_processing.output_processing()
    debug_file1 = os.path.join(
        example_out_dir,
        "17b_zeromq_OF1",
        "NREL5MW",
        "power_curve",
        "base",
        "NREL5MW_0.RO.dbg2",
    )
    local_vars1 = op1.load_fast_out(debug_file1, tmin=0)

    # Check that info is passed to ROSCO for first simulation
    op2 = output_processing.output_processing()
    debug_file2 = os.path.join(
        example_out_dir,
        "17b_zeromq_OF2",
        "NREL5MW",
        "power_curve",
        "base",
        "NREL5MW_0.RO.dbg2",
    )
    local_vars2 = op2.load_fast_out(debug_file2, tmin=0)

    # Generate plots
    _, axs = plt.subplots(2, 1)
    axs[0].plot(local_vars1[0]["Time"], local_vars1[0]["ZMQ_YawOffset"])
    axs[1].plot(local_vars2[0]["Time"], local_vars2[0]["ZMQ_YawOffset"])

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir, "17b_NREL5MW_ZMQ_Setpoints.png"))

    # Spot check input at time = 30 sec.
    ind1_30 = local_vars1[0]["Time"] == TIME_CHECK
    ind2_30 = local_vars2[0]["Time"] == TIME_CHECK

    # np.testing.assert_almost_equal(
    #     local_vars1[0]["ZMQ_YawOffset"][ind1_30], DESIRED_YAW_OFFSET[0]
    # )
    # np.testing.assert_almost_equal(
    #     local_vars2[0]["ZMQ_YawOffset"][ind2_30], DESIRED_YAW_OFFSET[1]
    # )
