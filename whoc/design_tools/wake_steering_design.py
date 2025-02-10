import numpy as np
import pandas as pd
from floris import FlorisModel, UncertainFlorisModel, WindRose
from floris.optimization.yaw_optimization.yaw_optimizer_sr import YawOptimizationSR
from scipy.interpolate import interp1d, RegularGridInterpolator


def build_simple_wake_steering_lookup_table(
    fmodel: FlorisModel,
    wd_resolution: float = 5,
    wd_min: float = 0,
    wd_max: float = 360,
    ws_resolution: float = 1,
    ws_min: float = 8,
    ws_max: float = 8,
    ti: float = 0.06,
    minimum_yaw_angle: float = 0.0,
    maximum_yaw_angle: float = 25.0,
) -> pd.DataFrame:
    """
    Build a simple wake steering lookup table for a given FlorisModel using the Serial Refine
    method.

    This is mainly intended for demonstration purposes. Users that want more control over the
    optimization process should use FLORIS's YawOptimizationSR procedure directly to produce
    the output df_opt.

    Args:
        fmodel (FlorisModel): An instantiated FlorisModel object.
        wd_resolution (float, optional): The resolution of the wind direction in degrees.
            Defaults to 5.
        wd_min (float, optional): The minimum wind direction in degrees. Defaults to 0.
        wd_max (float, optional): The maximum wind direction in degrees. Defaults to 360.
        ws_resolution (float, optional): The resolution of the wind speed in m/s.
            Defaults to 1.
        ws_min (float, optional): The minimum wind speed in m/s. Defaults to 8.
        ws_max (float, optional): The maximum wind speed in m/s. Defaults to 8.
        ti (float, optional): The turbulence intensity for wake steering design. Defaults to 0.06.
        minimum_yaw_angle (float, optional): The minimum allowable misalignment in degrees.
            Defaults to 0.0.
        maximum_yaw_angle (float, optional): The maximum allowable misalignment in degrees.
            Defaults to 25.0.

    Returns:
        pd.DataFrame: A yaw offset lookup table.
    """
    wind_rose = create_uniform_wind_rose(
        wd_resolution=wd_resolution,
        wd_min=wd_min,
        wd_max=wd_max,
        ws_resolution=ws_resolution,
        ws_min=ws_min,
        ws_max=ws_max,
        ti=ti,
    )

    fmodel.set(wind_data=wind_rose)

    yaw_opt = YawOptimizationSR(
        fmodel=fmodel,
        minimum_yaw_angle=minimum_yaw_angle,
        maximum_yaw_angle=maximum_yaw_angle,
    )

    return yaw_opt.optimize()


def build_uncertain_wake_steering_lookup_table(
    fmodel: FlorisModel,
    wd_std: float,
    wd_resolution: float = 5,
    wd_min: float = 0,
    wd_max: float = 360,
    ws_resolution: float = 1,
    ws_min: float = 8,
    ws_max: float = 8,
    ti: float = 0.06,
    minimum_yaw_angle: float = 0.0,
    maximum_yaw_angle: float = 25.0,
    kwargs_UncertainFlorisModel: dict = {},
) -> pd.DataFrame:
    """
    Build a simple wake steering lookup table for a given FlorisModel using the Serial Refine
    method, with uncertainty in the wind direction.

    This is mainly intended for demonstration purposes. Users that want more control over the
    optimization process should use FLORIS's YawOptimizationSR procedure directly to produce
    the output df_opt.

    Args:
        fmodel (FlorisModel): An instantiated FlorisModel object.
        wd_std (float): Wind direction standard deviation in degrees.
        wd_resolution (float, optional): The resolution of the wind direction in degrees.
            Defaults to 5.
        wd_min (float, optional): The minimum wind direction in degrees. Defaults to 0.
        wd_max (float, optional): The maximum wind direction in degrees. Defaults to 360.
        ws_resolution (float, optional): The resolution of the wind speed in m/s.
            Defaults to 1.
        ws_min (float, optional): The minimum wind speed in m/s. Defaults to 8.
        ws_max (float, optional): The maximum wind speed in m/s. Defaults to 8.
        ti (float, optional): The turbulence intensity for wake steering design. Defaults to 0.06.
        minimum_yaw_angle (float, optional): The minimum allowable misalignment in degrees.
            Defaults to 0.0.
        maximum_yaw_angle (float, optional): The maximum allowable misalignment in degrees.
            Defaults to 25.0.
        kwargs_UncertainFlorisModel (dict, optional): Additional keyword arguments for the
            instantiation of the UncertainFlorisModel. Defaults to an empty dictionary.

    Returns:
        pd.DataFrame: A yaw offset lookup table.
    """
    wind_rose = create_uniform_wind_rose(
        wd_resolution=wd_resolution,
        wd_min=wd_min,
        wd_max=wd_max,
        ws_resolution=ws_resolution,
        ws_min=ws_min,
        ws_max=ws_max,
        ti=ti,
    )

    fmodel.set(wind_data=wind_rose)

    ufmodel = UncertainFlorisModel(
        configuration=fmodel.core.as_dict(),
        wd_std=wd_std,
        **kwargs_UncertainFlorisModel,
    )

    yaw_opt = YawOptimizationSR(
        fmodel=ufmodel,
        minimum_yaw_angle=minimum_yaw_angle,
        maximum_yaw_angle=maximum_yaw_angle,
    )

    return yaw_opt.optimize()


def apply_static_rate_limits(
    df_opt: pd.DataFrame,
    wd_rate_limit: float = 5,
    ws_rate_limit: float = 10,
    ti_rate_limit: float = 1
) -> pd.DataFrame:
    """
    Apply static rate limits to a yaw offset lookup table.

    Args:
        df_opt (pd.DataFrame): A yaw offset lookup table.
        wd_rate_limit (float, optional): The maximum rate of change in yaw offset per degree change
            in wind direction [deg / deg]. Defaults to 5.
        ws_rate_limit (float, optional): The maximum rate of change in yaw offset per change in
            wind speed [deg / m/s]. Defaults to 10.
        ti_rate_limit (float, optional): The maximum rate of change in yaw offset per change in
            turbulence intensity [deg / -]. Defaults to 1.
    
    Returns:
        pd.DataFrame: A yaw offset lookup table with rate limits applied.
    """

    offsets_all = np.vstack(df_opt.yaw_angles_opt.to_numpy())

    wd_array = np.unique(df_opt.wind_direction)
    ws_array = np.unique(df_opt.wind_speed)
    ti_array = np.unique(df_opt.turbulence_intensity)

    wd_step = wd_array[1] - wd_array[0]
    ws_step = ws_array[1] - ws_array[0]
    ti_step = ti_array[1] - ti_array[0] if len(ti_array) > 1 else 1

    check_df_opt_ordering(df_opt)
    # 4D array, with dimensions: (wd, ws, ti, turbines)
    offsets_array = offsets_all.reshape(
        (len(wd_array), len(ws_array), len(ti_array), offsets_all.shape[-1])
    )

    # Apply wd rate limits
    offsets_limited_lr = offsets_array.copy()
    for i in range(1, len(wd_array)):
        delta_yaw = offsets_limited_lr[i, :, :, :] - offsets_limited_lr[i-1, :, :, :]
        delta_yaw = np.clip(delta_yaw, -wd_rate_limit*wd_step, wd_rate_limit*wd_step)
        offsets_limited_lr[i, :, :, :] = offsets_limited_lr[i-1, :, :, :] + delta_yaw
    offsets_limited_rl = offsets_array.copy()
    for i in range(len(wd_array)-2, -1, -1):
        delta_yaw = offsets_limited_rl[i, :, :, :] - offsets_limited_rl[i+1, :, :, :]
        delta_yaw = np.clip(delta_yaw, -wd_rate_limit*wd_step, wd_rate_limit*wd_step)
        offsets_limited_rl[i, :, :, :] = offsets_limited_rl[i+1, :, :, :] + delta_yaw
    offsets_array = (offsets_limited_lr + offsets_limited_rl) / 2

    # Apply ws rate limits
    offsets_limited_lr = offsets_array.copy()
    for j in range(1, len(ws_array)):
        delta_yaw = offsets_limited_lr[:, j, :, :] - offsets_limited_lr[:, j-1, :, :]
        delta_yaw = np.clip(delta_yaw, -ws_rate_limit*ws_step, ws_rate_limit*ws_step)
        offsets_limited_lr[:, j, :, :] = offsets_limited_lr[:, j-1, :, :] + delta_yaw
    offsets_limited_rl = offsets_array.copy()
    for j in range(len(ws_array)-2, -1, -1):
        delta_yaw = offsets_limited_rl[:, j, :, :] - offsets_limited_rl[:, j+1, :, :]
        delta_yaw = np.clip(delta_yaw, -ws_rate_limit*ws_step, ws_rate_limit*ws_step)
        offsets_limited_rl[:, j, :, :] = offsets_limited_rl[:, j+1, :, :] + delta_yaw
    offsets_array = (offsets_limited_lr + offsets_limited_rl) / 2

    # Apply ti rate limits
    offsets_limited_lr = offsets_array.copy()
    for k in range(1, len(ti_array)):
        delta_yaw = offsets_limited_lr[:, :, k, :] - offsets_limited_lr[:, :, k-1, :]
        delta_yaw = np.clip(delta_yaw, -ti_rate_limit*ti_step, ti_rate_limit*ti_step)
        offsets_limited_lr[:, :, k, :] = offsets_limited_lr[:, :, k-1, :] + delta_yaw
    offsets_limited_rl = offsets_array.copy()
    for k in range(len(ti_array)-2, -1, -1):
        delta_yaw = offsets_limited_rl[:, :, k, :] - offsets_limited_rl[:, :, k+1, :]
        delta_yaw = np.clip(delta_yaw, -ti_rate_limit*ti_step, ti_rate_limit*ti_step)
        offsets_limited_rl[:, :, k, :] = offsets_limited_rl[:, :, k+1, :] + delta_yaw
    offsets_array = (offsets_limited_lr + offsets_limited_rl) / 2

    # Flatten array back into 2D array for dataframe
    offsets_all_limited = offsets_array.reshape(
        (len(wd_array)*len(ws_array)*len(ti_array), offsets_all.shape[-1])
    )
    df_opt_rate_limited = df_opt.copy()
    df_opt_rate_limited["yaw_angles_opt"] = [*offsets_all_limited]

    return df_opt_rate_limited


def compute_hysteresis_zones(
    df_opt: pd.DataFrame,
    min_region_width: float = 2.0,
    yaw_rate_threshold: float = 10.0,
    verbose: bool = False,
) -> dict[str: list[tuple[float, float]]]:
    """
    Compute wind direction sectors where hysteresis is applied.

    Identifies wind direction sectors over which hysteresis is applied when
    there is a switch in sign in the yaw offset. Note that this is only applied
    for wind direction, that is, no hysteresis is applied for wind speed or
    turbulence intensity changes.

    Args:
        df_opt (pd.DataFrame): A yaw offset lookup table.
        min_region_width (float, optional): The minimum width of a hysteresis
            region in degrees. Defaults to 2.0.
        yaw_rate_threshold (float, optional): The threshold for identifying a
            hysteresis region in degrees per degree change in wind direction.
            Defaults to 10.0.
        verbose (bool, optional): Whether to print verbose output. Defaults to
            False.

    Returns:
        hysteresis_dict: A dictionary of hysteresis regions. Keys are turbine
           labels, following "T000", T001", etc. Values are lists of two-tuples
            representing the lower and upper wind direction bounds of the
            hysteresis region.
    """

    # Extract yaw offsets, wind directions
    check_df_opt_ordering(df_opt)
    offsets_stacked = np.vstack(df_opt.yaw_angles_opt.to_numpy())
    wind_directions = np.unique(df_opt.wind_direction)
    offsets = offsets_stacked.reshape(
        len(wind_directions),
        len(np.unique(df_opt.wind_speed)),
        len(np.unique(df_opt.turbulence_intensity)), 
        offsets_stacked.shape[1]
    )

    # Add 360 to end, if starting at/near 0
    if len(wind_directions) == 1:
        raise ValueError("Cannot compute hysteresis regions for single wind direction.")
    wd_step = (wind_directions[1]-wind_directions[0])
    if (wind_directions[0] - wd_step < 0) & (wind_directions[-1] + wd_step >= 360):
        offsets = np.concatenate((offsets, offsets[0:1, :, :, :]), axis=0)
        wd_centers = wind_directions + 0.5 * wd_step
        wind_directions = np.concatenate((wind_directions, [wind_directions[-1] + wd_step]))
    else:
        wd_centers = wind_directions[:-1] + 0.5 * wd_step
    
    # Define function that identifies hysteresis zones
    switching_idx = np.argwhere(np.diff(offsets, axis=0) >= yaw_rate_threshold*wd_step)
    # Drop information about ws, ti
    switching_idx = np.unique(switching_idx[:, [0, 3]], axis=0)
    # Convert to a per-turbine dictionary of switching wind directions
    hysteresis_dict = {}
    for t in np.unique(switching_idx[:,1]):
        hysteresis_dict["T{:03d}".format(t)] = (
            wd_centers[switching_idx[switching_idx[:,1] == t][:,0]]
        )
    if verbose:
        print("Center wind directions for hysteresis, per turbine: {}".format(hysteresis_dict))
        print("Computing hysteresis regions.")

    # Find hysteresis regions for each switching point
    # Note: doesn't handle the (unlikely) case that there is a large jump without a sign change
    for turbine_tag in hysteresis_dict.keys():
        hysteresis_wds = []
        for wd_switch_point in hysteresis_dict[turbine_tag]:
            t = int(turbine_tag[1:])
            # Starting point for hysteresis region
            lb = np.max(wind_directions[
                (wind_directions < wd_switch_point) & (offsets[:,:,:,t] < 0.1).any(axis=(1,2))
            ])
            # Ending point for hysteresis region
            ub = np.min(wind_directions[
                (wind_directions > wd_switch_point) & (offsets[:,:,:,t] > -0.1).any(axis=(1,2))
            ])
            # Check wide enough to satisfy min_region_width; if not, widen
            if (ub - lb) < min_region_width:
                center_point = (lb + ub)/2
                lb = center_point - min_region_width/2
                ub = center_point + min_region_width/2
            hysteresis_wds.append((lb, ub))
        hysteresis_dict[turbine_tag] = hysteresis_wds

    if verbose:
        print("Identified hysteresis regions: {}".format(hysteresis_dict))

    return hysteresis_dict


def apply_wind_speed_ramps(
    df_opt: pd.DataFrame,
    ws_resolution: float = 1,
    ws_min: float = 0,
    ws_max: float = 30,
    ws_wake_steering_cut_in: float = 3,
    ws_wake_steering_fully_engaged_low: float = 5,
    ws_wake_steering_fully_engaged_high: float = 10,
    ws_wake_steering_cut_out: float = 13,
) -> pd.DataFrame:
    """
    Apply wind speed ramps to a yaw offset lookup table.

    Args:
        df_opt (pd.DataFrame): A yaw offset lookup table.
        ws_resolution (float, optional): The resolution of the wind speed in m/s.
            Defaults to 1.
        ws_min (float, optional): The minimum wind speed in m/s. Defaults to 0.
        ws_max (float, optional): The maximum wind speed in m/s. Defaults to 30.
        ws_wake_steering_cut_in (float, optional): The wind speed at which wake steering
            begins to be applied. Defaults to 3.
        ws_wake_steering_fully_engaged_low (float, optional): The lower wind speed at which
            wake steering is fully engaged at the value provided in df_opt. Defaults to 5.
        ws_wake_steering_fully_engaged_high (float, optional): The upper wind speed at which
            wake steering is fully engaged at the value provided in df_opt. Defaults to 10.
        ws_wake_steering_cut_out (float, optional): The wind speed at which wake steering
            ceases to be applied. Defaults to 13.

    Returns:
        pd.DataFrame: A yaw offset lookup table for all wind speeds between ws_min and ws_max
            with wind speed ramps applied.
    """
    check_df_opt_ordering(df_opt)

    # Check valid ordering of wind speeds
    if (ws_wake_steering_cut_in
        <= ws_wake_steering_fully_engaged_low
        <= ws_wake_steering_fully_engaged_high
        <= ws_wake_steering_cut_out):
        pass
    else:
        raise ValueError(
            "Wind speed ramp values must be in the order: cut in, fully engaged low, "
            "fully engaged high, cut out."
        )

    # Check if there is more than one wind speed specified
    if len(df_opt["wind_speed"].unique()) > 1:
        raise ValueError(
            "Wind speed ramps can only be applied to a dataframe with a single wind speed."
        )
    else:
        ws_specified = df_opt["wind_speed"].unique()

    # Check that provided wind speed is between the fully engaged limits
    if (ws_specified < ws_wake_steering_fully_engaged_low
        or ws_specified > ws_wake_steering_fully_engaged_high):
        raise ValueError(
            "Provided wind speed must be between fully engaged limits."
        )

    offsets_specified = np.vstack(df_opt.yaw_angles_opt.to_numpy())[None,:,:]

    # Pack offsets with zero values at the cut in, start, finish, and cut out wind speeds
    offsets_ramps = np.concatenate(
        (
            np.zeros_like(offsets_specified),
            np.zeros_like(offsets_specified),
            offsets_specified,
            offsets_specified,
            np.zeros_like(offsets_specified),
            np.zeros_like(offsets_specified)
        ),
        axis=0
    )
    wind_speed_ramps = np.array([
        ws_min,
        ws_wake_steering_cut_in,
        ws_wake_steering_fully_engaged_low,
        ws_wake_steering_fully_engaged_high,
        ws_wake_steering_cut_out,
        ws_max
    ])

    # Build interpolator and interpolate to desired wind speeds
    interp = interp1d(
        wind_speed_ramps,
        offsets_ramps,
        axis=0,
        bounds_error=False,
        fill_value=np.zeros_like(offsets_ramps[0,:,:])
    )
    wind_speed_all = np.arange(ws_min, ws_max, ws_resolution)
    offsets_stacked = interp(wind_speed_all).reshape(-1, offsets_ramps.shape[2])

    wind_direction_stacked = np.tile(df_opt.wind_direction, len(wind_speed_all))
    wind_speed_stacked = np.repeat(wind_speed_all, len(df_opt))
    turbulence_intensity_stacked = np.tile(df_opt.turbulence_intensity, len(wind_speed_all))

    return pd.DataFrame({
        "wind_direction": wind_direction_stacked,
        "wind_speed": wind_speed_stacked,
        "turbulence_intensity": turbulence_intensity_stacked,
        "yaw_angles_opt": [offsets_stacked[i,:] for i in range(offsets_stacked.shape[0])]
    })


def get_yaw_angles_interpolant(df_opt):
    """Get an interpolant for the optimal yaw angles from a dataframe.

    Create an interpolant for the optimal yaw angles from a dataframe
    'df_opt', which contains the rows 'wind_direction', 'wind_speed',
    'turbulence_intensity', and 'yaw_angles_opt'. This dataframe is typically
    produced automatically from a FLORIS yaw optimization using Serial Refine
    or SciPy. One can additionally apply a ramp-up and ramp-down region
    to transition between non-wake-steering and wake-steering operation.

    Note that, in contrast to the previous implementation in FLASC, this implementation
    does not allow ramp_up_ws, ramp_down_ws, minimum_yaw_angle, or maximum_yaw_angle.
    Minimum and maximum yaw angles should be specified during the design optimization,
    while ramping with wind speed is now handled by apply_wind_speed_ramps.

    Wind speeds and turbulence intensities are extended to include all reasonable values
    by copying the first and last values. Wind directions are extended to handle wind directions
    up to 360 degrees only if the first value is 0 degrees. 

    An error is raised if the resulting interpolant is queried outside of the extended
    wind direction, wind speed, or turbulence intensity ranges.

    Args:
        df_opt (pd.DataFrame): Dataframe containing the rows 'wind_direction',
            'wind_speed', 'turbulence_intensity', and 'yaw_angles_opt'.

    Returns:
        RegularGridInterpolator: An interpolant function which takes the inputs
            (wind_directions, wind_speeds, turbulence_intensities), all of equal
            dimensions, and returns the yaw angles for all turbines. This function
            incorporates the ramp-up and ramp-down regions.
    """
    check_df_opt_ordering(df_opt)

    # Extract points and values
    wind_directions = np.unique(df_opt["wind_direction"])
    wind_speeds = np.unique(df_opt["wind_speed"])
    turbulence_intensities = np.unique(df_opt["turbulence_intensity"])
    yaw_offsets = np.vstack(df_opt["yaw_angles_opt"])

    # Store for possible use if no turbulence intensity is provided
    ti_ref = float(np.median(turbulence_intensities))

    # Reshape the yaw offsets to match the wind direction, wind speed, and turbulence intensity
    yaw_offsets = yaw_offsets.reshape(
        len(wind_directions),
        len(wind_speeds),
        len(turbulence_intensities),
        yaw_offsets.shape[1],
    )

    # Expand wind direction range to cover 0 deg to 360 deg
    if wind_directions[0] == 0.0:
        wind_directions = np.concatenate([wind_directions, [360.0]])
        yaw_offsets = np.concatenate([yaw_offsets, yaw_offsets[0:1, :, :, :]], axis=0)
    else:
        print(
            "0 degree wind direction not found in data. "
            "Wind directions will not be wrapped around 0/360 degree point."
        )

    # Create lower and upper wind speed and turbulence intensity bounds
    wind_speeds = np.concatenate([[-1.0], wind_speeds, [999.0]])
    yaw_offsets = np.concatenate(
        [yaw_offsets[:, 0:1, :, :], yaw_offsets, yaw_offsets[:, -1:, :, :]],
        axis=1
    )
    turbulence_intensities = np.concatenate([[-1.0], turbulence_intensities, [999.0]])
    yaw_offsets = np.concatenate(
        [yaw_offsets[:, :, 0:1, :], yaw_offsets, yaw_offsets[:, :, -1:, :]],
        axis=2
    )

    # Linear interpolant for the yaw angles
    interpolant = RegularGridInterpolator(
        points=(wind_directions, wind_speeds, turbulence_intensities),
        values=yaw_offsets,
        bounds_error=True
    )

    # Store for bounds checks 
    wd_min = wind_directions.min()
    wd_max = wind_directions.max()
    ws_min = wind_speeds.min()
    ws_max = wind_speeds.max()
    ti_min = turbulence_intensities.min()
    ti_max = turbulence_intensities.max()

    # Create a wrapper function to return
    def yaw_angle_interpolant(wd_array, ws_array, ti_array=None):
        # Deal with missing ti_array
        if ti_array is None:
            ti_array = np.ones(np.shape(wd_array), dtype=float) * ti_ref

        # Format inputs
        wd_array = np.array(wd_array, dtype=float)
        ws_array = np.array(ws_array, dtype=float)
        ti_array = np.array(ti_array, dtype=float)

        # Check inputs are within bounds
        if (np.any(wd_array < wd_min) or np.any(wd_array > wd_max)
            or np.any(ws_array < ws_min) or np.any(ws_array > ws_max)
            or np.any(ti_array < ti_min) or np.any(ti_array > ti_max)):
            err_msg = (
                "Interpolator queried outside of allowable bounds:\n"
                "Wind direction bounds: ["+str(wd_min)+", "+str(wd_max)+"]\n"
                "Wind speed bounds: ["+str(ws_min)+", "+str(ws_max)+"]\n"
                "Turbulence intensity bounds: ["+str(ti_min)+", "+str(ti_max)+"]\n\n"
                "Queried at:\n"
                "Wind directions: "+str(wd_array)+" \n"
                "Wind speeds: "+str(ws_array)+" \n"
                "Turbulence intensities: "+str(ti_array)
            )
            raise ValueError(err_msg)

        interpolation_points = np.column_stack((wd_array, ws_array, ti_array))
        return np.array(interpolant(interpolation_points), dtype=float)

    return yaw_angle_interpolant


def create_uniform_wind_rose(
    wd_resolution: float = 5,
    wd_min: float = 0,
    wd_max: float = 360,
    ws_resolution: float = 1,
    ws_min: float = 8,
    ws_max: float = 8,
    ti: float = 0.06,
):

    if wd_min == 0 and wd_max == 360:
        wd_max = wd_max - wd_resolution
    wind_directions = np.arange(wd_min, wd_max+0.001, wd_resolution)
    
    wind_speeds = np.arange(ws_min, ws_max+0.001, ws_resolution)

    return WindRose(
        wind_speeds=wind_speeds,
        wind_directions=wind_directions,
        ti_table=ti,
    )

def check_df_opt_ordering(df_opt):
    """
    Check that the ordering of inputs is first wind direction, then wind speed,
    then turbulence intensity.

    Raises an error if this is found not to be the case.

    Args:
        df_opt (pd.DataFrame): A yaw offset lookup table.
    """

    inputs_all = df_opt[["wind_direction", "wind_speed", "turbulence_intensity"]].to_numpy()
    wd_unique = np.unique(df_opt["wind_direction"])
    ws_unique = np.unique(df_opt["wind_speed"])
    ti_unique = np.unique(df_opt["turbulence_intensity"])

    # Check full
    if not inputs_all.shape[0] == len(wd_unique)*len(ws_unique)*len(ti_unique):
        raise ValueError(
            "All combinations of wind direction, wind speed, and turbulence intensity "
            "must be specified."
        )

    # Check order is correct
    wds_reshaped = inputs_all[:,0].reshape((len(wd_unique), len(ws_unique), len(ti_unique)))
    wss_reshaped = inputs_all[:,1].reshape((len(wd_unique), len(ws_unique), len(ti_unique)))
    tis_reshaped = inputs_all[:,2].reshape((len(wd_unique), len(ws_unique), len(ti_unique)))

    if (not np.all(wds_reshaped == wd_unique[:,None,None])
        or not np.all(wss_reshaped == ws_unique[None,:,None])
        or not np.all(tis_reshaped == ti_unique[None,None,:])):
        raise ValueError(
            "df_opt must be ordered first by wind direction, then by wind speed,"
            "then by turbulence intensity."
        )
