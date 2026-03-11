from .odometry.env import OdometryEnv
from .odometry.bang_bang_agent import OdometryBangBangAgent
from .odometry.pid_agent import OdometryPIDAgent
from .odometry.sac_agent import OdometrySACAgent

from .tracking.env import TrackingEnv
from .tracking.agent import TrackingAgent

from .avoidance.env import AvoidanceEnv
from .avoidance.bang_bang_agent import AvoidanceBangBangAgent


def make_bang_bang_odometry_mdp(
    robot_2wd,
    max_forward_v,
    max_turn_v,
    dist_err_thres,
    heading_err_thres,
    dt,
):
    agent = OdometryBangBangAgent(
        max_forward_v=max_forward_v,
        max_turn_v=max_turn_v,
        dist_err_thres=dist_err_thres,
        heading_err_thres=heading_err_thres,
        dt=dt,
    )

    env = OdometryEnv(
        robot=robot_2wd,
        goal=(0, 0),
        # render_filename=f"{world_id}/odometry.png"
    )

    return agent, env


def make_pid_odometry_mdp(
    robot_2wd,
    max_forward_v,
    max_turn_v,
    dist_err_thres,
    heading_err_thres,
    dt,
):

    agent = OdometryPIDAgent(
        max_forward_v=max_forward_v,
        max_turn_v=max_turn_v,
        dist_err_thres=dist_err_thres,
        heading_err_thres=heading_err_thres,
        dt=dt,
    )

    env = OdometryEnv(
        robot=robot_2wd,
        goal=(0, 0),
        # render_filename=f"{world_id}/odometry.png"
    )

    return agent, env


def make_sac_odometry_mdp(
    robot_2wd,
    model_path,
    max_forward_v,
    max_turn_v,
    dist_err_thres,
    heading_err_thres,
    dt,
    obs_norm_radius=6,
    render_filename="",
):

    agent = OdometrySACAgent(
        model_path=model_path,
        max_forward_v=max_forward_v,
        max_turn_v=max_turn_v,
        dist_err_thres=dist_err_thres,
        heading_err_thres=heading_err_thres,
        dt=dt,
        obs_norm_radius=obs_norm_radius,
    )

    env = OdometryEnv(
        robot=robot_2wd,
        goal=(0, 0),
        render_filename=render_filename,
    )

    return agent, env


def make_tracking_mdp(
    robot_2wd,
    cam_link,
    max_forward_v,
    max_turn_v,
    dt,
    *,
    target_color_hex=None,
    cascade_classfier_path=None,
    left_margin: float = 0.4,
    right_margin: float = 0.6,
):

    agent = TrackingAgent(
        dt=dt,
        max_forward_v=max_forward_v,
        max_turn_v=max_turn_v,
        left_margin=left_margin,
        right_margin=right_margin,
    )

    env = TrackingEnv(
        robot=robot_2wd,
        cam_client=cam_link,
        cascade_classfier_path=cascade_classfier_path,
        target_color_hex=target_color_hex,
        use_cascade=cascade_classfier_path is not None,
    )

    return agent, env


def make_bang_bang_avoidance_mdp(
    robot_2wd,
    velocity_max,
    dt,
    safe_distance_min,
    forward_seq,
):
    agent = AvoidanceBangBangAgent(
        dt=dt,
        safe_distance_min=safe_distance_min,
        velocity_max=velocity_max,
        forward_seq=forward_seq,
    )

    env = AvoidanceEnv(
        robot=robot_2wd,
    )

    return agent, env
