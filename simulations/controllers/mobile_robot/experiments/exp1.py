import os

from mdps import (
    make_bang_bang_odometry_mdp,
    make_tracking_mdp,
    make_bang_bang_avoidance_mdp,
)

from common import robot_instance, camera_instance

from .helpers import run_experiment


def main():
    o_agent, o_env = make_bang_bang_odometry_mdp(
        robot_2wd=robot_instance(),
        dist_err_thres=0.1,
        heading_err_thres=5,
        max_forward_v=0.02 * 20,  # 10 / 20
        max_turn_v=0.02 * 2,
        dt=0.1,
    )

    t_agent, t_env = make_tracking_mdp(
        robot_2wd=robot_instance(),
        cam_link=camera_instance(),
        max_forward_v=0.08,
        max_turn_v=0.04,
        dt=0.01,
        target_color_hex="#ff0000",
    )

    avoid_agent, avoid_env = make_bang_bang_avoidance_mdp(
        robot_2wd=robot_instance(),
        dt=0.1,
        forward_seq=5,
        safe_distance_min=15 / 100,
        velocity_max=0.02 * 5,
    )

    odometry_nodes = [
        (2.1, 0),
        (2.1, -2.4),
        (4.4, -2.4),
        (4.4, -3.75),
        (-1.5, -3.75),
    ]

    run_experiment(
        file=__file__,
        robot_2wd=robot_instance(),
        odom_agent=o_agent,
        odom_env=o_env,
        tracking_agent=t_agent,
        tracking_env=t_env,
        nodes=odometry_nodes,
        avoid_env=avoid_env,
        avoid_agent=avoid_agent,
    )
