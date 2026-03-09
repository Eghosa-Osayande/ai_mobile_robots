# CONFIG

# odometry
odometry_nodes = [
    (2, 0),
    (2, -2.3),
    (4.4, -2.3),
    (4.4, -3.7),
    (3.4, -3.7),
    (2.4, -3.7),
    (1.4, -3.7),
    (0.4, -3.7),
    (-1.4, -3.7),
]
odometry_nodes=[
    (2,0),
    (2,0.5),
]


def run(robot_2wd, world_id):
    import math
    from stable_baselines3 import SAC
    from contribution.rl.mdp.obstacle_navigation.env import (
        TwoWheelObstacleNavigationEnv,
    )

    infos = []

    model = SAC.load(
        "contribution/rl/training_outputs/obstacle_navigation/checkpoints/_1680000_steps.zip"
    )

    evn = TwoWheelObstacleNavigationEnv(
        robot_factory=lambda *_, **kw: robot_2wd,
        wheel_speed_limit=0.02 * 5,
        dt=0.001,
        max_steps=999,
        goal_radius=0.15,
        arena_radius=6.0,
        obs_norm_radius=6.0,
        render_enabled=True,
        allow_reverse= not True,
        robot_theta_tranformer=lambda th: math.radians(th),
        render_path=f"{world_id}/odometry.png",
        safe_distance=0.1,
    )

    for node in odometry_nodes:
        obs, _ = evn.reset(
            goal_xy=node,
            reset_robot=False,
        )

        while True:
            action, _ = model.predict(obs)
            obs, _, terminated, truncated, info = evn.step(action[::-1])

            infos.append(info)

            if terminated or truncated:
                break

    evn.close()

    return infos
