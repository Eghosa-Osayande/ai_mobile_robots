import os, json
from datetime import datetime


def run_experiment(
    file,
    robot_2wd,
    odom_agent,
    odom_env,
    tracking_agent,
    tracking_env,
    nodes,
    avoid_agent=None,
    avoid_env=None,
):
    current_dir = os.path.dirname(os.path.abspath(file))
    experiment_id = os.path.basename(file).split(".")[0]

    now = datetime.now()
    ts = now.strftime("%Y-%m-%d-") + str(int(now.timestamp()))

    logs_path = f"{current_dir}/logs/{experiment_id}/{ts}"

    logs = []

    try:
        # Odometry

        if len(nodes) > 0:
            print("Odometry Start")

            robot_2wd.step(2)

            for node in nodes:
                obs, _ = odom_env.reset(
                    goal=node,
                )

                while True:
                    action = odom_agent.act(obs)
                    obs, _, is_avoiding, truncated, info = odom_env.step(action)
                    logs.append(info)
                    if is_avoiding or truncated:
                        break
            os.makedirs(logs_path, exist_ok=True)
            odom_env.render(path=logs_path + "/odometry.png")
            print("Odometry End")
            robot_2wd.step(2)

        # Tracking/Avoidance

        track_obs, _ = tracking_env.reset()
        if avoid_env:
            avoid_obs, _ = avoid_env.reset()

        while True:
            if avoid_agent and avoid_env:
                while True:
                    avoid_obs = avoid_env._obs()
                    avoid_action = avoid_agent.act(avoid_obs)
                    _, _, is_clear, _, info = avoid_env.step(
                        avoid_action,
                    )
                    # print("is clear =>", is_clear)
                    # infos.append(info)
                    if is_clear:
                        break

            track_action = tracking_agent.act(track_obs)

            track_obs, _, terminated, truncated, info = tracking_env.step(
                track_action,
            )

            logs.append(info)

            if terminated or truncated:
                break

        print("Tracking/Avoidance End")

        if avoid_env:
            avoid_env.close()
        tracking_env.close()
        odom_env.close()
    except Exception as e:
        raise e
    finally:
        logs_str = json.dumps(logs)

        os.makedirs(logs_path, exist_ok=True)
        log_file = f"{logs_path}/logs.json"

        with open(log_file, "w") as fd:
            fd.write(logs_str)

        print("Saved logs to ", log_file)
