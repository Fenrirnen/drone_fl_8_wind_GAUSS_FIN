import csv
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import zmq

from wind import GaussianWind


HERE = Path(__file__).resolve().parent
XML_PATH = HERE / "corridor.xml"
RESULTS_DIR = HERE.parent / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
CSV_PATH = RESULTS_DIR / "flight_log.csv"

MASS = 1.0
ARM = 0.32
SIM_TIME = 30.0


def main():
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    # После трёх суставов qpos = [x, z, theta], qvel = [vx, vz, omega].
    # Начальное положение body в XML уже z=2.4, поэтому qpos z начинается с 0.
    # Для удобства считаем абсолютную высоту через body xpos.
    drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

    wind = GaussianWind(
        mean_force=-0.7,
        sigma=0.30,
        alpha=0.85,
        seed=7,
    )

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://127.0.0.1:5555")

    with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t", "x", "z", "theta", "vx", "vz", "omega",
            "x_ref", "z_ref", "f1", "f2", "wind_z"
        ])

        with mujoco.viewer.launch_passive(
            model, data,
            show_left_ui=False,
            show_right_ui=False
        ) as viewer:

            # Удобный вид спереди на плоскость X-Z.
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_CAMERA, "main"
            )

            while viewer.is_running() and data.time < SIM_TIME:
                step_start = time.time()

                # Углы/скорости берутся напрямую из planar joints.
                x = float(data.qpos[0])
                z_relative = float(data.qpos[1])
                theta = float(data.qpos[2])

                vx = float(data.qvel[0])
                vz = float(data.qvel[1])
                omega = float(data.qvel[2])

                # Абсолютная высота центра дрона.
                z_absolute = float(data.xpos[drone_id][2])

                # Отправляем состояние в отдельный процесс MPC.
                socket.send_json({
                    "t": float(data.time),
                    "x": x,
                    "z": z_absolute,
                    "theta": theta,
                    "vx": vx,
                    "vz": vz,
                    "omega": omega,
                })
                command = socket.recv_json()

                f1 = float(command["f1"])
                f2 = float(command["f2"])
                x_ref = float(command["x_ref"])
                z_ref = float(command["z_ref"])

                # Суммарная тяга и управляющий момент.
                total_thrust = f1 + f2
                tau = ARM * (f2 - f1)

                # Тяга направлена вдоль локальной оси "вверх" дрона.
                # После поворота на theta:
                thrust_x = -total_thrust * math.sin(theta)
                thrust_z = +total_thrust * math.cos(theta)

                # Ветер дует сверху вниз: wind_z обычно отрицательный.
                wind_z = float(wind.sample())

                # ВАЖНО: qfrc_applied надо задавать заново каждый шаг.
                data.qfrc_applied[:] = 0.0
                data.qfrc_applied[0] = thrust_x
                data.qfrc_applied[1] = thrust_z + wind_z
                data.qfrc_applied[2] = tau

                writer.writerow([
                    data.time, x, z_absolute, theta, vx, vz, omega,
                    x_ref, z_ref, f1, f2, wind_z
                ])

                mujoco.mj_step(model, data)
                viewer.sync()

                # Примерно реальное время.
                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)

    # Просим контроллер корректно завершиться.
    try:
        socket.send_json({"command": "stop"})
        socket.recv_json()
    except Exception:
        pass

    socket.close()
    context.term()
    print(f"Simulation finished. Log saved to: {CSV_PATH}")


if __name__ == "__main__":
    main()
