import json
import zmq

from mpc_controller import MPCController


def main():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://127.0.0.1:5555")

    controller = MPCController(
        dt=0.02,
        horizon=25,
        mass=1.0,
        inertia=0.02,
        arm=0.32,
        gravity=9.81,
        f_max=9.0,
    )

    print("MPC controller started.")
    print("Waiting for simulator on tcp://127.0.0.1:5555 ...")

    while True:
        msg = socket.recv_json()

        if msg.get("command") == "stop":
            socket.send_json({"ok": True})
            break

        state = [
            msg["x"],
            msg["z"],
            msg["theta"],
            msg["vx"],
            msg["vz"],
            msg["omega"],
        ]
        t = msg["t"]

        f1, f2, x_ref, z_ref = controller.control(state, t)

        socket.send_json({
            "f1": f1,
            "f2": f2,
            "x_ref": x_ref,
            "z_ref": z_ref,
        })

    socket.close()
    context.term()
    print("Controller stopped.")


if __name__ == "__main__":
    main()
