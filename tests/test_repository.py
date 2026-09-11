from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_required_project_files_exist():
    required = [
        "README.md",
        "requirements.txt",
        "requirements-dev.txt",
        "examples/quadrotor_drone/sim/corridor.xml",
        "examples/quadrotor_drone/sim/simulator.py",
        "examples/quadrotor_drone/sim/wind.py",
        "examples/quadrotor_drone/control/mpc_controller.py",
        "examples/quadrotor_drone/control/controller_server.py",
        "examples/quadrotor_drone/plot_results.py",
    ]
    for relative_path in required:
        assert (ROOT / relative_path).is_file(), relative_path
