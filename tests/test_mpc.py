from pathlib import Path
import sys

CONTROL_DIR = Path(__file__).resolve().parents[1] / "examples" / "quadrotor_drone" / "control"
sys.path.insert(0, str(CONTROL_DIR))

from mpc_controller import MPCController  # noqa: E402


def test_reference_is_vertical_figure_eight():
    controller = MPCController(dt=0.02, horizon=5)

    # At t=0 the trajectory is at the center of the corridor,
    # moving upward, with zero horizontal velocity.
    x, z = controller.reference_point(0.0)
    assert abs(x) < 1e-12
    assert abs(z - 2.35) < 1e-12

    # Quarter period: x returns to zero while z reaches its upper extreme.
    period = 10.0
    x, z = controller.reference_point(period / 4.0)
    assert abs(x) < 1e-10
    assert abs(z - (2.35 + 1.15)) < 1e-10


def test_build_reference_has_expected_shape():
    controller = MPCController(dt=0.02, horizon=5)
    ref = controller.build_reference(0.0)
    assert ref.shape == (6, 6)
    assert ref[0, 0] == 0.0
    assert abs(ref[1, 0] - 2.35) < 1e-12
