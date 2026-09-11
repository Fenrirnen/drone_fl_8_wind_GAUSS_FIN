from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[1] / "examples" / "quadrotor_drone" / "sim"
sys.path.insert(0, str(SIM_DIR))

from wind import GaussianWind  # noqa: E402


def test_wind_is_reproducible_with_same_seed():
    w1 = GaussianWind(seed=7)
    w2 = GaussianWind(seed=7)
    values1 = [w1.sample() for _ in range(20)]
    values2 = [w2.sample() for _ in range(20)]
    assert values1 == values2


def test_wind_is_downward_on_average():
    wind = GaussianWind(mean_force=-0.7, sigma=0.30, alpha=0.85, seed=7)
    values = [wind.sample() for _ in range(1000)]
    average = sum(values) / len(values)
    assert average < 0.0
