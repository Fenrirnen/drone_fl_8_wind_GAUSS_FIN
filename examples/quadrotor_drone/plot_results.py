from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
csv_path = HERE / "results" / "flight_log.csv"

data = np.genfromtxt(csv_path, delimiter=",", names=True)

# 1. Траектория X-Z.
plt.figure()
plt.plot(data["x_ref"], data["z_ref"], "--", label="reference")
plt.plot(data["x"], data["z"], label="drone")
plt.xlabel("x, m")
plt.ylabel("z, m")
plt.title("2D trajectory tracking")
plt.grid(True)
plt.legend()
plt.axis("equal")
plt.tight_layout()
plt.savefig(HERE / "results" / "trajectory.png", dpi=160)

# 2. Ошибка слежения во времени.
error = np.sqrt(
    (data["x"] - data["x_ref"])**2
    + (data["z"] - data["z_ref"])**2
)
plt.figure()
plt.plot(data["t"], error)
plt.xlabel("time, s")
plt.ylabel("position error, m")
plt.title("Position tracking error")
plt.grid(True)
plt.tight_layout()
plt.savefig(HERE / "results" / "tracking_error.png", dpi=160)

# 3. Ветер.
plt.figure()
plt.plot(data["t"], data["wind_z"])
plt.xlabel("time, s")
plt.ylabel("wind force Z, N")
plt.title("Gaussian vertical wind")
plt.grid(True)
plt.tight_layout()
plt.savefig(HERE / "results" / "wind.png", dpi=160)

rmse = np.sqrt(np.mean(error**2))
max_error = np.max(error)

print(f"RMSE position error = {rmse:.4f} m")
print(f"Max position error  = {max_error:.4f} m")
print("Saved plots to results/")
