import numpy as np


class GaussianWind:
    """
    Ветер действует вдоль оси Z.
    Отрицательная сила = ветер дует сверху вниз.

    mean_force < 0 задаёт средний нисходящий ветер.
    sigma задаёт стандартное отклонение гауссовского шума.
    alpha делает шум немного "плавнее":
      0.0 -> почти белый шум;
      0.9 -> сильное сглаживание.
    """
    def __init__(self, mean_force=-0.7, sigma=0.30, alpha=0.85, seed=7):
        self.mean_force = float(mean_force)
        self.sigma = float(sigma)
        self.alpha = float(alpha)
        self.rng = np.random.default_rng(seed)
        self.filtered_noise = 0.0

    def sample(self):
        white_noise = self.rng.normal(0.0, self.sigma)
        self.filtered_noise = (
            self.alpha * self.filtered_noise
            + (1.0 - self.alpha) * white_noise
        )
        return self.mean_force + self.filtered_noise
