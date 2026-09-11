import numpy as np
import cvxpy as cp


class MPCController:
    """
    Линейный MPC вокруг режима висения.

    Состояние:
        x = [x, z, theta, vx, vz, omega]

    Управление внутри MPC:
        u = [delta_T, tau]

    delta_T — добавка к mg.
    tau     — момент вокруг оси Y.

    Затем:
        T = mg + delta_T
        f1 = (T - tau/L) / 2
        f2 = (T + tau/L) / 2
    """

    def __init__(self, dt=0.02, horizon=25, mass=1.0, inertia=0.02,
                 arm=0.32, gravity=9.81, f_max=9.0):
        self.dt = float(dt)
        self.N = int(horizon)
        self.m = float(mass)
        self.I = float(inertia)
        self.L = float(arm)
        self.g = float(gravity)
        self.f_max = float(f_max)

        # Непрерывная линеаризованная модель:
        # xdot = vx
        # zdot = vz
        # thetadot = omega
        # vxdot ≈ -g*theta
        # vzdot = delta_T/m
        # omegadot = tau/I
        Ac = np.zeros((6, 6))
        Ac[0, 3] = 1.0
        Ac[1, 4] = 1.0
        Ac[2, 5] = 1.0
        Ac[3, 2] = -self.g

        Bc = np.zeros((6, 2))
        Bc[4, 0] = 1.0 / self.m
        Bc[5, 1] = 1.0 / self.I

        # Простая дискретизация Эйлера.
        self.A = np.eye(6) + self.dt * Ac
        self.B = self.dt * Bc

        # Чем больше число, тем сильнее MPC "не любит" эту ошибку.
        self.Q = np.diag([70.0, 100.0, 35.0, 5.0, 8.0, 2.0])
        self.QN = np.diag([120.0, 160.0, 55.0, 8.0, 12.0, 4.0])
        self.R = np.diag([0.18, 0.30])

        self.X = cp.Variable((6, self.N + 1))
        self.U = cp.Variable((2, self.N))

        self.x0 = cp.Parameter(6)
        self.ref = cp.Parameter((6, self.N + 1))

        constraints = [self.X[:, 0] == self.x0]
        cost = 0

        # Ограничения на виртуальные входы MPC.
        delta_t_min = -0.65 * self.m * self.g
        delta_t_max = +0.80 * self.m * self.g
        tau_max = 0.32

        for k in range(self.N):
            err = self.X[:, k] - self.ref[:, k]
            cost += cp.quad_form(err, self.Q)
            cost += cp.quad_form(self.U[:, k], self.R)

            constraints += [
                self.X[:, k + 1] == self.A @ self.X[:, k] + self.B @ self.U[:, k],
                self.U[0, k] >= delta_t_min,
                self.U[0, k] <= delta_t_max,
                self.U[1, k] >= -tau_max,
                self.U[1, k] <= +tau_max,

                # Не даём линейной модели запрашивать безумный наклон.
                self.X[2, k] >= -0.65,
                self.X[2, k] <= +0.65,
            ]

        terminal_err = self.X[:, self.N] - self.ref[:, self.N]
        cost += cp.quad_form(terminal_err, self.QN)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def reference_point(self, t):
        """
        Вертикальная восьмёрка:
            x_ref = B sin(2wt)
            z_ref = z0 + A sin(wt)

        Двойная частота по X формирует вертикально ориентированную
        траекторию "8" в плоскости X-Z.
        """
        A = 1.15
        B = 0.75
        z0 = 2.35
        period = 10.0
        w = 2.0 * np.pi / period

        x = B * np.sin(2.0 * w * t)
        z = z0 + A * np.sin(w * t)
        return x, z

    def build_reference(self, t_now):
        ref = np.zeros((6, self.N + 1))

        A = 1.15
        B = 0.75
        z0 = 2.35
        period = 10.0
        w = 2.0 * np.pi / period

        for k in range(self.N + 1):
            t = t_now + k * self.dt

            x = B * np.sin(2.0 * w * t)
            z = z0 + A * np.sin(w * t)

            vx = 2.0 * B * w * np.cos(2.0 * w * t)
            vz = A * w * np.cos(w * t)

            ref[:, k] = [x, z, 0.0, vx, vz, 0.0]

        return ref

    def control(self, state, t_now):
        self.x0.value = np.asarray(state, dtype=float)
        ref = self.build_reference(t_now)
        self.ref.value = ref

        try:
            self.problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                verbose=False,
                eps_abs=1e-4,
                eps_rel=1e-4,
                max_iter=4000,
            )
        except Exception:
            self.problem.solve(solver=cp.CLARABEL, verbose=False)

        if self.U.value is None:
            # Безопасный запасной режим: просто висение.
            delta_T = 0.0
            tau = 0.0
        else:
            delta_T = float(self.U.value[0, 0])
            tau = float(self.U.value[1, 0])

        total_thrust = self.m * self.g + delta_T

        f1 = 0.5 * (total_thrust - tau / self.L)
        f2 = 0.5 * (total_thrust + tau / self.L)

        # Физические ограничения каждого "мотора".
        f1 = float(np.clip(f1, 0.0, self.f_max))
        f2 = float(np.clip(f2, 0.0, self.f_max))

        x_ref, z_ref = self.reference_point(t_now)
        return f1, f2, x_ref, z_ref
