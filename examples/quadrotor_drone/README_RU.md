# 2D квадрокоптер в вертикальном коридоре + Gaussian wind + MPC

## Что реализовано

- MuJoCo как физический симулятор.
- Планарный квадрокоптер: x, z, pitch.
- Заблокированы 3 пространственные степени свободы: y, roll, yaw.
- Два виртуальных ротора с силами f1 и f2.
- Коридор: пол, левая стена, правая стена.
- Нисходящий ветер как гауссовский случайный процесс.
- Траектория-восьмёрка:
  x_ref = A sin(w t)
  z_ref = z0 + B sin(2 w t)
- MPC через CVXPY + OSQP.
- Связь simulator <-> controller по ZeroMQ REQ/REP.
- CSV-лог и построение графиков.

## Установка Windows

Откройте PowerShell в корне проекта:

    py -m venv .venv
    .\.venv\Scripts\Activate.ps1
    python -m pip install --upgrade pip
    pip install -r requirements.txt

Если PowerShell запрещает активацию:

    Set-ExecutionPolicy -Scope Process Bypass
    .\.venv\Scripts\Activate.ps1

## Запуск

Нужно ДВА терминала.

Терминал 1:

    cd examples\quadrotor_drone\control
    python controller_server.py

Терминал 2:

    cd examples\quadrotor_drone\sim
    python simulator.py

После окончания:

    cd ..
    python plot_results.py

Результаты:
- results/flight_log.csv
- results/trajectory.png
- results/tracking_error.png
- results/wind.png

## Если дрон нестабилен

1. Сначала поставьте mean_force=0 и sigma=0 в sim/wind.py.
2. Проверьте полёт без ветра.
3. Потом верните mean_force=-0.7, sigma=0.30.
4. Если дрон слишком резко качается, уменьшите tau_max в mpc_controller.py.
5. Если слишком медленно реагирует по X, увеличьте Q[0,0].
6. Если слишком медленно по Z, увеличьте Q[1,1].
7. Если управление слишком резкое, увеличьте R.
