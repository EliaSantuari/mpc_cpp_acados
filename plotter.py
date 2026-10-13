import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle
import yaml



def animate_robot_feet(history_X, history_obs, obs_r, target, steps_per_phase, pred_op=None, pred_pe=None, interval=80, tail_frames=40):
    """
    Anima CoM, impronte/traettorie dei piedi e ostacolo (Versione Standalone)
    """
    traj = np.asarray(history_X)
    obs_traj = np.asarray(history_obs)

    if pred_op is not None: pred_op = np.asarray(pred_op)
    if pred_pe is not None: pred_pe = np.asarray(pred_pe)

    n = min(len(traj), len(obs_traj))
    traj, obs_traj = traj[:n], obs_traj[:n]
    if pred_op is not None: pred_op = pred_op[:n]
    if pred_pe is not None: pred_pe = pred_pe[:n]

    com = traj[:, 0:2]
    p0 = traj[:, 6:8]
    p1 = traj[:, 8:10]

    fig, ax = plt.subplots(figsize=(9, 7))

    all_x = np.concatenate((com[:, 0], p0[:, 0], p1[:, 0], obs_traj[:, 0]))
    all_y = np.concatenate((com[:, 1], p0[:, 1], p1[:, 1], obs_traj[:, 1]))

    if pred_op is not None:
        all_x = np.concatenate((all_x, pred_op[:, :, 0].ravel()))
        all_y = np.concatenate((all_y, pred_op[:, :, 1].ravel()))
    if pred_pe is not None:
        all_x = np.concatenate((all_x, pred_pe[:, :, 0].ravel()))
        all_y = np.concatenate((all_y, pred_pe[:, :, 1].ravel()))

    margin = obs_r + 0.3
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Animazione CoM, impronte dei piedi e ostacolo")
    ax.grid(True, alpha=0.4)

    target_x, target_y = target
    ax.plot(target_x, target_y, marker="x", markersize=10, markeredgewidth=2.5,
            color="dodgerblue", zorder=5, label=f"Target {target_x, target_y}")

    com_line, = ax.plot([], [], color="black", linewidth=1.8, alpha=0.65, label="Traiettoria CoM")
    obs_line, = ax.plot([], [], color="red", linestyle="--", linewidth=1.5, alpha=0.6, label="Traiettoria ostacolo")

    leg0_line, = ax.plot([], [], color="forestgreen", linewidth=1.8, label="CoM–p0")
    leg1_line, = ax.plot([], [], color="purple", linewidth=1.8, label="CoM–p1")

    p0_trail = LineCollection([], linewidths=2.0, zorder=2)
    p1_trail = LineCollection([], linewidths=2.0, zorder=2)
    ax.add_collection(p0_trail)
    ax.add_collection(p1_trail)

    p0_marks = ax.scatter([], [], marker="^", s=75, c="green", edgecolors="black", zorder=4, label="Impronte p0")
    p1_marks = ax.scatter([], [], marker="v", s=75, c="purple", edgecolors="black", zorder=4, label="Impronte p1")

    com_dot, = ax.plot([], [], "ko", markersize=8, zorder=6, label="CoM")

    obstacle_now = Circle((0, 0), obs_r, color="red", alpha=0.25, zorder=1, label="Ostacolo")
    ax.add_patch(obstacle_now)
    obstacle_center, = ax.plot([], [], "ro", markersize=5, zorder=5)

    ghost_circles = []

    def fading_segments(points, start, end, color):
        if end - start < 2:
            return [], []
        segments, colors = [], []
        for j in range(start, end - 1):
            segments.append([points[j], points[j + 1]])
            age = (end - 1) - j
            alpha = max(0.05, 0.8 * (1 - age / max(1, tail_frames)))
            colors.append((*plt.matplotlib.colors.to_rgb(color), alpha))
        return segments, colors

    pred_op_line, = ax.plot([], [], color="deepskyblue", linestyle="--", marker=".", markersize=5, linewidth=2, label="Horizon ottimistico", zorder=7)
    pred_pe_line, = ax.plot([], [], color="darkorange", linestyle="--", marker=".", markersize=5, linewidth=2, label="Horizon pessimistico", zorder=7)

    def update(frame):
        nonlocal ghost_circles

        if pred_op is not None: pred_op_line.set_data(pred_op[frame, :, 0], pred_op[frame, :, 1])
        if pred_pe is not None: pred_pe_line.set_data(pred_pe[frame, :, 0], pred_pe[frame, :, 1])

        start = max(0, frame - tail_frames + 1)

        com_line.set_data(com[start:frame + 1, 0], com[start:frame + 1, 1])
        com_dot.set_data([com[frame, 0]], [com[frame, 1]])

        leg0_line.set_data([com[frame, 0], p0[frame, 0]], [com[frame, 1], p0[frame, 1]])
        leg1_line.set_data([com[frame, 0], p1[frame, 0]], [com[frame, 1], p1[frame, 1]])

        seg0, col0 = fading_segments(p0, start, frame + 1, "green")
        seg1, col1 = fading_segments(p1, start, frame + 1, "purple")
        p0_trail.set_segments(seg0)
        p0_trail.set_colors(col0)
        p1_trail.set_segments(seg1)
        p1_trail.set_colors(col1)

        idx = np.arange(0, frame + 1, steps_per_phase)
        idx = idx[idx >= start]
        ages = frame - idx
        alphas = np.maximum(0.05, 0.85 * (1 - ages / max(1, tail_frames)))

        p0_marks.set_offsets(p0[idx])
        p1_marks.set_offsets(p1[idx])
        p0_marks.set_facecolors([(0.0, 0.5, 0.0, a) for a in alphas])
        p1_marks.set_facecolors([(0.5, 0.0, 0.5, a) for a in alphas])

        obs_line.set_data(obs_traj[start:frame + 1, 0], obs_traj[start:frame + 1, 1])

        for circle in ghost_circles:
            circle.remove()
        ghost_circles.clear()

        stride = max(1, tail_frames // 12)
        for j in list(range(start, frame, stride)):
            age = frame - j
            alpha = max(0.04, 0.45 * (1 - age / max(1, tail_frames)))
            ghost = Circle(obs_traj[j, :2], obs_r, fill=False, edgecolor="red", linewidth=1.4, alpha=alpha, zorder=1)
            ax.add_patch(ghost)
            ghost_circles.append(ghost)

        obstacle_now.center = obs_traj[frame, :2]
        obstacle_center.set_data([obs_traj[frame, 0]], [obs_traj[frame, 1]])

        ax.set_title(f"Step {frame + 1}/{n} — Raggio ostacolo: {obs_r:.2f} m")

        return (com_line, com_dot, leg0_line, leg1_line, p0_trail, p1_trail, 
                p0_marks, p1_marks, obs_line, obstacle_now, obstacle_center, 
                pred_op_line, pred_pe_line)

    ax.legend(loc="best")
    animation = FuncAnimation(fig, update, frames=n, interval=interval, blit=False, repeat=False)
    return animation


def plot_extra_graphs(df):
    """
    Crea i nuovi plot richiesti senza sovrascrivere o alterare quelli esistenti.
    Assumiamo che Alpha, Beta e Gamma siano rispettivamente in u4, u5 e u6.
    """

    with open("config.yaml", "r") as file:
            config = yaml.safe_load(file)
    
    
    v_max_x = config['limits']['v_max_x']
    v_max_y = config['limits']['v_max_x']
    theta_dot_max = config['limits']['v_max_x']
    dt_min = config['limits']['dt_min']
    dt_max = config['limits']['dt_max']



    # 1. Velocità Locali del Robot con Limiti
    fig_vel, axes_vel = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    axes_vel[0].plot(df.index, df['x3'], color='blue', label='v_x')
    axes_vel[0].axhline(v_max_x, color='red', linestyle='--', label='Limite (±1.5)')
    axes_vel[0].axhline(-v_max_x, color='red', linestyle='--')
    axes_vel[0].set_ylabel('v_x [m/s]')
    axes_vel[0].legend(loc='upper right')
    axes_vel[0].grid(True)
    
    axes_vel[1].plot(df.index, df['x4'], color='orange', label='v_y')
    axes_vel[1].axhline(v_max_y, color='red', linestyle='--', label='Limite (±0.45)')
    axes_vel[1].axhline(-v_max_y, color='red', linestyle='--')
    axes_vel[1].set_ylabel('v_y [m/s]')
    axes_vel[1].legend(loc='upper right')
    axes_vel[1].grid(True)
    
    axes_vel[2].plot(df.index, df['x5'], color='green', label='theta_dot')
    axes_vel[2].axhline(theta_dot_max, color='red', linestyle='--', label='Limite (±0.6)')
    axes_vel[2].axhline(-theta_dot_max, color='red', linestyle='--')
    axes_vel[2].set_ylabel('theta_dot [rad/s]')
    axes_vel[2].set_xlabel('Step')
    axes_vel[2].legend(loc='upper right')
    axes_vel[2].grid(True)
    fig_vel.suptitle('Velocità Locali e Limiti Kinematici', fontsize=14)
    fig_vel.tight_layout()

    # 2. Tempo di Computazione
    if 'comp_time' in df.columns:
        fig_time, ax_time = plt.subplots(figsize=(10, 4))
        ax_time.plot(df.index, df['comp_time'], color='purple', label='Tempo Computazionale')
        ax_time.set_ylabel('Tempo [ms]')
        ax_time.set_xlabel('Step')
        ax_time.set_title('Tempo di Computazione per Step')
        ax_time.grid(True)
        ax_time.legend()
        fig_time.tight_layout()

    # 3. Variabili di Controllo
    fig_ctrl, axes_ctrl = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
    
    if 'u4' in df.columns:
        axes_ctrl[0].plot(df.index, df['u4'], color='royalblue', label='Alpha')
        axes_ctrl[0].axhline(0.1, color='red', linestyle='--')
        axes_ctrl[0].axhline(0.9, color='red', linestyle='--', label='Limiti (0.1 - 0.9)')
    axes_ctrl[0].set_ylabel('Alpha')
    axes_ctrl[0].legend(loc='upper right')
    axes_ctrl[0].grid(True)
    
    if 'u5' in df.columns:
        axes_ctrl[1].plot(df.index, df['u5'], color='forestgreen', label='Beta')
        axes_ctrl[1].axhline(0, color='red', linestyle='--')
        axes_ctrl[1].axhline(1, color='red', linestyle='--', label='Limiti (0 - 1)')
    axes_ctrl[1].set_ylabel('Beta')
    axes_ctrl[1].legend(loc='upper right')
    axes_ctrl[1].grid(True)
    
    if 'u6' in df.columns:
        axes_ctrl[2].plot(df.index, df['u6'], color='darkorchid', label='Gamma')
        axes_ctrl[2].axhline(0, color='red', linestyle='--')
        axes_ctrl[2].axhline(1, color='red', linestyle='--', label='Limiti (0 - 1)')
    axes_ctrl[2].set_ylabel('Gamma')
    axes_ctrl[2].legend(loc='upper right')
    axes_ctrl[2].grid(True)
    
    if 'u7' in df.columns:
        dt_vals = df['u7'].values
        if np.max(dt_vals) > 10:  # Assicura conversione in secondi se i dati sono in ms
            dt_vals = dt_vals / 1000.0
        axes_ctrl[3].plot(df.index, dt_vals, color='darkorange', label='dt')
        axes_ctrl[3].axhline(dt_min, color='red', linestyle='--')
        axes_ctrl[3].axhline(dt_max, color='red', linestyle='--', label='Limiti (0.1 - 0.25)')
    axes_ctrl[3].set_ylabel('dt [s]')
    axes_ctrl[3].set_xlabel('Step')
    axes_ctrl[3].legend(loc='upper right')
    axes_ctrl[3].grid(True)
    fig_ctrl.suptitle('Variabili di Controllo: Alpha, Beta, Gamma, dt', fontsize=14)
    fig_ctrl.tight_layout()

    # 4. Distanza Piede - Anca Futura (stessa figura, due plot separati)
    off_x = config['robot']['off_x']
    off_y = config['robot']['off_y']
    
    # State values per step k+1 (Futuro)
    cx_next = df['x0'].values[1:]
    cy_next = df['x1'].values[1:]
    th_next = df['x2'].values[1:]
    
    # Posizione dei piedi allo step k (Attuale)
    p0x_curr = df['x6'].values[:-1]
    p0y_curr = df['x7'].values[:-1]
    p1x_curr = df['x8'].values[:-1]
    p1y_curr = df['x9'].values[:-1]
    
    # Posizione dei piedi allo step k+1 (Futuro)
    p0x_next = df['x6'].values[1:]
    p0y_next = df['x7'].values[1:]
    p1x_next = df['x8'].values[1:]
    p1y_next = df['x9'].values[1:]

    # Numero di elementi
    n = len(cx_next)

    eff_off_y_prec = np.where(np.arange(n) % 2 == 0, off_y, -off_y)
    eff_off_y = np.where(np.arange(n) % 2 == 0, -off_y, off_y)


    # Trasformazione Rototraslazionale dell'anca al tempo k+1
    # Anca P0 (ipotizzata con Y offset negativo)
    hip0x_next_prec = cx_next + off_x * np.cos(th_next) - eff_off_y_prec * np.sin(th_next)
    hip0y_next_prec = cy_next + off_x * np.sin(th_next) + eff_off_y_prec * np.cos(th_next)
    
    # Anca P1 (ipotizzata con Y offset positivo)
    hip1x_next_prec = cx_next + (-off_x) * np.cos(th_next) - (-eff_off_y_prec) * np.sin(th_next)
    hip1y_next_prec = cy_next + (-off_x) * np.sin(th_next) + (-eff_off_y_prec) * np.cos(th_next)

    ## Piede Successivo
    hip0x_next = cx_next + off_x * np.cos(th_next) - eff_off_y * np.sin(th_next)
    hip0y_next = cy_next + off_x * np.sin(th_next) + eff_off_y * np.cos(th_next)
    
    # Anca P1 (ipotizzata con Y offset positivo)
    hip1x_next = cx_next + (-off_x) * np.cos(th_next) - (-eff_off_y) * np.sin(th_next)
    hip1y_next = cy_next + (-off_x) * np.sin(th_next) + (-eff_off_y) * np.cos(th_next)
    
    # Calcolo Distanze (Attuale - Anca Futura)
    dist0_curr_hip = np.sqrt((p0x_curr - hip0x_next_prec)**2 + (p0y_curr - hip0y_next_prec)**2)
    dist1_curr_hip = np.sqrt((p1x_curr - hip1x_next_prec)**2 + (p1y_curr - hip1y_next_prec)**2)
    
    # Calcolo Distanze (Futuro - Anca Futura)
    dist0_next_hip = np.sqrt((p0x_next - hip0x_next)**2 + (p0y_next - hip0y_next)**2)
    dist1_next_hip = np.sqrt((p1x_next - hip1x_next)**2 + (p1y_next - hip1y_next)**2)
    
    fig_kin, (ax_curr, ax_next) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    max_ext = np.sqrt(config['robot']['max_ext_sq'])
    
    # Plot 1: Piede Attuale e Anca Futura
    ax_curr.plot(dist0_curr_hip, label='P0 attuale -> Anca(k+1)', color='green')
    ax_curr.plot(dist1_curr_hip, label='P1 attuale -> Anca(k+1)', color='purple')
    ax_curr.axhline(max_ext, color='red', linestyle='--', linewidth=2, label='Limite (0.3m)')
    ax_curr.set_title('Distanza: Piede Attuale e Anca Nello Step Successivo')
    ax_curr.set_ylabel('Distanza [m]')
    ax_curr.legend()
    ax_curr.grid(True)
    
    # Plot 2: Piede Futuro e Anca Futura
    ax_next.plot(dist0_next_hip, label='P0(k+1) -> Anca(k+1)', color='green')
    ax_next.plot(dist1_next_hip, label='P1(k+1) -> Anca(k+1)', color='purple')
    ax_next.axhline(max_ext, color='red', linestyle='--', linewidth=2, label='Limite (0.3m)')
    ax_next.set_title('Distanza: Piede Futuro e Anca Futura (k+1)')
    ax_next.set_xlabel('Step')
    ax_next.set_ylabel('Distanza [m]')
    ax_next.legend()
    ax_next.grid(True)
    
    fig_kin.tight_layout()


def plot_standalone_results(csv_path):
    if not os.path.exists(csv_path):
        print(f"Errore: Il file {csv_path} non esiste.")
        return

    print(f"Lettura dei dati da {csv_path}...")
    df = pd.read_csv(csv_path)
    df = df.apply(pd.to_numeric, errors='coerce')
    steps = len(df)

    t_comp = df['comp_time'].values

    cx, cy = df['x0'].values, df['x1'].values
    theta = df['x2'].values
    vx, vy, vth = df['x3'].values, df['x4'].values, df['x5'].values
    p0x, p0y = df['x6'].values, df['x7'].values
    p1x, p1y = df['x8'].values, df['x9'].values

    alpha = df['u4'].values
    dt = df['u7'].values / 1000.0 

    obs_x, obs_y = df['obs_x'].values, df['obs_y'].values

    pred_op_x_cols = [c for c in df.columns if c.startswith('pred_op_') and c.endswith('_x')]
    N_horizon = len(pred_op_x_cols)
    print(f"Dati caricati: {steps} steps, Orizzonte MPC N={N_horizon}")

    pred_op = []
    pred_pe = []
    if N_horizon > 0:
        for i in range(steps):
            p_op_x = [df[f'pred_op_{k}_x'].iloc[i] for k in range(N_horizon)]
            p_op_y = [df[f'pred_op_{k}_y'].iloc[i] for k in range(N_horizon)]
            pred_op.append(np.column_stack((p_op_x, p_op_y)))

            p_pe_x = [df[f'pred_pe_{k}_x'].iloc[i] for k in range(N_horizon)]
            p_pe_y = [df[f'pred_pe_{k}_y'].iloc[i] for k in range(N_horizon)]
            pred_pe.append(np.column_stack((p_pe_x, p_pe_y)))

    # --- Grafici Statici (Originali Intatti) ---
    fig_traj, ax_traj = plt.subplots(figsize=(10, 8))
    ax_traj.plot(cx, cy, 'b-', linewidth=2, label='Traiettoria CoM', zorder=2)
    ax_traj.plot(obs_x, obs_y, 'r--', linewidth=2, label='Traiettoria Ostacolo', zorder=2)
    ax_traj.plot(cx[0], cy[0], 'bo', markersize=8, label='Start Robot')
    ax_traj.plot(obs_x[0], obs_y[0], 'ro', markersize=8, label='Start Ostacolo')
    ax_traj.plot(cx[-1], cy[-1], 'b*', markersize=12, label='End Robot')

    step_draw = 1
    ax_traj.scatter(p0x[::step_draw], p0y[::step_draw], c='green', marker='^', s=60, label='Impronte Piede 0', zorder=3, alpha=0.8)
    ax_traj.scatter(p1x[::step_draw], p1y[::step_draw], c='purple', marker='v', s=60, label='Impronte Piede 1', zorder=3, alpha=0.8)

    ax_traj.set_title('Traiettoria 2D Statica', fontsize=14)
    ax_traj.legend(loc='best')
    ax_traj.grid(True)
    ax_traj.axis('equal')
    
    # ----------------------------------------------------
    # CHIAMATA ALLE NUOVE FUNZIONI DI PLOT
    # ----------------------------------------------------
    plot_extra_graphs(df)

    # Ricostruzione Vettori per l'Animazione
    history_X = np.column_stack([cx, cy, theta, vx, vy, vth, p0x, p0y, p1x, p1y])
    history_obs = np.column_stack([obs_x, obs_y])

    # Importo i valori dei parametri
    with open("config.yaml", "r") as file:
                config = yaml.safe_load(file)


    obs_radius = config['obstacles']['r_obs']
    target_pos = config['simulation']['c_target']
    steps_per_phase = config['simulation']['step_per_phase']

    # Lancia l'animazione!
    global ani 
    ani = animate_robot_feet(
        history_X=history_X,
        history_obs=history_obs,
        obs_r=obs_radius,
        target=target_pos,
        steps_per_phase=steps_per_phase,
        pred_op=pred_op,
        pred_pe=pred_pe,
        interval=100,
        tail_frames=15
    )

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    percorso_csv = "build/mpc_simulation_data.csv"
    if not os.path.exists(percorso_csv) and os.path.exists("mpc_simulation_data.csv"):
        percorso_csv = "mpc_simulation_data.csv"

    



    plot_standalone_results(percorso_csv)


