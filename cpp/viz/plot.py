import csv
import math
import re
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arrow
from matplotlib import animation

ROBOT_RADIUS = 0.3

def read_csv(path):
    rows = []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows, r.fieldnames if rows else []

def parse_obstacles_fieldnames(fieldnames):
    # 解析列名，找到所有 obs{i}_* 的字段
    # 返回：索引列表，每个索引包含该障碍的五个键
    pattern = re.compile(r"obs(\d+)_(ox|oy|r|vx|vy)$")
    obs_map = {}
    for fn in fieldnames:
        m = pattern.match(fn)
        if m:
            idx = int(m.group(1))
            key = m.group(2)
            obs_map.setdefault(idx, {})[key] = fn
    # 按索引排序
    indices = []
    for idx in sorted(obs_map.keys()):
        m = obs_map[idx]
        if all(k in m for k in ("ox", "oy", "r", "vx", "vy")):
            indices.append(m)
    return indices

def build_time_series(rows, obs_indices):
    # 机器人 & h_min 序列
    ts = [float(r["t"]) for r in rows]
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    thetas = [float(r["theta"]) for r in rows]
    vs = [float(r["v"]) for r in rows]
    hmins = [float(r["h_min"]) for r in rows]
    # 障碍物逐帧序列
    obs_series = []  # list of list per frame: [(ox,oy,r,vx,vy), ...]
    for r in rows:
        one = []
        for m in obs_indices:
            ox = float(r[m["ox"]])
            oy = float(r[m["oy"]])
            rr = float(r[m["r"]])
            vx = float(r[m["vx"]])
            vy = float(r[m["vy"]])
            one.append((ox, oy, rr, vx, vy))
        obs_series.append(one)
    return ts, xs, ys, thetas, vs, hmins, obs_series

def setup_animation_fig(xs, ys, ts, hmins, first_obs, ref_xs, ref_ys):
    fig = plt.figure(figsize=(12, 5))
    gs = fig.add_gridspec(1, 2, width_ratios=[7, 5])
    ax_env = fig.add_subplot(gs[0, 0])
    ax_h   = fig.add_subplot(gs[0, 1])

    ax_env.set_title('Trajectory & Dynamic Obstacles (CSV-driven)')
    ax_env.set_xlabel('x')
    ax_env.set_ylabel('y')
    ax_env.axis('equal')
    ax_env.grid(True)
    # 参考轨迹（来自 CSV），静态绘制
    if ref_xs and ref_ys:
        ax_env.plot(ref_xs, ref_ys, '--', color='gray', lw=1.5, label='Reference path')

    # 可视范围（根据轨迹与初帧障碍）
    x_all = xs + [o[0] for o in first_obs]
    y_all = ys + [o[1] for o in first_obs]
    if x_all and y_all:
        xmin, xmax = min(x_all) - 2.0, max(x_all) + 2.0
        ymin, ymax = min(y_all) - 2.0, max(y_all) + 2.0
        ax_env.set_xlim(xmin, xmax)
        ax_env.set_ylim(ymin, ymax)

    traj_line, = ax_env.plot([], [], '-b', lw=2, label='Robot trajectory')
    robot_patch = Circle((0, 0), ROBOT_RADIUS, edgecolor='blue', facecolor='cyan', alpha=0.4, zorder=3)
    ax_env.add_patch(robot_patch)
    yaw_arrow = Arrow(0, 0, 0, 0, width=0.1, color='blue', zorder=4)
    ax_env.add_patch(yaw_arrow)

    obs_patches = []
    vel_arrows = []
    rel_arrows = []
    cmap = plt.get_cmap('viridis')
    colors = [cmap(i / max(1, len(first_obs)-1)) for i in range(len(first_obs))]
    for i, (ox, oy, r, vx, vy) in enumerate(first_obs):
        c = Circle((ox, oy), r, edgecolor='black', facecolor=colors[i], alpha=0.5, zorder=2)
        ax_env.add_patch(c)
        obs_patches.append(c)
        va = Arrow(ox, oy, vx, vy, width=0.08, color='orange', zorder=4)
        ax_env.add_patch(va)
        vel_arrows.append(va)
        ra = Arrow(ox, oy, 0, 0, width=0.06, color='red', zorder=5)
        ax_env.add_patch(ra)
        rel_arrows.append(ra)

    ax_env.legend(loc='upper left')

    ax_h.set_title('Min h (DPCBF)')
    ax_h.set_xlabel('t')
    ax_h.set_ylabel('h_min')
    ax_h.grid(True)
    ax_h.plot(ts, hmins, '-r', lw=1.5)
    h_vline = ax_h.axvline(x=0.0, color='k', linestyle='--', lw=1)

    return fig, ax_env, traj_line, robot_patch, yaw_arrow, obs_patches, vel_arrows, rel_arrows, ax_h, h_vline

def main(save_mp4=False, mp4_path='output_anim.mp4'):
    rows, fieldnames = read_csv('output_dpcbf.csv')
    if not rows:
        print("No data found in output_dpcbf.csv")
        return

    obs_indices = parse_obstacles_fieldnames(fieldnames)
    ts, xs, ys, thetas, vs, hmins, obs_series = build_time_series(rows, obs_indices)
    # 参考轨迹坐标从 CSV 解析（若存在）
    ref_xs = [float(r["ref_x"]) for r in rows] if "ref_x" in fieldnames else []
    ref_ys = [float(r["ref_y"]) for r in rows] if "ref_y" in fieldnames else []

    fig, ax_env, traj_line, robot_patch, yaw_arrow, obs_patches, vel_arrows, rel_arrows, ax_h, h_vline = setup_animation_fig(xs, ys, ts, hmins, obs_series[0], ref_xs, ref_ys)

    def animate(k):
        nonlocal yaw_arrow
        t = ts[k]
        x, y, theta, v = xs[k], ys[k], thetas[k], vs[k]
        # 轨迹
        traj_line.set_data(xs[:k+1], ys[:k+1])
        # 机器人位置与朝向
        robot_patch.center = (x, y)
        arrow_len = max(0.5, min(1.0, v))
        yaw_arrow.remove()
        new_yaw = Arrow(x, y, arrow_len * math.cos(theta), arrow_len * math.sin(theta), width=0.1, color='blue', zorder=4)
        ax_env.add_patch(new_yaw)

        yaw_arrow = new_yaw

        # 障碍物（来自 CSV）
        obs_states = obs_series[k]
        for i, (ox, oy, r, vx, vy) in enumerate(obs_states):
            obs_patches[i].center = (ox, oy)
            obs_patches[i].set_radius(r)
            vel_arrows[i].remove()
            va = Arrow(ox, oy, vx, vy, width=0.08, color='orange', zorder=4)
            ax_env.add_patch(va)
            vel_arrows[i] = va
            # 相对速度箭头：v_rel = [vx - v cos(theta), vy - v sin(theta)]
            vrel_x = vx - v * math.cos(theta)
            vrel_y = vy - v * math.sin(theta)
            rel_arrows[i].remove()
            ra = Arrow(x, y, vrel_x, vrel_y, width=0.06, color='red', zorder=5)
            ax_env.add_patch(ra)
            rel_arrows[i] = ra

        h_vline.set_xdata([t, t])
        return [traj_line, robot_patch, h_vline] + obs_patches + vel_arrows + rel_arrows

    interval_ms = 50  # dt ~ 0.05s
    anim = animation.FuncAnimation(fig, animate, frames=len(ts), interval=interval_ms, blit=False, repeat=False)

    plt.tight_layout()
    if save_mp4:
        try:
            Writer = animation.writers['ffmpeg']
            fps = int(1.0 / (ts[1] - ts[0])) if len(ts) > 1 else 20
            writer = Writer(fps=fps, metadata=dict(artist='viz'), bitrate=1800)
            anim.save(mp4_path, writer=writer)
            print(f"Saved animation to {mp4_path}")
        except Exception as e:
            print(f"Failed to save mp4 (ffmpeg missing?): {e}")
            plt.show()
    else:
        plt.show()

if __name__ == '__main__':
    main(save_mp4=False)