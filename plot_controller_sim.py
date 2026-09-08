from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ===== LOAD DATA =====
folder = Path(__file__).resolve().parent
data_path = folder / "controller_sim.csv"

if not data_path.exists():
    raise FileNotFoundError("Run Controller_Sim.exe before running this script.")

data = np.genfromtxt(
    data_path,
    delimiter=',',
    names=True
)

time = data["time"]

target_n = data["target_N"]
target_e = data["target_E"]
target_alt = data["target_alt"]

quad_n = data["x6_pN"]
quad_e = data["x7_pE"]
quad_alt = -data["x8_pD"]

vN_cmd = data["vN_cmd"]
vE_cmd = data["vE_cmd"]
vH_cmd = data["vH_cmd"]

vN = data["x9_vN"]
vE = data["x10_vE"]
vH = -data["x11_vD"]


# ===== SET UP 3D ANIMATION =====
figure = plt.figure(figsize=(10, 8))
axis = figure.add_subplot(111, projection="3d")

all_n = np.concatenate((target_n, quad_n))
all_e = np.concatenate((target_e, quad_e))
all_alt = np.concatenate((target_alt, quad_alt))


def padded_limits(values):
    minimum = values.min()
    maximum = values.max()
    padding = max((maximum - minimum)*.05, .5)
    return minimum - padding, maximum + padding


axis.set_xlim(*padded_limits(all_n))
axis.set_ylim(*padded_limits(all_e))
axis.set_zlim(*padded_limits(all_alt))
axis.set_box_aspect((np.ptp(all_n), np.ptp(all_e), np.ptp(all_alt)))

axis.set_xlabel("North (m)")
axis.set_ylabel("East (m)")
axis.set_zlabel("Altitude (m)")
axis.set_title("Target and Quadcopter Trajectories")

axis.plot(target_n, target_e, target_alt, "--", color="green", alpha=.25)
target_line, = axis.plot([], [], [], color="green", linewidth=2, label="Target")
quad_line, = axis.plot([], [], [], color="blue", linewidth=2, label="Quadcopter")
target_point, = axis.plot([], [], [], "o", color="green")
quad_point, = axis.plot([], [], [], "o", color="blue")
time_text = axis.text2D(.02, .95, "", transform=axis.transAxes)

axis.legend()


def init():
    target_line.set_data([], [])
    target_line.set_3d_properties([])
    quad_line.set_data([], [])
    quad_line.set_3d_properties([])
    target_point.set_data([], [])
    target_point.set_3d_properties([])
    quad_point.set_data([], [])
    quad_point.set_3d_properties([])
    time_text.set_text("")
    return target_line, quad_line, target_point, quad_point, time_text


def update(frame):
    target_line.set_data(target_n[:frame], target_e[:frame])
    target_line.set_3d_properties(target_alt[:frame])
    quad_line.set_data(quad_n[:frame], quad_e[:frame])
    quad_line.set_3d_properties(quad_alt[:frame])

    target_point.set_data([target_n[frame - 1]], [target_e[frame - 1]])
    target_point.set_3d_properties([target_alt[frame - 1]])
    quad_point.set_data([quad_n[frame - 1]], [quad_e[frame - 1]])
    quad_point.set_3d_properties([quad_alt[frame - 1]])

    time_text.set_text(f"Time: {time[frame - 1]:.1f} s")
    return target_line, quad_line, target_point, quad_point, time_text


# ===== PLOT VELOCITIES =====
velocity_figure, velocity_axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

velocity_axes[0].plot(time, vN_cmd, color="orange", label="Desired")
velocity_axes[0].plot(time, vN, color="blue", label="Actual")
velocity_axes[0].set_ylabel("North\n(m/s)")

velocity_axes[1].plot(time, vE_cmd, color="orange", label="Desired")
velocity_axes[1].plot(time, vE, color="blue", label="Actual")
velocity_axes[1].set_ylabel("East\n(m/s)")

velocity_axes[2].plot(time, vH_cmd, color="orange", label="Desired")
velocity_axes[2].plot(time, vH, color="blue", label="Actual")
velocity_axes[2].set_ylabel("Vertical\n(m/s)")
velocity_axes[2].set_xlabel("Time (s)")

for velocity_axis in velocity_axes:
    velocity_axis.grid(True)

velocity_axes[0].legend()
velocity_figure.suptitle("Desired and Actual Quadcopter Velocities")
velocity_figure.tight_layout()


# ===== SAVE PLOTS =====
update(len(time))
figure.savefig(folder / "controller_trajectory.png", dpi=160, bbox_inches="tight")
velocity_figure.savefig(folder / "controller_velocities.png", dpi=160, bbox_inches="tight")
init()

print("Saved controller_trajectory.png")
print("Saved controller_velocities.png")


# ===== ANIMATION =====
frames = np.unique(np.linspace(1, len(time), min(len(time), 360), dtype=int))
interval = 12000/len(frames)
save_animation = "--save-animation" in sys.argv

if save_animation or plt.get_backend().lower() != "agg":
    animation = FuncAnimation(
        figure,
        update,
        frames=frames,
        init_func=init,
        interval=interval,
        blit=False,
        repeat=False
    )

    if save_animation:
        animation.save(folder / "controller_trajectory.gif", writer="pillow", fps=30)
        print("Saved controller_trajectory.gif")

if plt.get_backend().lower() != "agg":
    plt.show()
