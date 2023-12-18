import numpy as np
import matplotlib.pyplot as plt

trajectories = np.loadtxt("t_trajectories.csv", delimiter=",")
fig, ax = plt.subplots()
time_interval = 170
colormap = plt.get_cmap("summer")
num_colors = colormap.N
time_steps = 600

# First, plot all trajectories
for i in range(0, trajectories.shape[1], 2):
    x = trajectories[:, i]
    y = trajectories[:, i + 1]
    for j in range(len(x) - 1):
        color_idx = int(np.ceil(j / time_steps * num_colors)) % num_colors
        color = colormap(color_idx / num_colors)
        ax.plot(x[j : j + 2], y[j : j + 2], color=color, linewidth=1)

for i in range(0, trajectories.shape[1], 2):
    x = trajectories[:, i]
    y = trajectories[:, i + 1]
    ax.scatter(x[0], y[0], s=10, zorder=2)

    for t in range(time_interval, len(x), time_interval):
        ax.scatter(x[t], y[t], s=4, zorder=2)
        if t == 50:
            ax.scatter(x[t], y[t], s=4, zorder=2)
        if t == 510:
            circle_radius = 20
            circle_radius_2 = 7
            ax.scatter(
                x[t],
                y[t],
                s=np.pi * (circle_radius**2),
                color="silver",
                alpha=0.2,
                zorder=1,
            )
            ax.scatter(
                x[t],
                y[t],
                s=np.pi * (circle_radius_2**2),
                color="lightgreen",
                alpha=0.2,
                zorder=1,
            )

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.spines[["right", "top"]].set_visible(False)
mappable = plt.cm.ScalarMappable(cmap=colormap)
mappable.set_array(range(0, time_steps))
plt.colorbar(mappable, ax=ax).set_label("Time Steps")
plt.tight_layout()
plt.grid(False)
plt.savefig("cc_flocking.png", dpi=600)
plt.show()
