import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

def alpha(e, L, k=1):
    """
    Raised cosine from 0 to L.

    alpha(0) = 0
    alpha(L) = 1
    alpha(e>L) = 1
    """
    z = np.clip(e / L, 0.0, 1.0)
    return 0.5 * (1 - np.cos(np.pi * (z**k)))


# Parameters
L = 0.05
U = -0.06
k = 1

# Error axis
e = np.linspace(-0.2, 0.2, 1000)

# Activation
a = alpha(e, L, k)


def force_gain(e,initial_value, final_value, L=0.0, U=1.0, k=0.8):
    """
    Specular force gain following stiffness profile.
    e=0 -> 1
    e=1 -> final_value
    """
    a = alpha(e, L, k)

    return initial_value + a * (final_value - initial_value)


# Force gain parameters
final_value = 0.12
initial_value = 0.9

# Compute force gain using the same stiffness profile
g = force_gain(e,initial_value, final_value, L, U, k)




# Figure
fig, ax = plt.subplots(figsize=(9, 5.5))


# Background regions (subtle)
ax.axvspan(
    e.min(), L,
    color="#cb997e",
    alpha=0.2,
    label=r"$\alpha=0$ region"
)

ax.axvspan(
    L, U,
    color="#ddbea9",
    alpha=0.2,
    label="Transition region"
)

ax.axvspan(
    U, e.max(),
    color="#ffe8d6",
    alpha=0.2,
    label=r"$\alpha=1$ region"
)


# Activation curve
# Stiffness activation
ax.plot(
    e,
    a,
    linewidth=3.0,
    linestyle="-",
    color="#1f77b4",
    label=r"$\alpha(e)$"
)

# Wrench gain
ax.plot(
    e,
    g,
    linewidth=3.0,
    linestyle="--",
    color="#d62728",
    label=rf"Wrench gain ($G_{{max}}={final_value}$)"
)


# Vertical boundaries
ax.axvline(
    L,
    color="black",
    linestyle=":",
    linewidth=1.0
)

ax.axvline(
    U,
    color="black",
    linestyle=":",
    linewidth=1.0
)


# Add x-axis values near the lines
ax.text(
    L,
    -0.08,
    rf"$L={L:.3f}$",
    ha="center",
    va="top",
    fontsize=10,
    color="red",
    transform=ax.get_xaxis_transform()
)

ax.text(
    U,
    -0.08,
    rf"$U={U:.3f}$",
    ha="center",
    va="top",
    fontsize=10,
    color="green",
    transform=ax.get_xaxis_transform()
)





# Labels
ax.set_xlabel("Normalized force error $e$", fontsize=20)
ax.set_ylabel(r"$\alpha(e)$ / Wrench gain", fontsize=20)




# Limits
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(0.0, 1.05)

# Hide the default spines
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.spines["bottom"].set_visible(False)
ax.spines["left"].set_visible(False)

# Remove spines
for spine in ax.spines.values():
    spine.set_visible(False)

# X-axis (full width)
x_arrow = FancyArrowPatch(
    (-0.2, 0), (0.2, 0),
    arrowstyle='-|>',
    mutation_scale=15,
    linewidth=1,
    color='black',
    clip_on=False,
    zorder=100,
)
ax.add_patch(x_arrow)

# Y-axis (0 to 1)
y_arrow = FancyArrowPatch(
    (0, 0), (0, 1.02),
    arrowstyle='-|>',
    mutation_scale=15,
    linewidth=1,
    color='black',
    clip_on=False,
    zorder=100,
)
ax.add_patch(y_arrow)
# Keep ticks
ax.xaxis.set_ticks_position("bottom")
ax.yaxis.set_ticks_position("left")


# Grid and legend
ax.grid(False)

ax.legend(
    loc="center right",
    fontsize=10
)


plt.tight_layout()
plt.show()