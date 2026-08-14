import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Fuzzy set data extracted from Fuzzy.cpp
fuzzy_sets = {
    'Ultrasonic_FS': {
        'size': 4,
        'a': [0, 25, 60],
        'b': [45, 70, 100],
        'c': [70, 110, 150],
        'd': [100, 450, 1300],
        'e': [0, 0, 0]
    },
    'LeftFront_FS': {
        'size': 4,
        'a': [-1024, -510, -290],
        'b': [-400, -265, -220],
        'c': [-220, 0, 0],
        'd': [0, 0, 0],
        'e': [0, 0, 0]
    },
    'RightFront_FS': {
        'size': 4,
        'a': [-1024, -510, -290],
        'b': [-400, -265, -220],
        'c': [-220, 0, 0],
        'd': [0, 0, 0],
        'e': [0, 0, 0]
    },
    'Left_FS': {
        'size': 4,
        'a': [-1024, -730, -600],
        'b': [-680, -600, -525],
        'c': [-580, -500, -375],
        'd': [-475, -375, 0],
        'e': [0, 0, 0]
    },
    'Right_FS': {
        'size': 4,
        'a': [-1024, -730, -600],
        'b': [-680, -600, -525],
        'c': [-580, -500, -375],
        'd': [-475, -375, 0],
        'e': [0, 0, 0]
    },
    'Gyro_FS': {
        'size': 3,
        'a': [-180, -90, -0.2],
        'b': [-3, 0, 3],
        'c': [0.2, 90, 180],
        'd': [0, 0, 0],
        'e': [0, 0, 0]
    },
    'Correction_FS': {
        'size': 3,
        'a': [-1024, -1023, -50],
        'b': [-80, 0, 80],
        'c': [50, 1023, 1024],
        'd': [0, 0, 0],
        'e': [0, 0, 0]
    },
    'X_FS': {
        'size': 5,
        'a': [-1, -0.9, -0.7],
        'b': [-0.8, -0.5, -0.3],
        'c': [-0.4, 0, 0.4],
        'd': [0.3, 0.5, 0.8],
        'e': [0.7, 0.9, 1]
    },
    'Y_FS': {
        'size': 5,
        'a': [-1, -0.9, -0.6],
        'b': [-0.8, -0.5, -0.3],
        'c': [-0.4, 0, 0.4],
        'd': [0.3, 0.5, 0.8],
        'e': [0.6, 0.9, 1]
    },
    'W_FS': {
        'size': 5,
        'a': [-1, -0.8, -0.05],
        'b': [-0.6, -0.5, 0.6],
        'c': [-0.4, 0, 0.4],
        'd': [-0.6, 0.5, 0.6],
        'e': [0.05, 0.8, 1]
    }
}

def linz_fmf(x, p):
    """Left-open trapezoidal membership function (decreasing from 1 to 0)"""
    if x <= p[1]:
        return 1
    if x >= p[2]:
        return 0
    return (p[2] - x) / (p[2] - p[1])

def lins_fmf(x, p):
    """Right-open trapezoidal membership function (increasing from 0 to 1)"""
    if x <= p[0]:
        return 0
    if x >= p[1]:
        return 1
    return (x - p[0]) / (p[1] - p[0])

def tri_fmf(x, p):
    """Triangular membership function"""
    if x <= p[0] or x >= p[2]:
        return 0
    if x > p[0] and x <= p[1]:
        return (x - p[0]) / (p[1] - p[0])
    return (p[2] - x) / (p[2] - p[1])

def get_membership_function(fs, set_name, index):
    """Determine which membership function to use based on position"""
    size = fs['size']
    params = [fs['a'], fs['b'], fs['c'], fs['d'], fs['e']][index]
    
    # First set uses linz (left-open)
    if index == 0:
        return lambda x: linz_fmf(x, params)
    # Last set uses lins (right-open)
    elif index >= size - 1:
        return lambda x: lins_fmf(x, params)
    # Middle sets use triangular
    else:
        return lambda x: tri_fmf(x, params)

def plot_fuzzy_set(ax, fs, name):
    """Plot a single fuzzy set"""
    # Collect all parameter values to determine x range
    all_params = []
    for key in ['a', 'b', 'c', 'd', 'e']:
        if fs[key] != [0, 0, 0]:
            all_params.extend(fs[key])
    
    if not all_params:
        return
    
    min_val = min(all_params)
    max_val = max(all_params)
    
    # Add some padding
    padding = (max_val - min_val) * 0.1
    if padding == 0:
        padding = 1
    x_range = np.linspace(min_val - padding, max_val + padding, 500)
    
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    labels = ['A', 'B', 'C', 'D', 'E']
    
    for i in range(fs['size']):
        mf = get_membership_function(fs, name, i)
        y_values = [mf(x) for x in x_range]
        ax.plot(x_range, y_values, color=colors[i], label=labels[i], linewidth=2)
    
    ax.set_title(name, fontsize=10, fontweight='bold')
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)

# Create the plot
fig = plt.figure(figsize=(16, 12))
gs = GridSpec(4, 3, figure=fig, hspace=0.4, wspace=0.3)

# Plot all 10 fuzzy sets
axes = []
for i, (name, fs) in enumerate(fuzzy_sets.items()):
    row = i // 3
    col = i % 3
    ax = fig.add_subplot(gs[row, col])
    plot_fuzzy_set(ax, fs, name)
    axes.append(ax)

# Remove empty subplot (we have 10 sets in a 4x3 grid = 12 positions)
for i in range(10, 12):
    row = i // 3
    col = i % 3
    fig.add_subplot(gs[row, col]).axis('off')

plt.suptitle('Fuzzy Membership Functions', fontsize=16, fontweight='bold')
plt.savefig('fuzzy_membership_functions.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'fuzzy_membership_functions.png'")
plt.close()
