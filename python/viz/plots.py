import matplotlib.pyplot as plt

def plot_states(t, y, title_prefix="5-DOF model"):
    fig = plt.figure(figsize=(14,8))
    namesQ = ['X','Y','theta','z_uf','z_ur']
    namesV = ['Xdot','Ydot','thetadot','z_uf dot','z_ur dot']
    for i in range(5):
        ax = fig.add_subplot(2,5,i+1)
        ax.plot(t, y[:,i], linewidth=1.4)
        ax.grid(True); ax.set_xlabel('t [s]'); ax.set_ylabel(namesQ[i])
    for i in range(5):
        ax = fig.add_subplot(2,5,5+i+1)
        ax.plot(t, y[:,5+i], linewidth=1.4)
        ax.grid(True); ax.set_xlabel('t [s]'); ax.set_ylabel(namesV[i])
    fig.suptitle(f'{title_prefix}', fontweight='bold')
    fig.tight_layout()
    return fig
