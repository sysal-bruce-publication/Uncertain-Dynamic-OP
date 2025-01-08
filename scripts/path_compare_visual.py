"""
Plot the comparison between offline and online path
"""
import os.path
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def read_instances_data(inst_fname: str):
    """
    Read the data from the instance file.
    """
    xs, ys, zs, vs = [], [], [], []
    with open(inst_fname, 'r') as f:
        lines = f.readlines()
        for line in lines[1:]:  # First line is the header
            line = line.split(",")
            xs.append(float(line[0]))
            ys.append(float(line[1]))
            zs.append(float(line[2]))
            vs.append(float(line[3]))
    xs, ys, zs, vs = np.array(xs), np.array(ys), np.array(zs), np.array(vs)
    return xs, ys, zs, vs


def read_mission_log(log_fname: str):
    """
    Read the data from the mission log file.
    """
    xs, ys, zs = [], [], []
    ts_arrive, ts_leave = [], []
    es_arrive, es_leave, es_chrg = [], [], []
    with open(log_fname, 'r') as f:
        lines = f.readlines()[1:]
        for line in lines:
            line = line.split(",")
            xs.append(float(line[0]))
            ys.append(float(line[1]))
            zs.append(float(line[2]))
            ts_arrive.append(float(line[3]))
            es_arrive.append(float(line[4]))
            ts_leave.append(float(line[5]))
            es_leave.append(float(line[6]))
            es_chrg.append(float(line[7]))
    xs, ys, zs = np.array(xs), np.array(ys), np.array(zs)
    ts_arrive, ts_leave = np.array(ts_arrive), np.array(ts_leave)
    es_arrive, es_leave, es_chrg = np.array(es_arrive), np.array(es_leave), np.array(es_chrg)
    return xs, ys, zs, ts_arrive, ts_leave, es_arrive, es_leave, es_chrg


def read_solutions(sol_fname: str):
    """
    Read the data from the solution log file.
    """
    ps, cs, sols = [], [], []
    with open(sol_fname, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(",")
            ps.append(float(line[0]))
            cs.append(float(line[1]))
            sol = [int(_) for _ in line[2:]]
            sols.append(np.array(sol))
    ps, cs, sols = np.array(ps), np.array(cs), np.array(sols, dtype=object)
    return ps, cs, sols


def plot_compare_path(inst_fdir: str, mlog_fdir: str, sol_fdir: str,
                      out_fdir: str, axis_lim: float = 2000):
    """
    Plot the comparison between offline and online path.
    :param inst_fdir:
    :type inst_fdir:
    :param mlog_fdir:
    :type mlog_fdir:
    :param sol_fdir:
    :type sol_fdir:
    :param out_fdir:
    :type out_fdir:
    :param axis_lim:
    :type axis_lim:
    :return:
    :rtype:
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 17), dpi=300)
    fig.tight_layout(rect=[0.05, 0.05, 0.95, 0.95])
    plt.subplots_adjust(right=0.95)
    maker_size = 20
    tick_size = 15
    range_offset = 15

    sn_xs, sn_ys, sn_zs, sn_vs = read_instances_data(inst_fdir)
    sn_ps = np.array([8.82 - (10 * v**2 / 2000.) for v in sn_vs])
    sn_ps[0] = sn_ps[1] = 0
    sol_ps, sol_cs, sols = read_solutions(sol_fdir)
    offline_sols = sols[0]

    xs, ys, zs, ts_arrive, ts_leave, es_arrive, es_leave, es_chrg = read_mission_log(mlog_fdir)
    uav_x = xs[-1]
    uav_y = ys[-1]
    uav_time = ts_leave[-1]
    uav_engy = es_leave[-1]
    chrg_engy = np.sum(es_chrg)

    cmap = plt.cm.jet
    cmaplist = [cmap(i) for i in range(cmap.N)]
    cmaplist[0] = (.5, .5, .5, 1.0)
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        'Custom cmap', cmaplist, cmap.N)
    bounds = np.linspace(0, np.max(sn_ps), 7)
    norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
    cbar_ax = fig.add_axes([0.96, 0.13, 0.01, 0.72])
    cbar = matplotlib.colorbar.ColorbarBase(cbar_ax, cmap=cmap, norm=norm,
                                            spacing='proportional', ticks=bounds,
                                            boundaries=bounds, format='%1i')
    cbar.set_label('prize', fontsize=16)
    cbar.ax.tick_params(labelsize=16)

    ax2.scatter(sn_xs, sn_ys, marker='s', c=sn_ps,
                cmap=cmap, norm=norm, s=maker_size, zorder=1)

    for i in range(len(xs) - 1):
        xy1 = (xs[i], ys[i])
        xy2 = (xs[i + 1], ys[i + 1])
        con = patches.ConnectionPatch(xy2, xy1, 'data', 'data',
                                      arrowstyle='<-', mutation_scale=10,
                                      color='#0D4A70', alpha=0.6, zorder=2)
        ax2.add_patch(con)

    # ax.plot([-500, -500], [-400, -400], c='#0D4A70', label='Exp. path')
    ax2.set_xlabel('X [m]', fontsize=16)
    ax2.set_ylabel('Y [m]', fontsize=16)
    ax2_ticks = np.linspace(0, axis_lim, 8)
    ax2.set_xticks(ax2_ticks)
    ax2.set_yticks(ax2_ticks)
    ax2.set_xlim(-range_offset, axis_lim + range_offset)
    ax2.set_ylim(-range_offset, axis_lim + range_offset)
    ax2.tick_params(axis='both', which='major', labelsize=tick_size)
    ax2.set_title(f'Mission time: {uav_time:.2f} s, UAV battery {uav_engy:.2f} kJ, '
                  f'recharged {chrg_engy:.2f} kJ, exp. prize: {this_sol_p:.2f} kJ, '
                  f'exp. cost: {this_sol_c:.2f} kJ', fontsize=16)
    ax2.legend(loc='upper left', fontsize=16, framealpha=0.35)
    plt.savefig(out_fdir + "_path.png")
