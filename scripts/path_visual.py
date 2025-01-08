"""
Visualisation of the data in each iteration.
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
    return xs, ys, zs, ts_arrive, es_arrive, ts_leave, es_leave, es_chrg


def read_solutions(sol_fname: str):
    """
    Read the data from the solution log file.
    """
    ts, bs, ps, cs, sols = [], [], [], [], []
    with open(sol_fname, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(",")
            ts.append(float(line[0]))
            bs.append(float(line[1]))
            ps.append(float(line[2]))
            cs.append(float(line[3]))
            sol = [int(_) for _ in line[4:]]
            sols.append(np.array(sol))
    bs, ps, cs = np.array(bs), np.array(ps), np.array(cs)
    sols = np.array(sols, dtype=object)
    return bs, ps, cs, sols


def read_edge_costs(edge_cost_fname: str):
    """
    Read the edge costs from the file.
    """
    costs = []
    with open(edge_cost_fname, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(",")
            c = [float(_) for _ in line]
            costs.append(np.array(c))
    costs = np.array(costs, dtype=object)
    return costs


def plot_offline_path(inst_fdir: str, edge_cs_fdir: str, mlog_fdir: str,
                      sol_fdir: str, out_fdir: str, axis_lim: float = 1000):
    """
    Plot offline plan.
    """
    fig, ax = plt.subplots(1, 1, figsize=(17, 14), dpi=200)
    fig.tight_layout(rect=[0.05, 0.05, 0.95, 0.95])
    plt.subplots_adjust(right=0.95)
    maker_size = 20
    tick_size = 15
    range_offset = 20

    sn_xs, sn_ys, sn_zs, sn_vs = read_instances_data(inst_fdir)
    sn_ps = np.array([8.82 - (10 * v**2 / 2000.) for v in sn_vs])
    sn_ps[0] = sn_ps[1] = 0
    _, sol_ps, sol_cs, sols = read_solutions(sol_fdir)
    this_sol_p, this_sol_c, this_sol = sol_ps[0], sol_cs[0], sols[0]
    cs = read_edge_costs(edge_cs_fdir)
    sol_act_p = np.sum([sn_ps[this_sol[i]] for i in range(1, len(this_sol) - 1)])
    sol_visit_c = np.sum([cs[this_sol[i]][this_sol[i + 1]] for i in range(len(this_sol) - 1)])
    sol_service_c = this_sol_c - sol_visit_c if len(this_sol) > 2 else 0

    xs, ys, zs, ts_arrive, es_arrive, ts_leave, es_leave, es_chrg = read_mission_log(mlog_fdir)
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
    ax.scatter(uav_x, uav_y, marker='x', c='red', s=(maker_size * 2.5),
               zorder=3, label="UAV")
    ax.scatter(sn_xs, sn_ys, marker='s', c=sn_ps,
               cmap=cmap, norm=norm, s=maker_size, zorder=1)

    for i in range(len(this_sol) - 1):
        label = f'{cs[this_sol[i]][this_sol[i + 1]]:.2f} kJ'
        xy1 = (sn_xs[this_sol[i]], sn_ys[this_sol[i]])
        xy2 = (sn_xs[this_sol[i + 1]], sn_ys[this_sol[i + 1]])
        con = patches.ConnectionPatch(xy2, xy1, 'data', 'data',
                                      arrowstyle='<-', mutation_scale=10,
                                      color='#0D4A70', alpha=0.6, zorder=2)
        xy = np.mean((xy1, xy2), axis=0)
        ax.add_patch(con)
        ax.annotate(label, xy=xy, fontsize=10, va='center')
    for i in range(2, len(sn_xs)):
        if abs(sn_xs[0] - sn_xs[1]) < 1 and abs(sn_ys[0] - sn_ys[1]) < 1:
            ax.annotate('0', xy=(sn_xs[0], sn_ys[0] - range_offset + 3),
                        fontsize=9, va='center')
        ax.annotate(f'{i}', xy=(sn_xs[i], sn_ys[i] - range_offset + 3),
                    fontsize=9, va='center')
    ax.plot([-500, -500], [-400, -400], c='#0D4A70', label=f'Exp. path')
    ax.set_xlabel('X [m]', fontsize=16)
    ax.set_ylabel('Y [m]', fontsize=16)
    ax_ticks = np.linspace(0, axis_lim, 8)
    ax.set_xticks(ax_ticks)
    ax.set_yticks(ax_ticks)
    ax.set_xlim(-range_offset, axis_lim + range_offset)
    ax.set_ylim(-range_offset, axis_lim + range_offset)
    ax.tick_params(axis='both', which='major', labelsize=tick_size)

    ax.set_title(f'Mission time: {uav_time:.2f} s, '
                 f'UAV battery {uav_engy:.2f} kJ, '
                 f'recharged {chrg_engy:.2f} kJ\n'
                 f'Exp. prize: {sol_act_p:.2f} kJ, '
                 f'exp. visit cost: {sol_visit_c:.2f} kJ, '
                 f'exp. service cost: {sol_service_c:.2f} kJ', fontsize=16)
    ax.legend(loc='upper left', fontsize=15, framealpha=0.35)
    plt.savefig(out_fdir + "_path.png")


def plot_online_path(inst_fdir: str, edge_cs_fdir: str, mlog_fdir: str,
                     sol_fdir: str, out_fdir: str, bayesian_fdir: str,
                     new_edge_cs_fdir: str, axis_lim: float = 1000):
    """
    Plot all the data.
    """
    fig, ax = plt.subplots(1, 1, figsize=(17, 14), dpi=200)
    fig.tight_layout(rect=[0.05, 0.05, 0.95, 0.95])
    plt.subplots_adjust(right=0.95)
    maker_size = 20
    tick_size = 15
    range_offset = 20

    sn_xs, sn_ys, sn_zs, sn_vs = read_instances_data(inst_fdir)
    sn_ps = np.array([8.82 - (10 * v**2 / 2000.) for v in sn_vs])
    sn_ps[0] = sn_ps[1] = 0
    sol_bs, sol_ps, sol_cs, sols = read_solutions(sol_fdir)
    this_sol_b, this_sol_p, this_sol_c = sol_bs[-1], sol_ps[-1], sol_cs[-1]
    this_sol = sols[-1]
    iter_num = int(new_edge_cs_fdir.split("/")[-1].split("_")[-1].split(".")[0])
    if iter_num == 1:
        cs = read_edge_costs(edge_cs_fdir)
    else:
        prev_edge_cs_fdir = new_edge_cs_fdir.replace(f"_{iter_num}", f"_{iter_num - 1}")
        cs = read_edge_costs(prev_edge_cs_fdir)
    sol_visit_c = np.sum([cs[this_sol[i]][this_sol[i + 1]] for i in range(len(this_sol) - 1)])
    sol_service_c = this_sol_c - sol_visit_c if len(this_sol) > 2 else 0

    xs, ys, zs, ts_arrive, es_arrive, ts_leave, es_leave, es_chrg = read_mission_log(mlog_fdir)
    uav_x, uav_y = xs[-1], ys[-1]
    chrg_engy = np.sum(es_chrg[:-1])  # Not charged yet, update this later

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

    ax.scatter(uav_x, uav_y, marker='x', c='red', s=(maker_size * 2.5),
               zorder=3, label="UAV")
    ax.scatter(sn_xs, sn_ys, marker='s', c=sn_ps,
               cmap=cmap, norm=norm, s=maker_size, zorder=1)

    for i in range(2, len(sn_xs)):
        if abs(sn_xs[0] - sn_xs[1]) < 1 and abs(sn_ys[0] - sn_ys[1]) < 1:
            ax.annotate('0', xy=(sn_xs[0], sn_ys[0] - range_offset + 3),
                        fontsize=9, va='center')
        ax.annotate(f'{i}', xy=(sn_xs[i], sn_ys[i] - range_offset + 3),
                    fontsize=9, va='center')
    ax.plot([-500, -500], [-400, -400], c='#0D4A70', label='Exp. path')

    annos = []
    for i in range(len(this_sol) - 1):
        label = f'{cs[this_sol[i]][this_sol[i+1]]:.2f} kJ'
        xy1 = (sn_xs[this_sol[i]], sn_ys[this_sol[i]])
        xy2 = (sn_xs[this_sol[i + 1]], sn_ys[this_sol[i + 1]])
        con = patches.ConnectionPatch(xy2, xy1, 'data', 'data',
                                      arrowstyle='<-', mutation_scale=10,
                                      color='#0D4A70', alpha=0.6, zorder=2)
        xy = np.mean((xy1, xy2), axis=0)
        ax.add_patch(con)
        ann = ax.annotate(label, xy=xy, fontsize=10, va='center')
        annos.append(ann)

    ax.set_xlabel('X [m]', fontsize=16)
    ax.set_ylabel('Y [m]', fontsize=16)
    ax_ticks = np.linspace(0, axis_lim, 8)
    ax.set_xticks(ax_ticks)
    ax.set_yticks(ax_ticks)
    ax.set_xlim(-range_offset, axis_lim + range_offset)
    ax.set_ylim(-range_offset, axis_lim + range_offset)
    ax.tick_params(axis='both', which='major', labelsize=tick_size)
    if this_sol_b > 0:
        ax.set_title(f'Mission time: {ts_arrive[-1]:.2f} s, '
                     f'UAV battery {es_arrive[-1]:.2f} kJ, '
                     f'recharged {chrg_engy:.2f} kJ\n'
                     f'Safety belief: {this_sol_b} %, '
                     f'exp. prize: {this_sol_p:.2f} kJ, '
                     f'exp. visit cost: {sol_visit_c:.2f} kJ, '
                     f'exp. service cost: {sol_service_c:.2f} kJ', fontsize=16)
    else:
        ax.set_title(f'Mission time: {ts_arrive[-1]:.2f} s, '
                     f'UAV battery {es_arrive[-1]:.2f} kJ, '
                     f'recharged {chrg_engy:.2f} kJ\n'
                     f'Exp. prize: {this_sol_p:.2f} kJ, '
                     f'exp. visit cost: {sol_visit_c:.2f} kJ, '
                     f'exp. service cost: {sol_service_c:.2f} kJ', fontsize=16)
    ax.legend(loc='upper left', fontsize=15, framealpha=0.35)
    # plt.savefig(out_fdir + "_path.png")

    if bayesian_fdir is not None:
        im = plt.imread(bayesian_fdir)
        im_ax = fig.add_axes([0.3, 0.47, 0.45, 0.45], anchor='N', zorder=2)
        im_ax.axis('off')
        im_ax.imshow(im, alpha=0.6)
        plt.savefig(out_fdir + "_path_bay.png")

    if os.path.isfile(new_edge_cs_fdir):
        for ann in annos:
            ann.remove()
        cs = read_edge_costs(new_edge_cs_fdir)
        for i in range(len(this_sol) - 1):
            label = f'{cs[this_sol[i]][this_sol[i + 1]]:.2f} kJ'
            xy1 = (sn_xs[this_sol[i]], sn_ys[this_sol[i]])
            xy2 = (sn_xs[this_sol[i + 1]], sn_ys[this_sol[i + 1]])
            xy = np.mean((xy1, xy2), axis=0)
            ax.annotate(label, xy=xy, fontsize=10, va='center', color='red')
        if this_sol_b > 0:
            ax.set_title(f'Mission time: {ts_leave[-1]:.2f} s, '
                         f'UAV battery {es_leave[-1]:.2f} kJ, '
                         f'recharged {chrg_engy + es_chrg[-1]:.2f} kJ\n'
                         f'Safety belief: {this_sol_b} %, '
                         f'Exp. prize: {this_sol_p:.2f} kJ, '
                         f'exp. visit cost: {sol_visit_c:.2f} kJ, '
                         f'exp. service cost: {sol_service_c:.2f} kJ', fontsize=16)
        else:
            ax.set_title(f'Mission time: {ts_leave[-1]:.2f} s, '
                         f'UAV battery {es_leave[-1]:.2f} kJ, '
                         f'recharged {chrg_engy + es_chrg[-1]:.2f} kJ\n'
                         f'Exp. prize: {this_sol_p:.2f} kJ, '
                         f'exp. visit cost: {sol_visit_c:.2f} kJ, '
                         f'exp. service cost: {sol_service_c:.2f} kJ', fontsize=16)
        plt.savefig(out_fdir + "_path_new.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--inst_fdir", type=str,
                        default="../input/instance/california20_1.csv",
                        help="The instance file.")
    parser.add_argument("-e", "--edge_fdir", type=str,
                        default="../input/offline_edge_costs.csv",
                        help="The offline edge cost file.")
    parser.add_argument("-m", "--mlog_fdir", type=str,
                        default="../input/mission_log.csv",
                        help="The mission log file.")
    parser.add_argument("-s", "--sol_fdir", type=str,
                        default="../input/exp_chrg_list.csv",
                        help="The solution file.")
    parser.add_argument("-b", "--bay_fdir", type=str,
                        default=None, help="The Bayesian plot file.")
    parser.add_argument("-o", "--out_fdir", type=str,
                        default="../output/vis_1",
                        help="The full director of the plot, but no suffix.")
    parser.add_argument("-n", "--new_edge_fdir", type=str,
                        default=None, help="The new edge cost file.")
    args = parser.parse_args()

    if args.new_edge_fdir is None:
        plot_offline_path(inst_fdir=args.inst_fdir,
                          edge_cs_fdir=args.edge_fdir,
                          mlog_fdir=args.mlog_fdir,
                          sol_fdir=args.sol_fdir,
                          out_fdir=args.out_fdir)
    else:
        plot_online_path(inst_fdir=args.inst_fdir,
                         edge_cs_fdir=args.edge_fdir,
                         mlog_fdir=args.mlog_fdir,
                         sol_fdir=args.sol_fdir,
                         out_fdir=args.out_fdir,
                         bayesian_fdir=args.bay_fdir,
                         new_edge_cs_fdir=args.new_edge_fdir)
