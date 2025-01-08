"""Reset the execution environment to initial status."""
import os
import argparse

def change_strategy(simu_config, strategy, online_config: str = None):
    if online_config is not None:
        with open(online_config, 'r') as rf:
            online_data = json.load(rf)
            online_data["online"]["strategy"]["name"] = strategy
        with open(online_config, 'w') as wf:
            json.dump(online_data, wf, indent=4)

    with open(simu_config, 'r') as rf:
        simu_data = json.load(rf)
        if strategy == "Bayesian" or strategy == "MCGreedy":
            simu_data["simulation"]["log_power"] = True
        else:
            simu_data["simulation"]["log_power"] = False
    with open(simu_config, 'w') as wf:
        json.dump(simu_data, wf, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--plog_fdir", type=str,
                        default="../input/avg_power_log.csv",
                        help="Average power log file directory")
    parser.add_argument("-s", "--chrg_fdir", type=str,
                        default="../input/exp_chrg_list.csv",
                        help="Charging list file directory")
    parser.add_argument("-m", "--mlog_fdir", type=str,
                        default="../input/mission_log.csv",
                        help="Mission log file directory")
    parser.add_argument("-i", "--inst_fdir", type=str,
                        default="../input/instance/california20_0.csv",
                        help="Instance file directory")
    args = parser.parse_args()

    with open(args.plog_fdir, "w") as wf:
        pass
    with open(args.chrg_fdir, "r") as rf:
        line0 = rf.readlines()[0]
    with open(args.chrg_fdir, "w") as wf:
        wf.write(line0)
    with open(args.mlog_fdir, "r") as rf:
        lines01 = rf.readlines()[:2]
    with open(args.mlog_fdir, "w") as wf:
        for line in lines01:
            wf.write(line)

    # Next remove the instance file except the passed one.
    inst_fdir = args.inst_fdir.split("/")
    inst_fdir = "/".join(inst_fdir[:-1])
    inst_fname = args.inst_fdir.split("/")[-1]
    for fname in os.listdir(inst_fdir):  # Remove all csv files but not directory.
        if fname.endswith(".csv") and fname != inst_fname:
            os.remove(os.path.join(inst_fdir, fname))
