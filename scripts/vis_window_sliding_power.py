"""
Window sliding power, remove power observations that are out of this time window.
"""
import json
import argparse


def remove_data_out_of_window(config_fname: str,
                              power_log_fdir: str,
                              cared_duration: float) -> None:
    """
    Remove power observations that are out of this time window.
    """
    with open(config_fname, 'r') as rf:
        d = json.load(rf)
        dt = d["simulation"]["simu_dt"]

    with open(power_log_fdir, 'r') as rf:
        line = rf.readline().split(',')[:-1]
        if len(line) % 2 != 0:
            raise ValueError("The number of elements in the power log file is not even.")

        num_elems = (int(cared_duration / dt) + 1) * 2
        if len(line) > num_elems:
            line = line[-num_elems:]

    with open(power_log_fdir, 'w') as wf:
        wf.write(','.join(line))
        wf.write(',')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        default="../input/simulation_config.json",
        help="The configuration file name.",
    )
    parser.add_argument(
        "--plog_dir",
        type=str,
        default="../input/vis_avg_power_log.csv",
        help="The power log file name.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=50,
        help="The cared duration in seconds.",
    )

    args = parser.parse_args()
    remove_data_out_of_window(args.config,
                              args.plog_dir,
                              args.duration)
