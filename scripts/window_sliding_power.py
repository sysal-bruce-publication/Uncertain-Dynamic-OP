"""
Window sliding power, remove power observations that are out of this time window.
"""
import json
import argparse


def remove_data_out_of_window(config_fname: str,
                              cared_duration: float) -> None:
    """
    Remove power observations that are out of this time window.
    :param config_fname:
    :type config_fname:
    :param cared_duration:
    :type cared_duration:
    :return:
    :rtype:
    """
    with open(config_fname, 'r') as rf:
        d = json.load(rf)
        power_log_fname = d["instance"]["power_log_fname"]

    parent_dir = config_fname.split('/')[0]
    with open(parent_dir + "/" + power_log_fname + ".csv", 'r') as rf:
        lines = rf.readlines()
        care_lines = lines
        while float(care_lines[-1].strip(",")[0]) - float(care_lines[-1].strip(",")[0]) > cared_duration:
            care_lines.pop(0)

    with open(parent_dir + "/" + power_log_fname + ".csv", 'w') as wf:
        for line in care_lines:
            wf.write(line)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        default="input/simulation_config.json",
        help="The configuration file name.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=50,
        help="The cared duration in seconds.",
    )

    args = parser.parse_args()
    remove_data_out_of_window(args.config, args.duration)
