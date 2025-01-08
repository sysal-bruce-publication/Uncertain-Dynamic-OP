"""
Set the correct instance ID after manual simulation so that the planner
knows which iteration. Usage: (at root)

python scripts/correct_instance_id.py --id_now 1
"""
import json
import argparse


def revise_instance_id(config_fdir: str, id_now: int) -> None:
    """
    Set the correct instance ID after manual simulation so that the planner
    knows which iteration.
    :param config_fdir:
    :type config_fdir:
    :param id_now:
    :type id_now:
    :return:
    :rtype:
    """
    with open(config_fdir, 'r') as rf:
        d = json.load(rf)
        last_fname = d["instance"]["sensor_fname"].split("_")[0]
    # Now modify the instance file name in json.
    d["instance"]["sensor_fname"] = last_fname + "_" + str(id_now)
    with open(config_fdir, 'w') as wf:
        json.dump(d, wf, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str,
                        default="input/simulation_config.json",
                        help="The configuration file name.")
    parser.add_argument("--id", type=int,
                        default=0, help="Current instance ID.")
    args = parser.parse_args()
    revise_instance_id(args.config, args.id)
