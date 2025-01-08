"""Change the online strategy in the online configuration file."""
import json
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
        if strategy == "Bayesian":
            simu_data["simulation"]["log_power"] = True
            simu_data["simulation"]["use_avg_power"] = True
        elif strategy == "MCGreedy":
            simu_data["simulation"]["log_power"] = True
            simu_data["simulation"]["use_avg_power"] = False
        else:
            simu_data["simulation"]["log_power"] = False

    with open(simu_config, 'w') as wf:
        json.dump(simu_data, wf, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-sim", "--simu_config", type=str,
                        help="Simulation configuration file.")
    parser.add_argument("-onl", "--online_config", type=str,
                        default=None,
                        help="Online configuration file.")
    parser.add_argument("-stg", "--strategy", type=str,
                        help="Online strategy.")
    args = parser.parse_args()

    change_strategy(args.simu_config, args.strategy, args.online_config)
