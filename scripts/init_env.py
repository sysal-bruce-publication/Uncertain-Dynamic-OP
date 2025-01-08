"""
Initialize the environment
"""
import os
import argparse
import json


def clear_environment():
    """
    Clear the input and output folders.
    """
    os.system("rm -rf input/*.csv")
    os.system("rm -rf input/instance/*.csv")
    os.system("rm -rf output/*")


def set_init_uav_status(uav_config_fname: str, mlog_fname: str):
    """
    Set the initial UAV status for the mission planning problem.
    """
    with open(uav_config_fname) as json_data:
        d = json.load(json_data)
        uav_E = d["drone"]["E"]
        uav_t = d["drone"]["t"]
        uav_x = d["drone"]["x"]
        uav_y = d["drone"]["y"]
        uav_z = d["drone"]["z"]

    with open(mlog_fname, 'w') as wf:
        wf.write("x,y,z,t_arrive,E_arrive,t_leave,E_leave,E_chrg\n")
        wf.write(f"{uav_x},{uav_y},{uav_z},0,0,{uav_t},{uav_E},0\n")


def unify_instance(config_fdir: str, inst_name: str):
    """
    Unify the instance name in configuration json file.
    :param config_fdir:
    :type config_fdir:
    :param inst_name:
    :type inst_name:
    :return:
    :rtype:
    """
    with open(config_fdir, 'r') as file:
        config = json.load(file)
    config['instance']["sensor_fname"] = inst_name
    with open(config_fdir, 'w') as file:
        json.dump(config, file, indent=4)


def save_real_power_params(pconfig_dir: str, sconfig_dir: str, mean, std):
    params = {}
    with open(pconfig_dir, 'r') as f:
        data = json.load(f)
        for i in range(100):
            this_key = "config" + str(i)
            if this_key not in data:
                break
            mean_diff = abs(data[this_key]['mean_offset'] - mean)
            std_diff = abs(data[this_key]['std_offset'] - std)
            if mean_diff <= 1e-4 and std_diff <= 1e-4:
                for key, value in data[this_key].items():
                    params[key] = value
                break
    if not params:
        raise ValueError(f"{mean}, {std}, no such configuration found")

    with open(sconfig_dir, 'r') as rf:
        data = json.load(rf)
    data["simulation"]["normal"] = params
    with open(sconfig_dir, 'w') as wf:
        json.dump(data, wf, indent=4)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--inst_name", help="Instance file name")
    parser.add_argument("-u", "--uav0_config_fdir", type=str,
                        default="input/uav0_status.json")
    parser.add_argument("-m", "--mlog", type=str,
                        default="input/mission_log.csv")
    parser.add_argument("-s", "--simu_config_fdir", type=str,
                        default=None, help="Simulation configuration file directroy")
    parser.add_argument("-on", "--on_config_fdir", type=str,
                        default=None, help="Online configuration file directroy")
    parser.add_argument("-off", "--off_config_fdir", type=str,
                        default=None, help="Offline configuration file directroy")
    parser.add_argument("-p", "--pwr_config_dir", type=str,
                        default="input/real_power_config.json")
    parser.add_argument('--mean', type=float, default=100,
                        help="Mean offset for real power distribution in %")
    parser.add_argument('--std', type=float, default=100,
                        help="Standard deviation offset for real power distribution in %")
    args = parser.parse_args()

    clear_environment()
    set_init_uav_status(args.uav0_config_fdir, args.mlog)
    if args.off_config_fdir:
        unify_instance(args.off_config_fdir, args.inst_name)
    if args.on_config_fdir:
        unify_instance(args.on_config_fdir, args.inst_name)
    if args.simu_config_fdir:
        unify_instance(args.simu_config_fdir, args.inst_name)

    save_real_power_params(args.pwr_config_dir,
                           args.simu_config_fdir, args.mean, args.std)

