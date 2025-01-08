"""
Modify the offline path (exp_chrg_list.csv) in the config file
"""
import argparse


def modify_offline_path(sol_fname: str):
    with open(sol_fname, 'r') as rf:
        lines = rf.readlines()
        lastline = lines[-1].split(',')
        lastline.pop(4)

    with open(sol_fname, 'a+') as af:
        lastline = ','.join(lastline)
        af.write(lastline)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--sol_fname', type=str,
                        default="../input/exp_chrg_list.csv",
                        help='solution file name')
    args = parser.parse_args()
    modify_offline_path(args.sol_fname)
