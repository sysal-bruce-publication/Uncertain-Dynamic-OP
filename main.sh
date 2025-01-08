#!/usr/bin/env bash

#### Below parameters should be modified according to the experiment. ####
declare -a INSTANCES=("california10" "california20" "california30" "california40")
declare -a PWR_MEAN_OFFSETS=(90 110) # (90 100 110 120)
declare -a PWR_STD_OFFSETS=(110 120) # (90 100 110 120)
START_EXE=0
MAX_EXES=2
MAX_ITER=20
##########################################################################

INPUT_DIR="input"
RESULT_DIR="results"
############## Check below files before running this script. ##############
INIT_UAV_CONFIG_DIR="${INPUT_DIR}/uav0_status.json"
MISSION_LOG_DIR="${INPUT_DIR}/mission_log.csv"
OFFLINE_CONFIG_FILE="offline_config.json"
ONLINE_CONFIG_FILE="online_config.json"
SIMULATION_CONFIG_FILE="simulation_config.json"
REAL_PWR_LOG_DIR="${INPUT_DIR}/real_power_config.json"
##########################################################################
OFFLINE_CONFIG_DIR="${INPUT_DIR}/${OFFLINE_CONFIG_FILE}"
SIMULATION_CONFIG_DIR="${INPUT_DIR}/${SIMULATION_CONFIG_FILE}"
ONLINE_CONFIG_DIR="${INPUT_DIR}/${ONLINE_CONFIG_FILE}"
SOL_DIR="${INPUT_DIR}/exp_chrg_list.csv"
AVG_POWER_LOG_DIR="${INPUT_DIR}/avg_power_log.csv"
TIME_WINDOW=900  # Latest 15 minutes to record power consumption.
#OUTPUT_DIR="output"
#EDGE_COSTS_FILE="${INPUT_DIR}/offline_edge_costs.csv"


for INSTANCE in "${INSTANCES[@]}"; do
	INIT_INSTANCE_FDIR="${INPUT_DIR}/instance/${INSTANCE}_0.csv"
	for PWR_MEAN in "${PWR_MEAN_OFFSETS[@]}"; do
		for PWR_STD in "${PWR_STD_OFFSETS[@]}"; do
			exe_num=0
			while [ $exe_num -ne $MAX_EXES ]; do
				iter_num=0
				INSTANCE_FNAME="${INSTANCE}_${iter_num}"
				echo "[INFO] == ${INSTANCE} == MEAN ${PWR_MEAN} == STD ${PWR_STD} == ${exe_num} =="
				python scripts/init_env.py -s $SIMULATION_CONFIG_DIR -on $ONLINE_CONFIG_DIR -off $OFFLINE_CONFIG_DIR -i "${INSTANCE_FNAME}" -u $INIT_UAV_CONFIG_DIR -m $MISSION_LOG_DIR -p $REAL_PWR_LOG_DIR --mean "${PWR_MEAN}" --std "${PWR_STD}"
				cp "${INPUT_DIR}/instance/metadata/${INSTANCE_FNAME}.csv" "${INPUT_DIR}/instance/${INSTANCE_FNAME}.csv"
				src/offline-planning/offline.exe $OFFLINE_CONFIG_FILE
				exe_status=$?
				if [ $exe_status -ne 0 ]; then
					echo "[ERROR] Failed to generate offline charging list."
					exit 1
				fi
				python scripts/correct_instance_id.py --config $SIMULATION_CONFIG_DIR --id $iter_num
				src/simulation/x64/Debug/simulation.exe $SIMULATION_CONFIG_FILE 1
				EXPERIMENT_DIR="${RESULT_DIR}/${INSTANCE}/mean${PWR_MEAN}_std${PWR_STD}/${exe_num}"
				echo "[INFO] Initialization done."

				#################################### OFFLINE ####################################
				python scripts/change_online_strategy.py -sim $SIMULATION_CONFIG_DIR -stg "Offline"
				while true; do
					python scripts/correct_instance_id.py --config $SIMULATION_CONFIG_DIR --id $iter_num
					src/simulation/sim.exe $SIMULATION_CONFIG_FILE 0
					exe_status=$?
					if [ $exe_status -ne 0 ]; then
						break
					fi
					python scripts/offline_path_delete_one_visit.py -s $SOL_DIR
					iter_num=$((iter_num+1))
					python scripts/correct_instance_id.py --config $ONLINE_CONFIG_DIR --id $iter_num
					if [ $iter_num -eq $MAX_ITER ]; then
						echo "[WARN] Out of maximam allowed number of iterations."
						break
					fi
				done
				STGY_NAME="Offline"
				TARGET_DIR="${EXPERIMENT_DIR}/${STGY_NAME}"
				if [ ! -d "${TARGET_DIR}" ]; then
					mkdir -p "${TARGET_DIR}"
				fi
				mv output/* "${TARGET_DIR}"
				cp input/exp_chrg_list.csv "${TARGET_DIR}"
				cp input/mission_log.csv "${TARGET_DIR}"
				if [ ! -d "${EXPERIMENT_DIR}/${INSTANCE}_0.csv" ]; then
					cp input/instance/*.csv "${EXPERIMENT_DIR}"
					cp input/offline_edge_costs.csv "${EXPERIMENT_DIR}"
					cp input/real_edge_costs.csv "${EXPERIMENT_DIR}"
				fi
				echo "[INFO] ${STGY_NAME} results saved to ${TARGET_DIR}."

				#################################### ONLINE_ROMP ####################################
				python scripts/reset_to_init_env.py -p $AVG_POWER_LOG_DIR -s $SOL_DIR -m $MISSION_LOG_DIR -i "${INIT_INSTANCE_FDIR}"
				python scripts/change_online_strategy.py -sim $SIMULATION_CONFIG_DIR -onl $ONLINE_CONFIG_DIR -stg "DoNothing"
				iter_num=0
				while true; do
					python scripts/correct_instance_id.py --config $SIMULATION_CONFIG_DIR --id $iter_num
					src/simulation/sim.exe $SIMULATION_CONFIG_FILE 0
					exe_status=$?
					if [ $exe_status -ne 0 ]; then
						break
					fi
					iter_num=$((iter_num+1))
					python scripts/correct_instance_id.py --config $ONLINE_CONFIG_DIR --id $iter_num
					src/online-planning/online.exe $ONLINE_CONFIG_FILE
					if [ $iter_num -eq $MAX_ITER ]; then
						echo "[WARN] Out of maximam allowed number of iterations."
						break
					fi
				done
				STGY_NAME="Online_DoNothing"
				TARGET_DIR="${EXPERIMENT_DIR}/${STGY_NAME}"
				if [ ! -d "${TARGET_DIR}" ]; then
					mkdir -p "${TARGET_DIR}"
				fi
				cp input/avg_power_log.csv "${TARGET_DIR}"
				cp input/exp_chrg_list.csv "${TARGET_DIR}"
				cp input/mission_log.csv "${TARGET_DIR}"
				echo "[INFO] ${STGY_NAME} results saved to ${TARGET_DIR}."

				#################################### ONLINE_WeightedErr ####################################
				python scripts/reset_to_init_env.py -p $AVG_POWER_LOG_DIR -s $SOL_DIR -m $MISSION_LOG_DIR -i "${INIT_INSTANCE_FDIR}"
				python scripts/change_online_strategy.py -sim $SIMULATION_CONFIG_DIR -onl $ONLINE_CONFIG_DIR -stg "WeightedAvg"
				iter_num=0
				while true; do
					python scripts/correct_instance_id.py --config $SIMULATION_CONFIG_DIR --id $iter_num
					src/simulation/sim.exe $SIMULATION_CONFIG_FILE 0
					exe_status=$?
					if [ $exe_status -ne 0 ]; then
						break
					fi
					iter_num=$((iter_num+1))
					python scripts/correct_instance_id.py --config $ONLINE_CONFIG_DIR --id $iter_num
					src/online-planning/online.exe $ONLINE_CONFIG_FILE
					if [ $iter_num -eq $MAX_ITER ]; then
						echo "[WARN] Out of maximam allowed number of iterations."
						break
					fi
				done
				STGY_NAME="Online_WeightedAvg"
				TARGET_DIR="${EXPERIMENT_DIR}/${STGY_NAME}"
				if [ ! -d "${TARGET_DIR}" ]; then
					mkdir -p "${TARGET_DIR}"
				fi
				mv output/* "${TARGET_DIR}"
				cp input/avg_power_log.csv "${TARGET_DIR}"
				cp input/exp_chrg_list.csv "${TARGET_DIR}"
				cp input/mission_log.csv "${TARGET_DIR}"
				echo "[INFO] ${STGY_NAME} results saved to ${TARGET_DIR}."

				#################################### ONLINE_BAYESIAN ####################################
				python scripts/reset_to_init_env.py -p $AVG_POWER_LOG_DIR -s $SOL_DIR -m $MISSION_LOG_DIR -i "${INIT_INSTANCE_FDIR}"
				python scripts/change_online_strategy.py -sim $SIMULATION_CONFIG_DIR -onl $ONLINE_CONFIG_DIR -stg "Bayesian"
				iter_num=0
				while true; do
					python scripts/correct_instance_id.py --config $SIMULATION_CONFIG_DIR --id $iter_num
					src/simulation/sim.exe $SIMULATION_CONFIG_FILE 0
					exe_status=$?
					if [ $exe_status -ne 0 ]; then
						break
					fi
					python scripts/window_sliding_power.py --config $SIMULATION_CONFIG_DIR --duration $TIME_WINDOW
					iter_num=$((iter_num+1))
					python scripts/correct_instance_id.py --config $ONLINE_CONFIG_DIR --id $iter_num
					src/online-planning/online.exe $ONLINE_CONFIG_FILE
					if [ $iter_num -eq $MAX_ITER ]; then
						echo "[WARN] Out of maximam allowed number of iterations."
						break
					fi
				done
				STGY_NAME="Online_Bayesian"
				TARGET_DIR="${EXPERIMENT_DIR}/${STGY_NAME}"
				if [ ! -d "${TARGET_DIR}" ]; then
					mkdir -p "${TARGET_DIR}"
				fi
				mv output/* "${TARGET_DIR}"
				cp input/avg_power_log.csv "${TARGET_DIR}"
				cp input/exp_chrg_list.csv "${TARGET_DIR}"
				cp input/mission_log.csv "${TARGET_DIR}"
				echo "[INFO] ${STGY_NAME} results saved to ${TARGET_DIR}."

				exe_num=$((exe_num+1))
			done
		done
	done
done
