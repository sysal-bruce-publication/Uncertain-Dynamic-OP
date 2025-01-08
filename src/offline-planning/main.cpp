/*****************************************************************//**
 * \file   main.cpp
 * \brief  
 * 
 * \author Qiuchen Qian
 * \date   March 2024
 *********************************************************************/
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include "json.hpp"
#include "sys_timer.h"
#include "acs.h"

using std::cout, std::cerr;
using std::runtime_error;
using std::unique_ptr, std::make_unique;
using std::stringstream, std::string;
using std::to_string, std::stoi, std::stod, std::stoul;
using std::ifstream, std::getline, std::ofstream;
using std::vector;
namespace fs = std::filesystem;

//! Global variables for later usage.
fs::path root_dir        = fs::current_path();		// Main root
fs::path input_fdir      = root_dir / "input";		// Input root 
fs::path output_fdir     = root_dir / "output";		// Output root
fs::path inst_fdir       = input_fdir / "instance"; // Sensor nodes
string inst_fname        = "x";					// Sensor nodes file name
string chrg_fname        = "x";					// Charging list file name
string mission_log_fname = "x";					// Log name of mission
string power_log_fname   = "x";					// Log name of average power
string edge_fname        = "x";					// Edge cost matrix file name
bool print_result        = false;				// Print result flag
bool save_est_sensors    = false;				// Save estimated sensors flag
size_t num_ants          = 0;					// Number of ants in ACS
size_t max_no_impr       = 0;					// Max number of no improvement
size_t num_iters_acs     = 0;					// Number of iterations in ACS
size_t num_iters_2opt    = 0;					// Number of iterations in 2-opt
double q0                = 0;					// ACS hyper-parameter q0
double rho               = 0;					// ACS pheromone evaporation rate
double alpha             = 0;					// ACS hyper-parameter alpha
double beta              = 0;					// ACS hyper-parameter beta
double tol_acs           = 0;					// ACS no improvement threshold
double uav_dt            = 0;					// UAV power update delta time [s]

namespace DataIO
{
	using std::this_thread::sleep_for;

	constexpr auto COMMA_CHAR = ',';
	constexpr auto FILE_TYPE  = ".csv";
	constexpr size_t MAX_ATTEMPTS = 3;
	constexpr std::chrono::milliseconds RETRY_DELAY(100);

	static vector<string> split(const string& s, char delim)
	{
		vector<string> result;
		stringstream ss(s);
		string item;

		while (getline(ss, item, delim)) { result.push_back(item); }
		return result;
	}

	static void readConfigFromJSON(const fs::path& fdir, const string& fname)
	{
		string full_path = (fdir / fname).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				nlohmann::json json_data = nlohmann::json::parse(fin);
				inst_fname               = json_data["instance"]["sensor_fname"];
				mission_log_fname        = json_data["instance"]["mission_log_fname"];
				power_log_fname          = json_data["instance"]["power_log_fname"];
				chrg_fname               = json_data["instance"]["chrg_fname"];
				edge_fname               = json_data["instance"]["edge_fname"];
				print_result             = json_data["print_result"];
				save_est_sensors         = json_data["save_est_sensors"];
				num_ants                 = json_data["ACS"]["num_ants"];
				max_no_impr              = json_data["ACS"]["max_no_impr_acs"];
				num_iters_acs            = json_data["ACS"]["num_iters_acs"];
				num_iters_2opt           = json_data["ACS"]["num_iters_2opt"];
				q0                       = json_data["ACS"]["q0"];
				rho                      = json_data["ACS"]["rho"];
				alpha                    = json_data["ACS"]["alpha"];
				beta                     = json_data["ACS"]["beta"];
				tol_acs                  = json_data["ACS"]["tol"];
				uav_dt                   = json_data["uav_power_dt"];
				bool print_config        = json_data["print_config"];
				if (print_config) { cout << json_data.dump(4) << "\n"; cout.flush(); }
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readMissionLog(const fs::path& fdir, const string& fname, 
		double& engy, Point3D& coord)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				// Exclude first line (column name)
				string first_line; getline(fin, first_line);
				string x, y, z, t_arrive, E_arrive, t_leave, E_leave, E_chrg;

				while (getline(fin, x, COMMA_CHAR)) {
					getline(fin, y, COMMA_CHAR);
					getline(fin, z, COMMA_CHAR);
					getline(fin, t_arrive, COMMA_CHAR);
					getline(fin, E_arrive, COMMA_CHAR);
					getline(fin, t_leave, COMMA_CHAR);
					getline(fin, E_leave, COMMA_CHAR);
					getline(fin, E_chrg);

					engy    = stod(E_leave);
					coord.x = stod(x);
					coord.y = stod(y);
					coord.z = stod(z);
				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readSensorsFromCSV(const fs::path& fdir, const string& fname,
		vector<Sensor>& sensors)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				if (!sensors.empty()) { sensors.clear(); }
				string first_line; getline(fin, first_line); // Exclude column name
				size_t cnt = 0;
				string x, y, z, volt;
				while (getline(fin, x, COMMA_CHAR)) {
					getline(fin, y, COMMA_CHAR);
					getline(fin, z, COMMA_CHAR);
					getline(fin, volt);

					if (cnt > 1) {
						sensors.push_back(
							Sensor(stod(x), stod(y), stod(z), stod(volt)));
					}
					else { 
						sensors.push_back(Sensor(stod(x), stod(y), stod(z))); cnt++;
					}

				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readOfflineEdgeCosts(const fs::path& fdir, const string& fname,
		const vector<vector<double>>& edge_costs)
	{

	}

	static void saveEstSensorsToCSV(const fs::path& fdir, const string& fname,
		const vector<Sensor>& sensors)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path);
			if (fout.is_open()) {
				for (size_t i = 0; i < sensors.size(); i++) {
					fout << sensors[i].prize() << COMMA_CHAR 
						<< sensors[i].chrgCost() << "\n";
				}
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveInitEdgeCosts(const fs::path& fdir, const string& fname,
		const vector<vector<double>>& edge_costs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path);
			if (fout.is_open()) {
				size_t num_nodes = edge_costs.size();
				for (size_t i = 0; i < num_nodes; i++) {
					for (size_t j = 0; j < num_nodes; j++) {
						fout << edge_costs[i][j];
						if (j < num_nodes - 1) { fout << COMMA_CHAR; }
					}
					fout << "\n";
				}
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveFinalPath(const fs::path& fdir, const string& fname,
		const double exe_time, const double path_prize, 
		const double path_cost, const vector<size_t>& path)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path);
			if (fout.is_open()) {
				fout << exe_time << COMMA_CHAR << "0.0" << COMMA_CHAR
					<< path_prize << COMMA_CHAR << path_cost;
				for (const size_t idx : path) { fout << COMMA_CHAR << idx; }
				fout << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void createPowerLog(const fs::path& fdir, const string& fname, bool visual)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path);
			if (fout.is_open()) { fout.close(); return; }
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}
}

int main(int argc, char* argv[])
{
    //string config_fname = "offline_config.json";
	string config_fname = argv[1];
	DataIO::readConfigFromJSON(input_fdir, config_fname);
	//! Load UAV initial state, initialize ACS solver.
	double uav_E0 = 0; Point3D uav_coord0 = Point3D();
	
	DataIO::readMissionLog(input_fdir, mission_log_fname, uav_E0, uav_coord0);
	DroneState uav0 = DroneState(uav_E0, uav_dt, uav_coord0);
	unique_ptr<ACS> acs_solver = make_unique<ACS>(
		num_ants, max_no_impr, num_iters_acs, num_iters_2opt, 
		q0, alpha, beta, rho, tol_acs, uav0);
	//! Load sensor node instance
	std::vector<Sensor> sensors;
	DataIO::readSensorsFromCSV(inst_fdir, inst_fname, sensors);
	if (save_est_sensors) {
		DataIO::saveEstSensorsToCSV(output_fdir, "offline_exp_" + inst_fname, sensors);
	}
	//! Algorithm main process.
	unique_ptr<SysTimer> sys_timer = nullptr; double alg_time = 0;
	sys_timer = make_unique<SysTimer>(); sys_timer->tick();
	acs_solver->initColony(sensors);
	acs_solver->initPhmoneMatrix();
	acs_solver->evolveUntilStopCriteria();
	sys_timer->tock(); alg_time += sys_timer->duration().count();
	time_t time_id = std::chrono::system_clock::to_time_t(
		std::chrono::system_clock::now());
	//! Save final path and edge costs.
	Fitness final_fit = acs_solver->getBestFitness();
	vector<size_t> final_path = acs_solver->getBestPath();
	acs_solver->checkPathCostPrize(final_path, final_fit.cost, final_fit.prize);
	alg_time /= 1000.;
	DataIO::saveFinalPath(input_fdir, chrg_fname, alg_time,
		final_fit.prize, final_fit.cost, final_path);
	DataIO::saveInitEdgeCosts(input_fdir, edge_fname, acs_solver->getEdgeCosts());
	if (print_result) {  //! Print result to the terminal.
		cout << "[INFO] Offline alg. time " << alg_time << " s, path cost " 
			<< final_fit.cost << "/" << acs_solver->getBudget() << " kJ, path prize " 
			<< final_fit.prize << " kJ.\n";
	}
	DataIO::createPowerLog(input_fdir, power_log_fname, true);
	return 0;
}

