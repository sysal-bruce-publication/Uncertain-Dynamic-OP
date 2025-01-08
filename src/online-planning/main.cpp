#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include "system_timer.h"
#include "json.hpp"
#include "acs.h"

using std::cout, std::cerr;
using std::runtime_error, std::invalid_argument;
using std::unique_ptr, std::make_unique;
using std::ifstream, std::getline, std::ofstream;
using std::stringstream, std::string;
using std::to_string, std::stoi, std::stod, std::stoul;
using std::vector;
namespace fs = std::filesystem;

//! Global variables for later usage.
fs::path root_dir        = fs::current_path();	// Main root
fs::path input_fdir      = root_dir / "input";					// Input root 
fs::path output_fdir     = root_dir / "output";					// Output root
fs::path inst_fdir       = input_fdir / "instance";				// Sensor nodes
string inst_fname        = "x";				// Instance file name
string chrg_fname        = "x";				// Charging list file name
string mission_log_fname = "x";				// Mission log file name
string power_log_fname   = "x";				// Average power log file name
string edge_fname        = "x";				// Edge cost matrix file name
string strategy          = "x";				// Strategy to update nbr edge cost.
size_t num_samples_MC    = 0;				// Number of samples MC
size_t num_ants          = 0;				// Number of ants in ACS
size_t max_no_impr       = 0;				// Max number of no improvement
size_t num_iters_acs     = 0;				// Number of iterations in ACS
size_t num_iters_2opt    = 0;				// Number of iterations in 2-opt
double q0                = 0;				// ACS hyper-parameter q0
double rho               = 0;				// ACS pheromone evaporation rate
double alpha_acs         = 0;				// ACS hyper-parameter alpha
double beta_acs          = 0;				// ACS hyper-parameter beta
double tol_acs           = 0;				// ACS no improvement threshold
double act_weight        = 0;				// Weight for actual energy cost
double exp_weight        = 0;				// Weight for expected energy cost
double alpha0_BE		 = 0;				// Alpha0 for Bayesian estimation
double kappa0_BE		 = 0;				// Kappa0 for Bayesian estimation
double belief_weight     = 0;				// Belief weight for Bayesian estimation
double bgt0				 = 0;				// Initial budget
bool print_result        = false;			// Whether print ACS's result
bool save_edge_file      = false;
vector<double> cdf_probs;					// CDF probabilities for Bayesian estimation

namespace DataIO
{
	using std::this_thread::sleep_for;
	using json = nlohmann::json;

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
				json json_data    = json::parse(fin);
				print_result      = json_data["print_result"];
				mission_log_fname = json_data["instance"]["mission_log_fname"];
				power_log_fname   = json_data["instance"]["power_log_fname"];
				chrg_fname        = json_data["instance"]["chrg_fname"];
				inst_fname        = json_data["instance"]["sensor_fname"];
				edge_fname        = json_data["instance"]["edge_fname"];
				save_edge_file    = json_data["instance"]["save_edge_file"];
				num_ants          = json_data["ACS"]["num_ants"];
				max_no_impr       = json_data["ACS"]["max_no_impr_acs"];
				num_iters_acs     = json_data["ACS"]["num_iters_acs"];
				num_iters_2opt    = json_data["ACS"]["num_iters_2opt"];
				q0                = json_data["ACS"]["q0"];
				rho               = json_data["ACS"]["rho"];
				alpha_acs         = json_data["ACS"]["alpha"];
				beta_acs          = json_data["ACS"]["beta"];
				tol_acs           = json_data["ACS"]["tol"];
				strategy          = json_data["online"]["strategy"]["name"];
				bool print_config = json_data["print_config"];
				if (strategy == "WeightedAvg") {
					act_weight = json_data["online"]["strategy"][strategy]["act_weight"];
					exp_weight = json_data["online"]["strategy"][strategy]["exp_weight"];
				}
				else if (strategy == "MCGreedy") {
					num_samples_MC = json_data["online"]["strategy"][strategy]["num_samples"];
				}
				else if (strategy == "Bayesian") {
					alpha0_BE = json_data["online"]["strategy"][strategy]["alpha0"];
					kappa0_BE = json_data["online"]["strategy"][strategy]["kappa0"];
					belief_weight = json_data["online"]["strategy"][strategy]["belief_weight"];
					if (!cdf_probs.empty()) { cdf_probs.clear(); } cdf_probs.reserve(50);
					for (const double cdf_p : json_data["online"]["strategy"][strategy]["cdf_probs"]) {
						cdf_probs.push_back(cdf_p);
					}
					cdf_probs.shrink_to_fit();
				}
				if (print_config) { cout << json_data.dump(4) << "\n"; cout.flush(); }
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " << full_path + "\n"; throw runtime_error("");
	}

	/**
	 * Read mission log to get LATEST UAV status (coordinate and residual energy).
	 * 
	 * \param fdir
	 * \param fname
	 * \param coord
	 * \param engy_uav
	 * \param engy_cruise
	 */
    static void readMissionLog(const fs::path& fdir, const string& fname,
        Point3D& coord, double& engy_leave, double& engy_visit, double& init_bat)
    {
		string full_path = (fdir / (fname + FILE_TYPE)).string(); 
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			size_t cnt = 0;
			if (fin.is_open()) {
				string first_line; getline(fin, first_line); // Exclude column name
				string x, y, z, e_arr, t_arr, e_leave, t_leave, e_chrg;
				double e_arrive = 0;
				vector<double> es_leave;
				while (getline(fin, x, COMMA_CHAR)) {
					getline(fin, y, COMMA_CHAR);
					getline(fin, z, COMMA_CHAR);
					getline(fin, t_arr, COMMA_CHAR);
					getline(fin, e_arr, COMMA_CHAR);
					getline(fin, t_leave, COMMA_CHAR);
					getline(fin, e_leave, COMMA_CHAR);
					getline(fin, e_chrg);

					double engy_leave = stod(e_leave);
					if (cnt == 0) { init_bat = engy_leave; cnt++; }
					coord = Point3D(stod(x), stod(y), stod(z));
					e_arrive = stod(e_arr);
					es_leave.push_back(engy_leave);
				}
				engy_visit = es_leave[es_leave.size() - 2] - e_arrive;
				engy_leave = es_leave.back();
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " << full_path + "\n"; throw runtime_error("");
    }

	/**
	 * Read visit history and latest expected charging list.
	 * 
	 * \param fdir
	 * \param fname
	 * \param visit_hist Visit history of target nodes.
	 * \param last_path Latest expected charging list.
	 */
	static void readVisitHistory(const fs::path& fdir, const string& fname,
		vector<size_t>& visit_hist, vector<size_t>& last_path)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				if (!visit_hist.empty()) { visit_hist.clear(); }
				visit_hist.push_back(0);
				vector<vector<size_t>> chrg_lists;
				string line = "";
				while (getline(fin, line)) {
					vector<string> items = split(line, COMMA_CHAR);
					vector<size_t> this_list; this_list.reserve(items.size() - 4);
					//! INDEX 0: ALG TIME, INDEX 1: BELIEF; 
					//! INDEX 2: PATH PRIZE; INDEX 3: PATH COST;
					for (size_t i = 4; i < items.size(); i++) {
						this_list.push_back(stoul(items[i]));
					}
					chrg_lists.push_back(this_list);
					visit_hist.push_back(this_list[1]);
				}
				last_path = chrg_lists.back();
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readSensorNetwork(const fs::path& fdir, const string& fname,
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

	/**
	 * Read the expected edge costs calculated from offline planning.
	 * 
	 * \param fdir
	 * \param fname
	 * \param num_nodes
	 * \param edge_cost
	 */
	static void readOfflineEdgeCosts(const fs::path& fdir, const string& fname,
		const size_t num_nodes, vector<vector<double>>& edge_cost)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				string line;
				edge_cost.reserve(num_nodes);
				while (getline(fin, line)) {
					vector<double> row; row.reserve(num_nodes);
					stringstream ss(line);
					string data;
					while (getline(ss, data, COMMA_CHAR)) { row.push_back(stod(data)); }
					edge_cost.push_back(row);
				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readAvgPowerSamples(const fs::path& fdir, const string& fname,
		vector<double>& avg_pwr_t, vector<double>& avg_pwr_c, 
		vector<double>& avg_pwr_l)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				string line;
				while (getline(fin, line)) {
					vector<string> items = split(line, COMMA_CHAR);
					//! The first item is time ID, exclude that.
					avg_pwr_t.push_back(stod(items[1]));
					avg_pwr_l.push_back(stod(items.back()));
					for (size_t i = 2; i < items.size() - 1; i++) {
						avg_pwr_c.push_back(stod(items[i]));
					}
				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readPowerMinMax(const fs::path& fdir, const string& fname,
		vector<double>& pwr_minmaxs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				string line0; getline(fin, line0);
				vector<string> line0_items = split(line0, COMMA_CHAR);
				//! p_tf_min, p_tf_max, p_cr_min, p_cr_max, p_ld_min, p_ld_max 
				for (size_t i = 0; i < 6; i++) { 
					pwr_minmaxs.push_back(stod(line0_items[i + 1]));
				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveExpSensorNetwork(const fs::path& fdir, const string& fname,
		const vector<Sensor>& sensors)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::out);
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

	static void saveUpdatedEdgeCosts(const fs::path& fdir, const string& fname,
		const vector<vector<double>>& edge_costs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::out);
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

	static void saveUpdatedPath(const fs::path& fdir, const string& fname, 
		const double exe_time, const double belief, const double path_prize, 
		const double path_cost, const vector<size_t>& path)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::app);
			if (fout.is_open()) {
				fout << exe_time << COMMA_CHAR << belief << COMMA_CHAR
					<< path_prize << COMMA_CHAR << path_cost;
				for (const size_t idx : path) { fout << COMMA_CHAR << idx; }
				fout << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveOccursMap(const fs::path& fdir, const string& fname,
		const std::multimap<size_t, vector<size_t>, std::greater<size_t>>& occurs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::out);
			if (fout.is_open()) {
				for (auto it = occurs.begin(); it != occurs.end(); ++it) {
					fout << it->first;
					for (size_t elem : it->second) { fout << COMMA_CHAR << elem; }
					fout << "\n"; 
				}
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}
}

int main(int argc, char* argv[])
{
	//! Read configuration from JSON file.
	//string config_fname = "online_config.json";
	string config_fname = argv[1];
	DataIO::readConfigFromJSON(input_fdir, config_fname);

	//! Read mission log, updated by SIMULATION phase.
	Point3D uav_coord = Point3D(); double uav_E_now = 0, visit_E_prev = 0;
	DataIO::readMissionLog(
		input_fdir, mission_log_fname, uav_coord, uav_E_now, visit_E_prev, bgt0);
	DroneState uav0 = DroneState(uav_E_now, uav_coord);

	//! Read charging list, updated by previous PLANNING phase.
	vector<size_t> visit_hist, last_path;
	DataIO::readVisitHistory(input_fdir, chrg_fname, visit_hist, last_path);

	//! Read updated sensor nodes, updated by SIMULATION phase.
	vector<Sensor> sensors;
	DataIO::readSensorNetwork(inst_fdir, inst_fname, sensors);

	//! Read offline edge costs.
	size_t num_nodes = sensors.size();
	vector<vector<double>> offline_costs;
	DataIO::readOfflineEdgeCosts(input_fdir, edge_fname, num_nodes, offline_costs);

	//! Initialize ACS solver
	size_t start_idx = visit_hist.back();
	unique_ptr<ACS> acs_solver = make_unique<ACS>(start_idx, num_ants, max_no_impr, 
		num_iters_acs, num_iters_2opt, q0, alpha_acs, beta_acs, rho, tol_acs, uav0);

	//! ACS Main process
	double alg_time = 0;
	unique_ptr<SysTimer> sys_timer = make_unique<SysTimer>(); sys_timer->tick();
	acs_solver->initColony(sensors, visit_hist, offline_costs);
	double belief = 0;
	Fitness final_fit = Fitness();
	vector<size_t> final_path = {start_idx, END_IDX};
	if (strategy == "WeightedAvg") {
		size_t prev_idx = visit_hist[visit_hist.size() - 2];
		acs_solver->updateNeighbEdgeCostsWeightedAvg(
			prev_idx, visit_E_prev, act_weight, exp_weight);
		if (acs_solver->initPhmoneMatrix(last_path)) {
			acs_solver->evolveUntilStopCriteria();
		}
		sys_timer->tock(); alg_time += sys_timer->duration().count();
		final_fit = acs_solver->getBestFitness();
		final_path = acs_solver->getBestPath();
	}
	else if (strategy == "DoNothing") {
		if (acs_solver->initPhmoneMatrix(last_path)) {
			acs_solver->evolveUntilStopCriteria();
		}
		sys_timer->tock(); alg_time += sys_timer->duration().count();
		final_fit = acs_solver->getBestFitness();
		final_path = acs_solver->getBestPath();
	}
	else if (strategy == "Bayesian") {
		vector<double> avg_pwr_takeoff, avg_pwr_cruise, avg_pwr_landing;
		DataIO::readAvgPowerSamples(input_fdir, power_log_fname,
			avg_pwr_takeoff, avg_pwr_cruise, avg_pwr_landing);
		acs_solver->updatePosteriorDistributions(alpha0_BE, kappa0_BE,
			avg_pwr_takeoff, avg_pwr_cruise, avg_pwr_landing);
		vector<AvgPower> avg_powers;
		acs_solver->getUpdatedAvgPowers(cdf_probs, avg_powers);
		size_t num_tests = avg_powers.size();
		vector<vector<vector<double>>> test_costs; test_costs.reserve(num_tests);
		vector<Fitness> test_fitnesses; test_fitnesses.reserve(num_tests);
		vector<vector<size_t>> test_paths; test_paths.reserve(num_tests);
		vector<vector<double>> this_test_cost;
		for (size_t i = 0; i < num_tests; i++) {
			acs_solver->getUpdatedNeighbEdgeCostsBayesian(
				avg_powers[i], this_test_cost);
			test_costs.push_back(this_test_cost);
			acs_solver->setEdgeCosts(this_test_cost);
			if (acs_solver->initPhmoneMatrix(last_path)) { 
				acs_solver->evolveUntilStopCriteria(); 
			}
			test_fitnesses.push_back(acs_solver->getBestFitness());
			test_paths.push_back(acs_solver->getBestPath());
		}
		double prob_min = *cdf_probs.begin(), prob_max = cdf_probs.back();
		double prob_step = prob_max - prob_min;
		Fitness fit_min = *min_element(test_fitnesses.begin(), test_fitnesses.end(),
			[](const Fitness& lhs, const Fitness& rhs) { return lhs < rhs; });
		Fitness fit_max = *max_element(test_fitnesses.begin(), test_fitnesses.end(),
			[](const Fitness& lhs, const Fitness& rhs) { return lhs < rhs; });
		if (fit_min.prize != fit_max.prize) { 
			double prize_step = fit_max.prize - fit_min.prize;
			double prize_weight = 1 - belief_weight;

			std::multimap<double, size_t, std::greater<double>> scores;
			for (size_t i = 0; i < num_tests; i++) {
				double s = belief_weight * (cdf_probs[i] - prob_min) / prob_step
					+ prize_weight * (test_fitnesses[i].prize - fit_min.prize) / prize_step;
				scores.insert(std::make_pair(s, i));
			}
			sys_timer->tock(); alg_time += sys_timer->duration().count();
			size_t best_idx = scores.begin()->second;
			final_fit = test_fitnesses[best_idx];
			final_path = test_paths[best_idx];
			belief = cdf_probs[best_idx] * 100;
			acs_solver->setEdgeCosts(test_costs[best_idx]);
		}
		else {
			sys_timer->tock(); alg_time += sys_timer->duration().count();
			final_fit = test_fitnesses.back();
			final_path = test_paths.back();
			belief = cdf_probs.back() * 100;
			acs_solver->setEdgeCosts(test_costs.back());
		}
	}
	else if (strategy == "MCGreedy") {
		vector<double> pwr_minmaxs;
		DataIO::readPowerMinMax(input_fdir, power_log_fname, pwr_minmaxs);
		double p_t_len = pwr_minmaxs[3] - pwr_minmaxs[0];
		double p_c_len = pwr_minmaxs[4] - pwr_minmaxs[1];
		double p_l_len = pwr_minmaxs[5] - pwr_minmaxs[2];
		final_fit.prize = 1e6; //! We want to search the most conservative path.
		std::multimap<size_t, vector<size_t>, std::greater<size_t>> occurs;

		for (size_t i = 0; i < num_samples_MC; i++) {
			//! Get the first and second pair of the multimap
			if (occurs.size() > 1) {
				auto it = occurs.begin(); size_t num_occurs_1st = it->first;
				size_t num_occurs_2nd = (++it)->first;
				if (num_occurs_1st - num_occurs_2nd > num_samples_MC - i) { break; }
			}

			acs_solver->updateNeighbEdgeCostsMCGreedy(p_t_len, p_c_len, p_l_len,
				pwr_minmaxs[0], pwr_minmaxs[1], pwr_minmaxs[2]);
			if (acs_solver->initPhmoneMatrix(last_path)) {
				acs_solver->evolveUntilStopCriteria();
			}
			vector<size_t> this_path = acs_solver->getBestPath();
			//! Now iterate through multimap to find any occurrence. 
			//! If there exists same path, the key + 1. 
			bool is_occurred = false;

			for (auto it = occurs.begin(); it != occurs.end(); ++it) {
				if (it->second == this_path) {
					if (it->first == occurs.begin()->first || !occurs.empty()) {
						final_fit = acs_solver->getBestFitness();
					}
					auto nodeHandler = occurs.extract(it->first);
					nodeHandler.key() += 1;
					occurs.insert(std::move(nodeHandler));
					is_occurred = true; break;
				}
			}
			//! If there is no occurrence, insert new path.
			if (!is_occurred) { occurs.insert(std::make_pair(1, this_path)); }
		}
		sys_timer->tock(); alg_time += sys_timer->duration().count();
		//DataIO::saveOccursMap(output_fdir, "occurs_" + inst_fname, occurs);
		if (!occurs.empty()) { final_path = occurs.begin()->second; }
		else {
			final_fit = acs_solver->getBestFitness();
			final_path = acs_solver->getBestPath();
		}
	}
	else { cerr << "[ERROR] Unknown strategy.\n"; throw invalid_argument(""); }

	//! Save final solution and execution time.
	alg_time /= 1000.;
	DataIO::saveUpdatedPath(input_fdir, chrg_fname, alg_time, belief,
		final_fit.prize, final_fit.cost, final_path);
	if (strategy != "DoNothing" && save_edge_file) {
		DataIO::saveUpdatedEdgeCosts(
			output_fdir, "edges_" + inst_fname, acs_solver->getEdgeCosts());
	}
	time_t time_id = std::chrono::system_clock::to_time_t(
		std::chrono::system_clock::now());
		
	if (print_result) {  //! Print result to the terminal.
		cout << "[INFO] " << strategy << " alg. time " << alg_time 
			<< " s, path cost " << final_fit.cost << "/" 
			<< acs_solver->getBudget() << " kJ, path prize " 
			<< final_fit.prize << " kJ";
		if (strategy == "Bayesian") { 
			cout << ", safety belief " << belief << " %.\n"; 
		}
		else { cout << ".\n"; }
	}
	return 0;
}

