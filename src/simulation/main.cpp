#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream>
#include "json.hpp"
#include "drone.h"

using std::cout, std::cerr;
using std::string;
using std::unique_ptr, std::make_unique;
using std::tuple, std::make_tuple, std::get;
using std::vector;
using std::to_string;

namespace fs                   = std::filesystem;
fs::path root_dir              = fs::current_path();
fs::path input_fdir            = root_dir / "input";		// Input root 
fs::path output_fdir           = root_dir / "output";		// Output root
fs::path inst_fdir             = input_fdir / "instance";   // Sensor nodes
string mission_log_fname       = "x";					// Mission log file name
string power_log_fname         = "x";					// Measured power file name
string chrg_fname              = "x";					// Charging list file name
string inst_fname              = "x";					// Instance file name
string edge_fname              = "x";		// File name of all real-world edge costs
bool log_power                 = false;		// 
bool print_result              = false;
bool use_avg_power             = false;
double simu_dt                 = 0;			// Simulation time step
size_t sample_per_simu_dt      = 0;			// Sample per x simulation time step
double eta_ipt                 = 0;			// IPT efficiency
double eta_chrg                = 0;			// CC to supcap bank charging efficiency
double real_engy_coeff_min     = 0;			// Lower bound for uniform distb
double real_engy_coeff_max     = 0;			// Upper bound for uniform distb
double takeoff_mean            = 0;			// Mean for normal distb (Takeoff)
double takeoff_std             = 0;			// Standard deviation for normal distb
double cruise_mean             = 0;			// Mean for normal distb (Cruise)
double cruise_std	           = 0;			// Standard deviation for normal distb
double landing_mean            = 0;			// Mean for normal distb (Landing)
double landing_std             = 0;			// Standard deviation for normal distb
constexpr size_t DEPOT_IDX     = 0;			// The depot start index, always 0.
constexpr size_t END_IDX       = 1;			// The depot end index, always 1.
constexpr int MISSION_CONTINUE = 0;			// Mission continue flag.
constexpr int MISSION_END      = 1;			// Mission end flag.
constexpr int VISIT_FAILED     = -1;		// UAV Visit failed flag.
constexpr int SERVICE_FAILED   = -2;		// UAV Service failed flag.
constexpr int RTH_FAILED       = -3;		// UAV Return-To-Home failed flag.

//! uav_coord, t_arrive, E_arrive, t_depart, E_depart, E_recharged
typedef tuple<Point3D, double, double, double, double, double> CheckPt;

namespace DataIO
{
	using std::ifstream, std::getline, std::ofstream;
	using std::stod, std::stoi, std::stoul;
	using std::stringstream;
	using std::runtime_error;
	using std::this_thread::sleep_for;

	constexpr auto USCOR_CHAR = '_';
	constexpr auto COMMA_CHAR = ',';
	constexpr auto FILE_TYPE  = ".csv";
	constexpr size_t MAX_ATTEMPTS = 3;
	constexpr std::chrono::milliseconds RETRY_DELAY(100);

	static vector<string> split(const string& s, char delim) {
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
				power_log_fname          = json_data["instance"]["power_log_fname"];
				mission_log_fname        = json_data["instance"]["mission_log_fname"];
				chrg_fname               = json_data["instance"]["chrg_fname"];
				inst_fname               = json_data["instance"]["sensor_fname"];
				edge_fname               = json_data["instance"]["edge_fname"];
				simu_dt                  = json_data["simulation"]["simu_dt"];
				sample_per_simu_dt       = json_data["simulation"]["sample_dt"];
				eta_ipt                  = json_data["simulation"]["eta_ipt"];
				eta_chrg                 = json_data["simulation"]["eta_chrg"];
				real_engy_coeff_min      = json_data["simulation"]["uniform"]["real_engy_lb"];
				real_engy_coeff_max      = json_data["simulation"]["uniform"]["real_engy_ub"];
				takeoff_mean             = json_data["simulation"]["normal"]["takeoff_mean"];
				takeoff_std              = json_data["simulation"]["normal"]["takeoff_std"];
				cruise_mean              = json_data["simulation"]["normal"]["cruise_mean"];
				cruise_std               = json_data["simulation"]["normal"]["cruise_std"];
				landing_mean             = json_data["simulation"]["normal"]["landing_mean"];
				log_power				 = json_data["simulation"]["log_power"];
				use_avg_power            = json_data["simulation"]["use_avg_power"];
				landing_std              = json_data["simulation"]["normal"]["landing_std"];
				print_result             = json_data["print_result"];
				bool print_config        = json_data["print_config"];
				if (print_config) { cout << json_data.dump(4) << "\n"; cout.flush(); }
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
				string first_line; getline(fin, first_line);
				size_t cnt = 0;
				string x, y, z, volt;
				if (!sensors.empty()) { sensors.clear(); }
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

	static void readLatestMissionLog(const fs::path& fdir, const string& fname,
		Point3D& coord, double& time_leave, double& engy_leave, double& engy_chrg)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				fin.seekg(-1, std::ios_base::end);		// Go to one spot before the EOF
				if (fin.peek() == '\n') {
					fin.seekg(-1, std::ios_base::cur);
					int i = static_cast<int>(fin.tellg());
					for (i; i > -1; i--) {
						if (fin.peek() == '\n') { fin.get(); break; }
						fin.seekg(i, std::ios_base::beg);
					}
				}

				string last_line; getline(fin, last_line);
				vector<string> tokens = split(last_line, COMMA_CHAR);
				coord.x               = stod(tokens[0]);
				coord.y               = stod(tokens[1]);
				coord.z               = stod(tokens[2]);
				time_leave            = stod(tokens[5]);
				engy_leave            = stod(tokens[6]);
				engy_chrg             = stod(tokens[7]);
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void createEdgeVisitCostFile(const fs::path& fdir, const string& fname)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::out);
			if (fout.is_open()) { fout.close(); return; }
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveOneEdgeVisitCost(const fs::path& fdir, const string& fname,
		const size_t idx0, const size_t idx1, const double visit_time,
		const double visit_engy, const double avg_pwr_t, const double avg_pwr_l,
		const vector<double>& avg_pwr_c)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::app);
			if (fout.is_open()) {
				fout << idx0 << COMMA_CHAR << idx1 << COMMA_CHAR << visit_time 
					<< COMMA_CHAR << visit_engy << COMMA_CHAR << avg_pwr_t;
				for (double p : avg_pwr_c) { fout << COMMA_CHAR << p; }
				fout << COMMA_CHAR << avg_pwr_l << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveOneEdgeVisitCost(const fs::path& fdir, const string& fname,
		const size_t idx0, const size_t idx1, const double visit_time,
		const double visit_engy, const vector<double>& pwrs_minmaxs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::app);
			if (fout.is_open()) {
				fout << idx0 << COMMA_CHAR << idx1 << COMMA_CHAR << visit_time
					<< COMMA_CHAR << visit_engy;
				for (double p : pwrs_minmaxs) { fout << COMMA_CHAR << p; }
				fout << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveMissionResults(const fs::path& fdir, const string& fname,
		const CheckPt& uav_check_pts)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::app);
			if (fout.is_open()) {
				fout << get<0>(uav_check_pts).x << COMMA_CHAR 
					<< get<0>(uav_check_pts).y << COMMA_CHAR 
					<< get<0>(uav_check_pts).z << COMMA_CHAR 
					<< get<1>(uav_check_pts) << COMMA_CHAR
					<< get<2>(uav_check_pts) << COMMA_CHAR
					<< get<3>(uav_check_pts) << COMMA_CHAR
					<< get<4>(uav_check_pts) << COMMA_CHAR
					<< get<5>(uav_check_pts) << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveObservedAvgPower(const fs::path& fdir, const string& fname, 
		const double time_id, const vector<double>& obs_avg_pwr)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::app);
			if (fout.is_open()) {
				fout << time_id;
				for (size_t i = 0; i < obs_avg_pwr.size(); ++i) {
					fout << COMMA_CHAR << obs_avg_pwr[i];
				}
				fout << "\n";
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveObservedPowerMinMax(const fs::path& fdir, const string& fname,
		const double time_id, const vector<double>& obs_pwrs_minmaxs)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				//! Get first line data
				string line0; getline(fin, line0);
				vector<string> line0_items = split(line0, COMMA_CHAR);
				fin.close();

				ofstream fout(full_path, std::ios::out);
				if (fout.is_open()) {
					fout << time_id;
					if (line0_items.empty()) {
						for (double p : obs_pwrs_minmaxs) { fout << COMMA_CHAR << p; }
					}
					else {
						for (size_t i = 1; i < 4; i++) {
							double p_prev = stod(line0_items[i]);
							fout << COMMA_CHAR << (obs_pwrs_minmaxs[i - 1] < p_prev ? 
								obs_pwrs_minmaxs[i - 1] : p_prev);
						}
						for (size_t i = 4; i < 7; i++) {
							double p_prev = stod(line0_items[i]);
							fout << COMMA_CHAR << (obs_pwrs_minmaxs[i - 1] > p_prev ? 
								obs_pwrs_minmaxs[i - 1] : p_prev);
						}
					}
					fout << "\n";
					fout.close(); return;
				}
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void saveUpdatedWSN(const fs::path& fdir, const string& fname,
		const vector<Sensor>& sensors)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ofstream fout(full_path, std::ios::out);
			if (fout.is_open()) {
				fout << "lat,lon,alt,volt\n";
				for (size_t i = 0; i < sensors.size(); ++i) {
					fout << sensors[i].coord().x << COMMA_CHAR
						<< sensors[i].coord().y << COMMA_CHAR
						<< sensors[i].coord().z << COMMA_CHAR 
						<< sensors[i].volt() << "\n";
				}
				fout.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readStartAndNextTargetNode(const fs::path& fdir, 
		const string& fname, size_t& start_idx, size_t& next_sn_idx)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				fin.seekg(-1, std::ios_base::end);		// Go to one spot before the EOF
				if (fin.peek() == '\n') {
					fin.seekg(-1, std::ios_base::cur);
					int i = static_cast<int>(fin.tellg());
					for (i; i > -1; i--) {
						if (fin.peek() == '\n') { fin.get(); break; }
						fin.seekg(i, std::ios_base::beg);
					}
				}

				string last_line; getline(fin, last_line);
				vector<string> tokens = split(last_line, COMMA_CHAR);
				//! INDEX 0: EXE_TIME, INDEX 1: Belief;
				//! INDEX 2: Path prize; INDEX 3: Path cost; 
				start_idx = stoul(tokens[4]);  // INDEX 4 is the start node index.
				next_sn_idx = stoul(tokens[5]);  // INDEX 5 is the next node index.
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}

	static void readVisitCost(const fs::path& fdir, const string& fname,
		const size_t idx0, const size_t idx1, double& visit_time,
		double& visit_engy, vector<double>& avg_power_samples)
	{
		string full_path = (fdir / (fname + FILE_TYPE)).string();
		for (size_t attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
			ifstream fin(full_path);
			if (fin.is_open()) {
				string line;
				while (getline(fin, line)) {
					vector<string> tokens = split(line, COMMA_CHAR);
					if (stoul(tokens[0]) == idx0 && stoul(tokens[1]) == idx1) {
						visit_time = stod(tokens[2]);
						visit_engy = stod(tokens[3]);
						for (size_t i = 4; i < tokens.size(); i++) {
							avg_power_samples.push_back(stod(tokens[i]));
						}
						break;
					}
				}
				fin.close(); return;
			}
			if (attempt < MAX_ATTEMPTS - 1) { sleep_for(RETRY_DELAY); }
		}
		cerr << "[ERROR] Can't open " + full_path + "\n"; throw runtime_error("");
	}
}

static void calcAllRealWorldVisitCosts(const fs::path& fdir, const string& fname,
	const vector<Sensor>& sn_list, Drone& uav)
{
	DataIO::createEdgeVisitCostFile(fdir, fname);
	size_t num_nodes = sn_list.size();

	vector<vector<double>> dists = vector<vector<double>>(
		num_nodes, vector<double>(num_nodes));
	for (size_t i = 0; i < num_nodes - 1; i++) {
		for (size_t j = i + 1; j < num_nodes; j++) {
			if (i != END_IDX && j != DEPOT_IDX) {
				dists[i][j] = dists[j][i] = sn_list[i].distXY(sn_list[j]);
			}
		}
	}

	vector<vector<double>> edge_costs = vector<vector<double>>(
		num_nodes, vector<double>(num_nodes));
	for (size_t i = 0; i < num_nodes; i++) {
		for (size_t j = 0; j < num_nodes; j++) {
			if (i == END_IDX || j == DEPOT_IDX || i == j) { continue; }
			if (!sn_list[i].isClose(sn_list[j])) {
				double visit_time = 0, visit_engy = 0;
				double avg_pwr_t = 0, avg_pwr_l = 0;
				vector<double> avg_pwrs_c;
				uav.setCoord(sn_list[i].coord());
				uav.virtualVisit(sn_list[j].coord(), dists[i][j],
					visit_time, visit_engy, avg_pwr_t, avg_pwr_l, avg_pwrs_c);
				DataIO::saveOneEdgeVisitCost(fdir, fname, i, j, 
					visit_time, visit_engy, avg_pwr_t, avg_pwr_l, avg_pwrs_c);
			}
		}
	}
}

static void calcAllRealWorldVisitCostsMinMax(const fs::path& fdir, const string& fname,
	const vector<Sensor>& sn_list, Drone& uav)
{
	DataIO::createEdgeVisitCostFile(fdir, fname);
	size_t num_nodes = sn_list.size();
	vector<vector<double>> dists = vector<vector<double>>(
		num_nodes, vector<double>(num_nodes));
	for (size_t i = 0; i < num_nodes - 1; i++) {
		for (size_t j = i + 1; j < num_nodes; j++) {
			if (i != END_IDX && j != DEPOT_IDX) {
				dists[i][j] = dists[j][i] = sn_list[i].distXY(sn_list[j]);
			}
		}
	}

	vector<vector<double>> edge_costs = vector<vector<double>>(
		num_nodes, vector<double>(num_nodes));
	for (size_t i = 0; i < num_nodes; i++) {
		for (size_t j = 0; j < num_nodes; j++) {
			if (i == END_IDX || j == DEPOT_IDX || i == j) { continue; }
			if (!sn_list[i].isClose(sn_list[j])) {
				double visit_time = 0, visit_engy = 0;
				vector<double> pwrs_minmaxs;
				uav.setCoord(sn_list[i].coord());
				uav.virtualVisit(sn_list[j].coord(), dists[i][j],
					visit_time, visit_engy, pwrs_minmaxs);
				DataIO::saveOneEdgeVisitCost(fdir, fname, i, j,
					visit_time, visit_engy, pwrs_minmaxs);
			}
		}
	}
}

static int nextSensorSimulation(const size_t next_idx, const double time_visit,
	const double engy_visit, const double eta_ipt, const double eta_chrg, 
	const bool is_avg_power, vector<double>& power_samples, vector<Sensor>& sn_list, 
	CheckPt& uav_check_pts, Drone& uav)
{
	bool is_visit_ok = uav.getEnergyNow() > engy_visit + EPS;
	uav.setStatus(time_visit, engy_visit);
	double time_arrive = uav.getTimeNow(), engy_arrive = uav.getEnergyNow();
	if (is_visit_ok) { //! VISIT PROCESS
		//vector<double> obs_avg_pwr;
		//uav.getVisitSampleTimeStamps(sn_list[next_idx].coord(), obs_ts);
		size_t num_sn = sn_list.size();
		for (size_t i = 2; i < num_sn; i++) {  //! 0 for start idx, 1 for end idx
			sn_list[i].linearEnergyDecay(uav.getDeltaTime());
		}
		if (next_idx == END_IDX) {
			uav_check_pts = make_tuple(sn_list[next_idx].coord(), 
				time_arrive, engy_arrive, -1, -1, 0);
			return 1;  //! Mission end.
		}

		//! SERVICE PROCESS
		bool is_service_ok = uav.service(eta_ipt, eta_chrg, sn_list[next_idx]);
		double time_leave = uav.getTimeNow(), engy_leave = uav.getEnergyNow();
		double engy_chrg = uav.getDeltaChargedEnergy();
		if (is_service_ok) {
			for (size_t i = 2; i < num_sn; i++) { //! 0 for start idx, 1 for end idx
				if (i == next_idx) { sn_list[i].updateStatusAfterIPT(); }
				else { sn_list[i].linearEnergyDecay(uav.getDeltaTime()); }
			}
			uav_check_pts = make_tuple(sn_list[next_idx].coord(), 
				time_arrive, engy_arrive, time_leave, engy_leave, engy_chrg);
			if (log_power) {
				if (is_avg_power) {
					DataIO::saveObservedAvgPower(
						input_fdir, power_log_fname, uav.getTimeNow(), power_samples);
				}
				else {
					DataIO::saveObservedPowerMinMax(
						input_fdir, power_log_fname, uav.getTimeNow(), power_samples);
				}
			}
			return 0;  //! Mission continue.
		}
		else {
			uav_check_pts = make_tuple(sn_list[next_idx].coord(),
				time_leave, engy_leave, -1, -1, 0);
			return -1;  //! Visit succeeded, but service failed.
		}
	}
	else { 
		uav_check_pts = make_tuple(sn_list[next_idx].coord(),
			time_arrive, engy_arrive, -1, -1, 0);

		if (next_idx != END_IDX) { return -2; }  //! Visit failed.
		else { return -3; }						 //! Return-to-home failed.
	}
}

int main(int argc, char* argv[])
{
	string config_name = argv[1];
	bool compute_all_edges = std::stoi(argv[2]);
	//string config_name = "simulation_config.json";
	//bool compute_all_edges = false;

	DataIO::readConfigFromJSON(input_fdir, config_name);
	//! Read previous sensor network.
	vector<Sensor> sn_list;
	DataIO::readSensorsFromCSV(inst_fdir, inst_fname, sn_list);

	//! Compute all real-world edge costs (ONLY CALLED ONCE after offline planning).
	if (compute_all_edges) {
		unique_ptr<Drone> real_world_uav = make_unique<Drone>(simu_dt, 
			sample_per_simu_dt, takeoff_mean, takeoff_std, cruise_mean, cruise_std, 
			landing_mean, landing_std);
		if (use_avg_power) {
			calcAllRealWorldVisitCosts(
				input_fdir, edge_fname, sn_list, *real_world_uav);
		}
		else {
			calcAllRealWorldVisitCostsMinMax(
				input_fdir, edge_fname, sn_list, *real_world_uav);
		}
		return 0;
	}

	//! Then in [ONLINE] phase, we need to run the simulation.
	//! Read next target node index (can be end index or a sensor node index).
	size_t start_idx = 0, next_idx = 0;
	DataIO::readStartAndNextTargetNode(input_fdir, chrg_fname, start_idx, next_idx);
	//! Read real-world edge costs computed before.
	double time_visit = 0, engy_visit = 0;
	vector<double> power_samples;
	DataIO::readVisitCost(input_fdir, edge_fname, start_idx, next_idx, 
		time_visit, engy_visit, power_samples);
	//! Read latest mission log and initialize UAV.
	Point3D uav_coord = Point3D();
	double uav_t = 0, uav_E = 0, uav_chrg_E = 0;
	DataIO::readLatestMissionLog(
		input_fdir, mission_log_fname, uav_coord, uav_t, uav_E, uav_chrg_E);
	unique_ptr<Drone> online_uav = make_unique<Drone>(
		simu_dt, sample_per_simu_dt, uav_t, uav_E, uav_chrg_E, uav_coord);
	//! Simulation
	CheckPt this_check_pt;
	int status = nextSensorSimulation(next_idx, time_visit, engy_visit, eta_ipt, 
		eta_chrg, use_avg_power, power_samples, sn_list, this_check_pt, *online_uav);
	DataIO::saveMissionResults(input_fdir, mission_log_fname, this_check_pt);
	//! Save updated sensor network.
	vector<string> tokens = DataIO::split(inst_fname, DataIO::USCOR_CHAR);
	string wsn_iter_name = tokens[0] + DataIO::USCOR_CHAR +
		to_string(std::stoi(tokens[1]) + 1);
	DataIO::saveUpdatedWSN(inst_fdir, wsn_iter_name, sn_list);
	//! Print result at terminal.
	if (print_result) {
		switch (status)
		{
		case MISSION_CONTINUE:
			cout << "[INFO] Mission continue, UAV visited and recharged sensor "
				<< next_idx << ", used "
				<< online_uav->getDeltaTime() << " s, consumed "
				<< online_uav->getDeltaEnergy() << " kJ, recharged "
				<< online_uav->getDeltaChargedEnergy() << " kJ.\n";
			break;
		case MISSION_END:
			cout << "[INFO] Mission end, UAV returned to home (node 1).\n";
			break;
		case VISIT_FAILED:
			cout << "[WARN] Mission failed, UAV cannot visit " << next_idx << ".\n";
			break;
		case SERVICE_FAILED:
			cout << "[WARN] Mission failed, UAV cannot service " << next_idx << ".\n";
			break;
		case RTH_FAILED:
			cout << "[WARN] Mission failed, UAV cannot return to home (node 1).\n";
			break;
		default:
			cout << "[ERROR] Unknown mission status.\n";
			break;
		}
	}
	return status;
}

