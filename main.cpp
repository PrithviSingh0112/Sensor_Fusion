#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <algorithm>
#include "json.hpp"
#include <Eigen/Dense>
#include <chrono>

using json = nlohmann::json;
using namespace std;
using namespace std::chrono;

// ---------------------- Structs ----------------------
struct Detection {
    double x, y;
    string cam_id;
};

struct Cluster {
    string f_id;
    string f_timestamp;
    vector<Detection> cluster_data;
    double heading;
    string state;
};

struct IMUEntry {
    string timestamp;
    double heading;
    string state;
    system_clock::time_point time_point;
};

// ---------------------- Kalman Filter ----------------------
namespace MySensorFusion {
class KalmanFilter {
private:
    double x, P, Q, R;
public:
    KalmanFilter(double init_x = 0.0, double init_P = 1.0, double Q = 0.01, double R = 1.0)
        : x(init_x), P(init_P), Q(Q), R(R) {}

    double update(double measurement) {
        P += Q;
        double K = P / (P + R);
        x += K * (measurement - x);
        P *= (1 - K);
        return x;
    }
};
}

// ---------------------- Utility Functions ----------------------
bool is_within_distance(const Detection& a, const Detection& b, double threshold = 2.0) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy) <= threshold;
}

string generate_random_id() {
    static int counter = 0;
    return "F" + to_string(++counter);
}

system_clock::time_point parse_timestamp(const string& ts) {
    tm tm = {};
    istringstream ss(ts);
    ss >> get_time(&tm, "%Y-%m-%dT%H:%M:%S") 
       || ss >> get_time(&tm, "%Y-%m-%d %H:%M:%S");
    auto tp = system_clock::from_time_t(mktime(&tm));
    double fractions = 0.0;
    size_t dot_pos = ts.find('.');
    if (dot_pos != string::npos) {
        string frac_str = ts.substr(dot_pos + 1);
        fractions = stod("0." + frac_str);
    }
    return tp + milliseconds(static_cast<int>(fractions * 1000));
}

void load_imu_data(const string& csv_file, vector<IMUEntry>& imu_entries) {
    ifstream file(csv_file);
    if (!file.is_open()) {
        cerr << "Failed to open IMU file: " << csv_file << "\n";
        return;
    }
    string line;
    getline(file, line); // skip header

    while (getline(file, line)) {
        istringstream ss(line);
        string ts, dummy, heading_str, state;

        getline(ss, ts, ',');        // timestamp
        getline(ss, dummy, ',');     // ID
        getline(ss, dummy, ',');     // yaw
        getline(ss, heading_str, ','); // heading
        getline(ss, state, ',');     // state column

        try {
            double heading = stod(heading_str);
            system_clock::time_point tp = parse_timestamp(ts);
            imu_entries.push_back({ts, heading, state, tp});
        } catch (const exception& e) {
            cerr << "Error parsing IMU line: " << line << "\n" << e.what() << "\n";
        }
    }

    cout << "Loaded IMU entries: " << imu_entries.size() << "\n";
    sort(imu_entries.begin(), imu_entries.end(), [](const IMUEntry& a, const IMUEntry& b) {
        return a.time_point < b.time_point;
    });
}

void process_json_line(const string& line, map<string, vector<Detection>>& detections_by_ts) {
    try {
        auto j = json::parse(line);
        for (auto& [key, val] : j.items()) {
            string ts = val["timestamp"];
            auto positions = val["object_positions_x_y"];
            for (auto& pos : positions) {
                detections_by_ts[ts].push_back({pos[0], pos[1], key});
            }
        }
    } catch (json::exception& e) {
        cerr << "JSON parse error: " << e.what() << "\nOffending line: " << line << "\n";
    }
}

void write_cluster_to_csv(ofstream& out, const Cluster& cluster) {
    out << fixed << setprecision(3); // always keep 3 decimal places

    out << cluster.f_timestamp << "," << cluster.f_id << ",\"";
    for (size_t i = 0; i < cluster.cluster_data.size(); ++i) {
        out << "[" << cluster.cluster_data[i].x << "," 
            << cluster.cluster_data[i].y << "," 
            << cluster.cluster_data[i].cam_id << "]";
        if (i < cluster.cluster_data.size() - 1) out << ",";
    }
    out << "\"," << cluster.heading << "," << cluster.state << "\n";
}

// ---------------------- Main ----------------------
int main() {
    map<string, vector<Detection>> detections_by_ts;
    vector<IMUEntry> imu_entries;
    MySensorFusion::KalmanFilter kf;

    // Load JSON
    ifstream jsonfile("data/task_cam_data.json");
    stringstream buffer;
    buffer << jsonfile.rdbuf();

    try {
        json full_array = json::parse(buffer.str());
        int line_count = 0;
        for (const auto& obj : full_array) {
            process_json_line(obj.dump(), detections_by_ts);
            ++line_count;
        }
        cout << "Processed JSON entries: " << line_count << "\n";
    } catch (json::exception& e) {
        cerr << "JSON parse error: " << e.what() << "\n";
        return 1;
    }

    // Load IMU
    load_imu_data("data/task_imu(in).csv", imu_entries);
    if (imu_entries.empty()) return 1;

    ofstream fused("fused_data.csv");
    fused << "f_timestamp,f_id,cluster_data,heading,state\n";

    int cluster_total = 0;

    for (const auto& [timestamp, detections] : detections_by_ts) {
        system_clock::time_point cluster_time = parse_timestamp(timestamp);

        // Find nearest IMU entry within 100ms
        auto best_it = imu_entries.end();
long best_diff = std::numeric_limits<long>::max();

for (auto it = imu_entries.begin(); it != imu_entries.end(); ++it) {
    long diff = abs(duration_cast<milliseconds>(it->time_point - cluster_time).count());
    if (diff < best_diff) {
        best_diff = diff;
        best_it = it;
    }
}

// Apply a separate check for threshold
double heading = 0.0;
string state = "unknown";

if (best_it != imu_entries.end() && best_diff <= 500) {
    heading = kf.update(best_it->heading);
    state = best_it->state;
}


        // Clustering
        vector<bool> visited(detections.size(), false);
        for (size_t i = 0; i < detections.size(); ++i) {
            if (visited[i]) continue;

            Cluster cluster;
            cluster.f_timestamp = timestamp;
            cluster.cluster_data.push_back(detections[i]);
            visited[i] = true;

            for (size_t j = i + 1; j < detections.size(); ++j) {
                if (!visited[j] && is_within_distance(detections[i], detections[j])) {
                    cluster.cluster_data.push_back(detections[j]);
                    visited[j] = true;
                }
            }

            cluster.f_id = generate_random_id();
            cluster.heading = heading;
            cluster.state = state;

            write_cluster_to_csv(fused, cluster);
            ++cluster_total;
            
        }
    }

    
   

    cout << "Fusion complete. Total clusters: " << cluster_total << "\n";
    cout << "Output written to fused_data.csv\n";
    return 0;
}
