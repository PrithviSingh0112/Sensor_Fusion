
# Sensor Fusion: Camera + IMU (C++ Project)

This project implements a C++ application that fuses object detection data from a **camera** (JSON format) with **IMU sensor readings** (CSV format) using **timestamp synchronization**. It clusters detections, matches them to IMU entries within a time tolerance, and smooths the heading values using a Kalman filter.

---

## 🗂️ Project Structure

```
.
├── main.cpp                      # Main C++ source code
├── data/
│   ├── task_cam_data.json        # Input camera data (JSON)
│   └── task_imu(in).csv          # Input IMU data (CSV)
└── fused_data.csv                # Output file with fused results
```

---

## ⚙️ Dependencies

- C++17 compatible compiler (e.g. `g++`, `clang++`)
- [nlohmann/json](https://github.com/nlohmann/json) for JSON parsing
- [Eigen](https://eigen.tuxfamily.org/) for numerical operations (included, not required)

---

## 🧱 Build Instructions

```bash
g++ -std=c++17 main.cpp -o fusion -I /path/to/json -I /path/to/eigen
```

Replace `/path/to/json` and `/path/to/eigen` with the actual include paths to the libraries.

---

## 📥 Input Format

### Camera JSON (`data/task_cam_data.json`)

```json
[
  {
    "cam1": {
      "timestamp": "2024-05-24T12:30:01.123",
      "object_positions_x_y": [
        [10.2, 15.3],
        [10.5, 15.2]
      ]
    }
  }
]
```

### IMU CSV (`data/task_imu(in).csv`)

```
timestamp,id,yaw,heading,state
2024-05-24T12:30:01.100,imu1,0.02,145.6,moving
```

---

## 🚀 What It Does

- Reads camera detections and groups them by timestamp.
- Clusters detections based on proximity (default: 2m threshold).
- Finds the closest IMU record within ±500 ms of each camera timestamp.
- Applies a Kalman filter to smooth heading values.
- Outputs the fused data in `fused_data.csv`.

---

## 📤 Output Format (`fused_data.csv`)

| Field          | Description                                      |
|----------------|--------------------------------------------------|
| `f_timestamp`  | Frame timestamp                                  |
| `f_id`         | Generated cluster ID                             |
| `cluster_data` | List of `[x, y, cam_id]` values per detection     |
| `heading`      | Smoothed heading value from IMU                  |
| `state`        | IMU state associated with the timestamp          |

**Example:**
```csv
f_timestamp,f_id,cluster_data,heading,state
2024-05-24T12:30:01,F1,"[10.200,15.300,cam1],[10.500,15.200,cam1]",145.600,moving
```

---

## 🛠️ Customization

- Adjust clustering distance in `is_within_distance()`.
- Change timestamp tolerance in the IMU-matching loop (`<= 500 ms`).
- Tune Kalman filter behavior in `MySensorFusion::KalmanFilter`.

---

## ❗ Troubleshooting

If your output shows `"unknown"` for `state`, check:
- Timestamp formats (should match in both sources)
- That camera and IMU data cover overlapping time ranges
- That IMU entries exist within the 500 ms matching window

Use debug prints in the code to inspect mismatches.

---

## 📄 License

This project is provided as-is for educational and development purposes.
