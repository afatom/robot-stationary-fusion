#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <optional>
#include <cmath>

// Structure representing a single parsed telemetry line
struct PoolRobotTelemetry {
    double time;
    double pos_x;
    double pos_y;
    double pos_z;
    double yaw;
    double pitch;
    double roll;
    int state;
    double cov_x;
    double cov_y;
    double cov_z;
    double cov_psi_x;
    double cov_psi_y;
    double cov_psi_z;
};

class CSVTelemetryReader {
public:
    CSVTelemetryReader() = default;

    ~CSVTelemetryReader() {
        close();
    }

    // Opens the file and skips the header row
    bool open(const std::string& filepath) {
        file_.open(filepath);
        if (!file_.is_open()) {
            return false;
        }
        
        // Skip header line
        std::string header;
        std::getline(file_, header);
        return true;
    }

    void close() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    // Loops back to the start of the data stream (skipping header)
    void rewind() {
        if (file_.is_open()) {
            file_.clear();
            file_.seekg(0, std::ios::beg);
            std::string header;
            std::getline(file_, header); // skip header
        }
    }

    // Grabs the next valid row. Returns std::nullopt if end-of-file or malformed row
    std::optional<PoolRobotTelemetry> readNextRow() {
        if (!file_.is_open()) {
            return std::nullopt;
        }

        std::string line;
        if (!std::getline(file_, line)) {
            return std::nullopt; // End of file hit
        }

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> columns;

        while (std::getline(ss, cell, ',')) {
            columns.push_back(cell);
        }

        // The EKF dataset must contain exactly 14 columns
        if (columns.size() < 14) {
            return std::nullopt; // Malformed row
        }

        try {
            PoolRobotTelemetry data;
            data.time      = std::stod(columns[0]);
            data.pos_x     = std::stod(columns[1]);
            data.pos_y     = std::stod(columns[2]);
            data.pos_z     = std::stod(columns[3]);
            data.yaw       = std::stod(columns[4]);
            data.pitch     = std::stod(columns[5]);
            data.roll      = std::stod(columns[6]);
            data.state     = std::stoi(columns[7]);
            data.cov_x     = std::stod(columns[8]);
            data.cov_y     = std::stod(columns[9]);
            data.cov_z     = std::stod(columns[10]);
            data.cov_psi_x = std::stod(columns[11]);
            data.cov_psi_y = std::stod(columns[12]);
            data.cov_psi_z = std::stod(columns[13]);
            return data;
        } 
        catch (const std::exception&) {
            return std::nullopt; // Parsing conversion failed
        }
    }

private:
    std::ifstream file_;
};
