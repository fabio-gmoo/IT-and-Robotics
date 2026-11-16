#ifndef SCORE_MANAGER_HPP
#define SCORE_MANAGER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

struct MatchRecord {
    std::string timestamp;
    std::string mode;
    int scoreP1;
    int scoreP2;
    std::string winner;
    int durationSec;
};

class ScoreManager {
private:
    std::string filePath;

    bool fileExists(const std::string& path) {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }

    std::string getCurrentDateTime() {
        auto now = std::chrono::system_clock::now();
        std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

public:
    ScoreManager(const std::string& path = "data/scores.csv") : filePath(path) {
        // Ensure data directory exists
        struct stat info;
        if (stat("data", &info) != 0) {
            mkdir("data", 0777);
        }

        // Initialize CSV file with headers if it does not exist
        if (!fileExists(filePath)) {
            std::ofstream outFile(filePath, std::ios::out);
            if (outFile.is_open()) {
                outFile << "Timestamp,Mode,Player1_Score,Player2_Score,Winner,Duration_Sec\n";
                outFile.close();
            }
        }
    }

    bool saveMatch(const std::string& modeName, int p1, int p2, const std::string& winnerName, int durationSeconds) {
        std::ofstream outFile(filePath, std::ios::app);
        if (!outFile.is_open()) {
            std::cerr << "[ScoreManager] Error: Failed to open " << filePath << " for writing." << std::endl;
            return false;
        }

        outFile << getCurrentDateTime() << ","
                << modeName << ","
                << p1 << ","
                << p2 << ","
                << winnerName << ","
                << durationSeconds << "\n";

        outFile.close();
        std::cout << "[ScoreManager] Match record successfully saved to " << filePath << std::endl;
        return true;
    }

    std::vector<MatchRecord> loadRecentMatches(int limit = 5) {
        std::vector<MatchRecord> records;
        std::ifstream inFile(filePath);
        if (!inFile.is_open()) return records;

        std::string line;
        // Skip header line
        std::getline(inFile, line);

        while (std::getline(inFile, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string item;
            MatchRecord rec;

            if (std::getline(ss, rec.timestamp, ',') &&
                std::getline(ss, rec.mode, ',') &&
                std::getline(ss, item, ',') && (rec.scoreP1 = std::stoi(item), true) &&
                std::getline(ss, item, ',') && (rec.scoreP2 = std::stoi(item), true) &&
                std::getline(ss, rec.winner, ',') &&
                std::getline(ss, item, ',') && (rec.durationSec = std::stoi(item), true)) {
                records.push_back(rec);
            }
        }
        inFile.close();

        // Keep only the most recent records
        if (limit > 0 && records.size() > static_cast<size_t>(limit)) {
            records.erase(records.begin(), records.end() - limit);
        }
        return records;
    }
};

#endif // SCORE_MANAGER_HPP
