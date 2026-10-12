#include "score_manager.hpp"
#include <iostream>

int main() {
    std::cout << "=== TESTING MODULE 1: SCORE MANAGER ===" << std::endl;
    ScoreManager sm("data/scores.csv");

    // Simulate match completion logs
    sm.saveMatch("Mode 2: Semi-Auto", 5, 3, "Player 1", 45);
    sm.saveMatch("Mode 3: Autonomous", 2, 5, "Robot 2", 60);

    // Retrieve recent history
    std::vector<MatchRecord> recent = sm.loadRecentMatches(5);
    std::cout << "\nMatch history loaded from CSV (" << recent.size() << " records):" << std::endl;
    for (const auto& r : recent) {
        std::cout << " [" << r.timestamp << "] " 
                  << r.mode << " | P1: " << r.scoreP1 
                  << " - P2: " << r.scoreP2 
                  << " | Winner: " << r.winner 
                  << " (" << r.durationSec << "s)" << std::endl;
    }

    return 0;
}
