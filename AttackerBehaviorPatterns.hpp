#ifndef ATTACKER_BEHAVIOR_PATTERNS_HPP
#define ATTACKER_BEHAVIOR_PATTERNS_HPP

#include <string>
#include <vector>

namespace ns3 {

/**
 * @brief Structure to define adaptive attacker behavior patterns
 */
struct AttackerBehaviorPattern {
    std::string vehicleId;           // ID of the attacking vehicle
    std::string patternType;         // Type of attack pattern (ALWAYS_ATTACK, ON_OFF, etc.)
    std::vector<bool> attackSchedule; // Schedule of when to attack (true) or be honest (false)
    uint32_t currentEventIndex;      // Current event index in the schedule
    uint32_t totalParticipations;    // Total number of events participated in
    std::string description;         // Human-readable description of the pattern
};

/**
 * @brief Create adaptive attacker behavior patterns
 * @param attackerVehicles List of vehicle IDs that will be attackers
 * @param numEvents Total number of events in the simulation
 * @return Vector of attack behavior patterns
 */
std::vector<AttackerBehaviorPattern> CreateAttackerBehaviorPatterns(
    const std::vector<std::string>& attackerVehicles, uint32_t numEvents);

} // namespace ns3

#endif // ATTACKER_BEHAVIOR_PATTERNS_HPP