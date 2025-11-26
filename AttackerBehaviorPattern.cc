#include "AttackerBehaviorPatterns.hpp"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AttackerBehaviorPatterns");

std::vector<AttackerBehaviorPattern> CreateAttackerBehaviorPatterns(
    const std::vector<std::string>& attackerVehicles, uint32_t numEvents) {
    
    std::vector<AttackerBehaviorPattern> patterns;
    
    NS_LOG_INFO("=== CREATING ADAPTIVE ATTACKER BEHAVIOR PATTERNS ===");
    
    for (size_t i = 0; i < attackerVehicles.size(); ++i) {
        AttackerBehaviorPattern pattern;
        pattern.vehicleId = attackerVehicles[i];
        pattern.currentEventIndex = 0;
        pattern.totalParticipations = 0;
        pattern.attackSchedule.resize(numEvents);
        
        uint32_t patternIndex = i % 5;
        
        switch (patternIndex) {
            case 0: // ALWAYS ATTACK
                pattern.patternType = "ALWAYS_ATTACK";
                pattern.description = "Always sends false reports";
                for (uint32_t e = 0; e < numEvents; ++e) {
                    pattern.attackSchedule[e] = true;
                }
                break;
                
            case 1: // ON-OFF PATTERN
                pattern.patternType = "ON_OFF";
                pattern.description = "Alternates between attack and honest behavior";
                for (uint32_t e = 0; e < numEvents; ++e) {
                    pattern.attackSchedule[e] = (e % 2 == 0);
                }
                break;
                
            case 2: // BURST ATTACK
                pattern.patternType = "BURST_ATTACK";
                pattern.description = "Attacks in bursts: 3 attacks, 2 honest, repeat";
                for (uint32_t e = 0; e < numEvents; ++e) {
                    uint32_t cyclePos = e % 5;
                    pattern.attackSchedule[e] = (cyclePos < 3);
                }
                break;
                
            case 3: // GRADUAL REFORM
                pattern.patternType = "GRADUAL_REFORM";
                pattern.description = "Starts as attacker, gradually reforms to honest";
                for (uint32_t e = 0; e < numEvents; ++e) {
                    double reformProgress = static_cast<double>(e) / numEvents;
                    pattern.attackSchedule[e] = (reformProgress < 0.7);
                }
                break;
                
            case 4: // STRATEGIC DECEIVER
                pattern.patternType = "STRATEGIC_DECEIVER";
                pattern.description = "Mostly honest with strategic false reports";
                for (uint32_t e = 0; e < numEvents; ++e) {
                    pattern.attackSchedule[e] = (e == 4 || e == 9 || e == 19 || e == 24);
                }
                break;
        }
        
        patterns.push_back(pattern);
        
        std::cerr << "Attacker " << pattern.vehicleId << " (" << pattern.patternType << "): " << pattern.description << std::endl;
    }
    
    return patterns;
}

} // namespace ns3