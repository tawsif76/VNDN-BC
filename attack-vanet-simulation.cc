#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/command-line.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/netanim-module.h"  
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
// Custom Application Headers
#include "VanetBlockchainApp.hpp"
#include "vanet-vehicle-app.hpp"
#include "MetricsCollector.hpp"
#include "AttackerBehaviorPatterns.hpp" 
// WiFi Setup Helper
#include "ns3/wifi-setup-helper.h"
#include "ns3/wifi-adhoc-helper.h"
#include "ns3/multicast-vanet-strategy.h"

#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>

NS_LOG_COMPONENT_DEFINE("VanetSimulationLarge");

using namespace ns3;

// RSU position structure
struct RSUPosition {
    uint32_t id;
    Vector position;
    std::string name;
};

// Vehicle assignment structure
struct VehicleAssignment {
    uint32_t vehicleId;
    uint32_t assignedRsuId;
    std::string assignedRsuName;
    double distance;
};

struct SimulationScenario {
    uint32_t vehicleCount;
    double attackerPercentage;
    uint32_t runId;
    std::string scenarioName;
};


double CalculateEuclideanDistance(const Vector& pos1, const Vector& pos2) {
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    double dz = pos1.z - pos2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


Ptr<VanetVehicleApp> GetVehicleApp(Ptr<Node> vehicleNode) {
    if (!vehicleNode) return nullptr;
    Ptr<Application> app = vehicleNode->GetApplication(0);
    return DynamicCast<VanetVehicleApp>(app);
}


void SetupFIB(const NodeContainer& vehicleNodes, const NodeContainer& rsuNodes, 
                               const NetDeviceContainer& vehicleWifiDevices, const NetDeviceContainer& rsuWifiDevices,
                               const NetDeviceContainer& rsuCsmaDevices, uint32_t activeRsus, uint32_t numVehicles) {
    
    NS_LOG_INFO("=== ENHANCED FIB SETUP FOR ALL 10 RSUs ===");
    

    for (uint32_t i = 0; i < rsuNodes.GetN(); ++i) {
        Ptr<Node> rsuNode = rsuNodes.Get(i);
        ns3::ndn::StrategyChoiceHelper::Install(rsuNode, "/vanet", "/localhost/nfd/strategy/multicast");
        NS_LOG_INFO("RSU-" << i << ": Set multicast strategy for /vanet (Active: " 
                   << (i < activeRsus ? "YES" : "NO") << ")");
    }
    
    uint32_t totalRsuRoutes = 0;
    
    for (uint32_t i = 0; i < rsuNodes.GetN(); ++i) {
        Ptr<Node> rsuNode = rsuNodes.Get(i);
        std::string rsuName = "RSU-" + std::to_string(i);
        Ptr<ns3::ndn::L3Protocol> l3 = rsuNode->GetObject<ns3::ndn::L3Protocol>();

        if (!l3) {
            NS_LOG_ERROR("RSU " << rsuName << " has no NDN L3 protocol - CRITICAL ERROR!");
            continue;
        }

        NS_LOG_INFO("Setting up FIB for " << rsuName << " (Active: " << (i < activeRsus ? "YES" : "NO") << ")");

        if (i < rsuWifiDevices.GetN()) {
            Ptr<NetDevice> rsuWifiDev = rsuWifiDevices.Get(i);
            auto wifiFace = l3->getFaceByNetDevice(rsuWifiDev);
            if (wifiFace) {
                ns3::ndn::Name servicePrefix("/vanet/" + rsuName);
                ns3::ndn::FibHelper::AddRoute(rsuNode, servicePrefix, wifiFace, 0);
                totalRsuRoutes++;

                ns3::ndn::FibHelper::AddRoute(rsuNode, "/vanet", wifiFace, 10 + i);
                totalRsuRoutes++;
            
            } else {
                NS_LOG_ERROR(" FAILED to get WiFi face for " << rsuName);
            }
        }

        if (i < rsuCsmaDevices.GetN()) {
            Ptr<NetDevice> rsuCsmaDev = rsuCsmaDevices.Get(i);
            auto csmaFace = l3->getFaceByNetDevice(rsuCsmaDev);
            if (csmaFace) {
                NS_LOG_INFO("  RSU-" << i << " CSMA Face ID: " << csmaFace->getId());
                for (uint32_t j = 0; j < rsuNodes.GetN(); ++j) {
                    if (i != j) {
                        // Main RSU route
                        ns3::ndn::Name otherRsuRoute("/vanet/RSU-" + std::to_string(j));
                        ns3::ndn::FibHelper::AddRoute(rsuNode, otherRsuRoute, csmaFace, 1);
                        totalRsuRoutes++;
                    }
                }
                ns3::ndn::Name pbftRoute("/vanet/pbft");
                ns3::ndn::FibHelper::AddRoute(rsuNode, pbftRoute, csmaFace, 0);
                totalRsuRoutes++;
                
                ns3::ndn::FibHelper::AddRoute(rsuNode, "/vanet", csmaFace, 5);
                totalRsuRoutes++;
                
                NS_LOG_INFO("  CSMA inter-RSU routes added for " << rsuName 
                           << " (routes to " << (rsuNodes.GetN() - 1) << " other RSUs)");
            } else {
                NS_LOG_ERROR(" FAILED to get CSMA face for " << rsuName << " - CRITICAL!");
            }
        }
    }


    uint32_t vehiclesWithRoutes = 0;
    for (uint32_t i = 0; i < numVehicles; ++i) {
        Ptr<Node> vehicleNode = vehicleNodes.Get(i);
        std::string vehicleName = "V-" + std::to_string(i);
        
        Ptr<ns3::ndn::L3Protocol> l3 = vehicleNode->GetObject<ns3::ndn::L3Protocol>();
        if (!l3) {
            NS_LOG_ERROR("Vehicle " << vehicleName << " has no NDN L3 protocol!");
            continue;
        }

        if (i < vehicleWifiDevices.GetN()) {
            Ptr<NetDevice> vehicleWifiDev = vehicleWifiDevices.Get(i);
            auto wifiFace = l3->getFaceByNetDevice(vehicleWifiDev);
            if (wifiFace) {
                // Primary route to VANET services
                ns3::ndn::FibHelper::AddRoute(vehicleNode, "/vanet", wifiFace, 1);
                vehiclesWithRoutes++;
                
                for (uint32_t r = 0; r < rsuNodes.GetN(); ++r) {
                    ns3::ndn::Name rsuRoute("/vanet/RSU-" + std::to_string(r));
                    uint32_t priority = (r == (i % rsuNodes.GetN())) ? 0 : (r + 1);
                    ns3::ndn::FibHelper::AddRoute(vehicleNode, rsuRoute, wifiFace, priority);
                }
                
                NS_LOG_INFO("Vehicle " << vehicleName << " routes configured for all 10 RSUs");
            } else {
                NS_LOG_ERROR("Vehicle " << vehicleName << " FAILED to get WiFi face!");
            }
        }
    }
}


std::vector<VehicleAssignment> AssignVehiclesEquallyToAllRSUs(const NodeContainer& vehicleNodes, 
                                                             const std::vector<RSUPosition>& allRSUs) {
    std::vector<VehicleAssignment> assignments;
    
    NS_LOG_INFO("=== EQUAL DISTRIBUTION: Vehicle Assignment Across All " << allRSUs.size() << " RSUs ===");
    
    for (uint32_t v = 0; v < vehicleNodes.GetN(); ++v) {
        Ptr<Node> vehicleNode = vehicleNodes.Get(v);
        Ptr<MobilityModel> mobilityModel = vehicleNode->GetObject<MobilityModel>();
        Vector vehiclePos = mobilityModel->GetPosition();
        
    
        uint32_t assignedRsuIndex = v % allRSUs.size();
        const RSUPosition& assignedRsu = allRSUs[assignedRsuIndex];
        

        double distance = CalculateEuclideanDistance(vehiclePos, assignedRsu.position);
        
        VehicleAssignment assignment;
        assignment.vehicleId = v;
        assignment.assignedRsuId = assignedRsu.id;
        assignment.assignedRsuName = assignedRsu.name;
        assignment.distance = distance;
        assignments.push_back(assignment);
        
        NS_LOG_INFO("Vehicle V-" << v << " at (" << vehiclePos.x << "," << vehiclePos.y 
                    << ") → " << assignedRsu.name << " (Round-robin assignment)");
    }
    

    std::map<std::string, uint32_t> rsuVehicleCount;
    for (const auto& assignment : assignments) {
        rsuVehicleCount[assignment.assignedRsuName]++;
    }
    
    NS_LOG_INFO("=== DISTRIBUTION SUMMARY ===");
    for (const auto& pair : rsuVehicleCount) {
        NS_LOG_INFO(pair.first << ": " << pair.second << " vehicles");
    }
    NS_LOG_INFO("Total RSUs in use: " << rsuVehicleCount.size());
    
    return assignments;
}

std::string SelectPrimaryRsuForEvent(const std::string& eventLocation, 
                                   const std::vector<RSUPosition>& activeRSUs) {
    size_t underscorePos = eventLocation.find('_');
    if (underscorePos == std::string::npos) {
        NS_LOG_WARN("Invalid event location format: " << eventLocation << ", using RSU-0");
        return "RSU-0"; // Default fallback
    }
    
    double eventX = std::stod(eventLocation.substr(0, underscorePos));
    double eventY = std::stod(eventLocation.substr(underscorePos + 1));
    Vector eventPos(eventX, eventY, 0.0);
    
    // Find nearest RSU to event location
    std::string nearestRsu = "RSU-0";
    double minDistance = std::numeric_limits<double>::max();
    
    for (const auto& rsu : activeRSUs) {
        double distance = CalculateEuclideanDistance(eventPos, rsu.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestRsu = rsu.name;
        }
    }
    
    NS_LOG_INFO("*** Event at " << eventLocation << " assigned to " << nearestRsu 
               << " (distance: " << std::fixed << std::setprecision(1) << minDistance << "m) ***");
    return nearestRsu;
}

void ScheduleLocationQueries(const NodeContainer& vehicleNodes, uint32_t numVehicles,
                           const std::vector<std::string>& eventLocations,
                           const std::vector<RSUPosition>& activeRSUPositions,
                           double queryStartTime, bool in) {
    uint32_t numQueryingVehicles = numVehicles * 0.95;  
    uint32_t queriesPerVehicle = 1;  
    uint32_t totalQueryRounds = 3; 
    if (!in) queriesPerVehicle = 3, totalQueryRounds = 3;
    
    NS_LOG_INFO("=== INTENSIVE PARALLEL LOCATION QUERIES ===");
    NS_LOG_INFO("Querying vehicles: " << numQueryingVehicles << " (100%)");
    NS_LOG_INFO("Queries per vehicle: " << queriesPerVehicle);
    NS_LOG_INFO("Query rounds: " << totalQueryRounds);
    NS_LOG_INFO("Total queries: " << (numQueryingVehicles * queriesPerVehicle * totalQueryRounds));
    NS_LOG_INFO("PARALLEL with event reporting for batch processing stress test");

    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    
 
    uint32_t totalQueriesScheduled = 0;
    
    // Schedule multiple rounds of intensive queries
    for (uint32_t round = 0; round < totalQueryRounds; ++round) {
        for (uint32_t i = 0; i < numQueryingVehicles; ++i) {
            Ptr<VanetVehicleApp> vehApp = GetVehicleApp(vehicleNodes.Get(i));
            
            if (vehApp) {
                for (uint32_t q = 0; q < queriesPerVehicle; ++q) {
                    
                    double queryDelay = round * 0.1 + (i * 0.01); 
                    
                    std::string queryLocation = eventLocations[rand->GetInteger(0, eventLocations.size() - 1)];
               
                    std::string targetRsu;
                    
             
                    if (rand->GetValue(0.0, 1.0) < 0.7) {
                
                        targetRsu = SelectPrimaryRsuForEvent(queryLocation, activeRSUPositions);
                    } else {
                   
                        uint32_t randomRsuIndex = rand->GetInteger(0, activeRSUPositions.size() - 1);
                        targetRsu = activeRSUPositions[randomRsuIndex].name;
                    }
                    
                    std::string vehicleId = "V-" + std::to_string(i);

                    Simulator::Schedule(Seconds(queryDelay),
                                      &VanetVehicleApp::ScheduleLocationQuery, vehApp,
                                      queryLocation, targetRsu);
                    
                    totalQueriesScheduled++;
                }
            }
        }
        
    }
}

void RunSimulation(uint32_t runId, double attackerPercentage, uint32_t numVehicles,
                            NodeContainer& vehicleNodes, NodeContainer& rsuNodes, 
                            const std::vector<VehicleAssignment>& vehicleAssignments,
                            Ptr<MetricsCollector> metricsCollector, uint32_t activeRsus, 
                            const std::vector<RSUPosition>& activeRSUPositions)
{

    
    uint32_t numAttackers = static_cast<uint32_t>(numVehicles * attackerPercentage);

    metricsCollector->SetSimulationContext(numVehicles, numAttackers, Simulator::Now());
 
    metricsCollector->SchedulePeriodicReporting(Seconds(120.0));

    std::vector<uint32_t> attackerIndices;
    for (uint32_t i = 0; i < numAttackers; ++i) {
        attackerIndices.push_back(numVehicles - 1 - i);
    }
    
    for (uint32_t i = 0; i < numVehicles; ++i) {
        Ptr<VanetVehicleApp> vehApp = GetVehicleApp(vehicleNodes.Get(i));
        if (vehApp) {
            bool isAttacker = std::find(attackerIndices.begin(), attackerIndices.end(), i) != attackerIndices.end();
            vehApp->SetAttackerStatus(isAttacker);
        }
    }


    for (uint32_t i = 0; i < activeRsus; ++i) {
        Ptr<VanetBlockchainApp> rsuApp = DynamicCast<VanetBlockchainApp>(rsuNodes.Get(i)->GetApplication(0));
        if (rsuApp) {
            rsuApp->SetTotalVehicles(numVehicles);
            rsuApp->SetTotalAttackers(numAttackers);
            for (uint32_t v = 0; v < numVehicles; ++v) {
                std::string vehicleId = "V-" + std::to_string(v);
                bool isBaseAttacker = std::find(attackerIndices.begin(), attackerIndices.end(), v) != attackerIndices.end();
                rsuApp->m_vehicleBaseAttackerStatus[vehicleId] = isBaseAttacker;
                
                NS_LOG_DEBUG("RSU-" << i << " set base status for " << vehicleId 
                           << ": " << (isBaseAttacker ? "ATTACKER" : "HONEST"));
            }
        }
    }

    std::vector<std::string> attackerVehicleIds;
    for (uint32_t idx : attackerIndices) {
        attackerVehicleIds.push_back("V-" + std::to_string(idx));
    }
    
    uint32_t scaledEventCount;
    uint32_t scaledWitnessGroupSize;
    
    if (numVehicles <= 50) {
        scaledEventCount = 45;
        scaledWitnessGroupSize = 45;
    } else if (numVehicles <= 75) {
        scaledEventCount = 50;
        scaledWitnessGroupSize = 47;
    } else if (numVehicles <= 100) {
        scaledEventCount = 55;
        scaledWitnessGroupSize = 49;
    } else if (numVehicles <= 125) {
        scaledEventCount = 60;
        scaledWitnessGroupSize = 51;
    } else if (numVehicles <= 150) {
        scaledEventCount = 65;
        scaledWitnessGroupSize = 53;
    } 

    std::vector<AttackerBehaviorPattern> attackerPatterns = CreateAttackerBehaviorPatterns(attackerVehicleIds, scaledEventCount);
    

    for (const auto& pattern : attackerPatterns) {
        metricsCollector->SetVehicleGroundTruth(pattern.vehicleId, true, pattern.patternType);
    }
    
    for (uint32_t v = 0; v < numVehicles; ++v) {
        std::string vehicleId = "V-" + std::to_string(v);
        bool isInAttackerList = std::find(attackerVehicleIds.begin(), attackerVehicleIds.end(), vehicleId) != attackerVehicleIds.end();
        if (!isInAttackerList) {
            metricsCollector->SetVehicleGroundTruth(vehicleId, false, "HONEST");
        }
    }


    double registrationStartTime = 5.0;      
    double registrationInterval = 0.005;   
    
    for (uint32_t i = 0; i < numVehicles; ++i) {
        Ptr<VanetVehicleApp> vehApp = GetVehicleApp(vehicleNodes.Get(i));
        if (vehApp) {
            Simulator::Schedule(Seconds(registrationStartTime + i * registrationInterval),
                                &VanetVehicleApp::ScheduleRegistrationRequest, vehApp);
        }
    }


    std::vector<std::string> eventTypes = {"Accident", "Jam", "Roadwork", "Construction", "Breakdown"};
    std::vector<std::string> eventLocations = {
        "400_1200", "570_1210", "410_1060", "575_1070", "420_925",
        "350_1150", "600_1180", "450_1100", "520_980", "380_850",
        "300_1050", "650_1150", "480_1200", "550_950", "360_800",
        "700_1100", "320_1250", "580_1020", "440_850", "600_950"
    };

    std::vector<Ptr<Node>> honestNodes;
    std::vector<Ptr<Node>> attackerNodes;
    for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i) {
        Ptr<Node> node = vehicleNodes.Get(i);
        if (node) {
            Ptr<VanetVehicleApp> vehApp = GetVehicleApp(node);
            if (vehApp) {
                bool isAttacker = std::find(attackerIndices.begin(), attackerIndices.end(), i) != attackerIndices.end();
                if (isAttacker) {
                    attackerNodes.push_back(node);
                } else {
                    honestNodes.push_back(node);
                }
            }
        }
    }
    

    std::map<std::string, uint32_t> attackerParticipationCount;
    for (const auto& vehicleId : attackerVehicleIds) {
        attackerParticipationCount[vehicleId] = 0;
    }
    const uint32_t ATTACKER_REPORT_QUOTA = 10;


    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    double firstBatchTime = 12;
    double timeBetweenBatches = 4;
    NS_LOG_INFO("RSUs will learn attacker patterns from " << scaledEventCount << " scaled events!");

    for (uint32_t batchNum = 0; batchNum < scaledEventCount; ++batchNum) {
        std::string groundTruthEventTypeForBatch = eventTypes[rand->GetInteger(0, eventTypes.size() - 1)];
        std::string groundTruthLocationForBatch = eventLocations[rand->GetInteger(0, eventLocations.size() - 1)];
        double currentBatchBaseReportTime = firstBatchTime + (batchNum * timeBetweenBatches);
        Time actualEventOccurTime = Seconds(currentBatchBaseReportTime - 5.0);
        std::string primaryRsuForEvent = SelectPrimaryRsuForEvent(groundTruthLocationForBatch, activeRSUPositions);

        NS_LOG_INFO("--- Event Batch " << (batchNum + 1) << "/" << scaledEventCount << " ---");
        
        std::vector<Ptr<Node>> witnessGroup;

        if (batchNum == 5 || batchNum == 15) { 
            std::random_shuffle(attackerNodes.begin(), attackerNodes.end());
            for (uint32_t i = 0; i < scaledWitnessGroupSize && i < attackerNodes.size(); ++i) {
                witnessGroup.push_back(attackerNodes[i]);
            }
        } else {
            uint32_t numAttackersInGroup = std::round(scaledWitnessGroupSize * attackerPercentage);
            uint32_t numHonestInGroup = scaledWitnessGroupSize - numAttackersInGroup;

            std::vector<Ptr<Node>> availableAttackers;
            for (const auto& node : attackerNodes) {
                if (node) {
                    if (attackerParticipationCount[Names::FindName(node)] < ATTACKER_REPORT_QUOTA) {
                        availableAttackers.push_back(node);
                    }
                }
            }
            if (availableAttackers.size() < numAttackersInGroup) {
                for (const auto& node : attackerNodes) {
                    if (node && (std::find(availableAttackers.begin(), availableAttackers.end(), node) == availableAttackers.end())) {
                        availableAttackers.push_back(node);
                    }
                }
            }
            std::random_shuffle(availableAttackers.begin(), availableAttackers.end());

            for (uint32_t i = 0; i < numAttackersInGroup && i < availableAttackers.size(); ++i) {
                Ptr<Node> selectedNode = availableAttackers[i];
                if (selectedNode) {
                    witnessGroup.push_back(selectedNode);
                    attackerParticipationCount[Names::FindName(selectedNode)]++;
                }
            }
            std::random_shuffle(honestNodes.begin(), honestNodes.end());
            if (!honestNodes.empty()) {
                for (uint32_t i = 0; i < numHonestInGroup; ++i) {
                    witnessGroup.push_back(honestNodes[i % honestNodes.size()]);
                }
            }
        }


        NS_LOG_INFO("  Created Witness Group: " << witnessGroup.size() << " vehicles");
        
        std::map<std::string, bool> attackerBehaviorThisEvent;
        for (const auto& pattern : attackerPatterns) {
            bool shouldAttack = (batchNum < pattern.attackSchedule.size()) ? 
                            pattern.attackSchedule[batchNum] : false;
            attackerBehaviorThisEvent[pattern.vehicleId] = shouldAttack;
        }


        for (uint32_t i = 0; i < witnessGroup.size(); ++i) {
            Ptr<Node> node = witnessGroup[i];
            Ptr<VanetVehicleApp> vehApp = GetVehicleApp(node);

            if (vehApp) {
                double maxStaggerTime = timeBetweenBatches * 0.45;
                double staggerDelay = (witnessGroup.size() > 0) ? (maxStaggerTime / witnessGroup.size()) : 0.1;
                double individualStaggerDelay = i * staggerDelay;
                double scheduledReportTime = currentBatchBaseReportTime + individualStaggerDelay;
                
                std::string vehicleName = Names::FindName(node);
                bool willActMalicious = false;
                if (attackerBehaviorThisEvent.count(vehicleName)) {
                    willActMalicious = attackerBehaviorThisEvent[vehicleName];
                }

                std::string finalReportType = groundTruthEventTypeForBatch;
                bool isHonestVehicle = !vehApp->IsCurrentlyAttacker();

                if (isHonestVehicle && rand->GetValue(0.0, 1.0) < 0.02) { 
                    if (groundTruthEventTypeForBatch == "Accident") finalReportType = "Breakdown";
                    else finalReportType = "Accident";
                }

                if (willActMalicious) {
                    auto it = std::find_if(attackerPatterns.begin(), attackerPatterns.end(),
                                           [&](const AttackerBehaviorPattern& p) { return p.vehicleId == vehicleName; });

                    if (it != attackerPatterns.end() && it->patternType == "COLLUDING_LIAR") {
                        finalReportType = "Construction";
                        if (groundTruthEventTypeForBatch == "Construction") finalReportType = "Jam";
                    } else {
                        finalReportType = "Roadwork";
                        if (groundTruthEventTypeForBatch == "Roadwork") finalReportType = "Breakdown";
                    }
                }
                
                vehApp->SetBehaviorForEvent(batchNum, willActMalicious);
                vehApp->SetCurrentEventIndex(batchNum);
                
                Simulator::Schedule(Seconds(scheduledReportTime),
                                    &VanetVehicleApp::ScheduleEventReportToSpecificRsu, vehApp,
                                    finalReportType,
                                    groundTruthLocationForBatch,
                                    actualEventOccurTime,
                                    primaryRsuForEvent);
            }
        }
    }
}

void RunSingleScenario(const SimulationScenario& scenario, 
                      const std::vector<RSUPosition>& allRSUs,
                      const std::vector<RSUPosition>& activeRSUPositions,
                      uint32_t activeRsus, double simulationTime, 
                      const std::string& mobilityTraceFile) {
    
    uint32_t numVehicles = scenario.vehicleCount;
    double attackerPercentage = scenario.attackerPercentage;
    uint32_t runId = scenario.runId;
    uint32_t numRsus = allRSUs.size();
    
    // Create metrics collector
    Ptr<MetricsCollector> metricsCollector = CreateObject<MetricsCollector>();
    uint32_t numAttackers = static_cast<uint32_t>(numVehicles * attackerPercentage);
    metricsCollector->SetSimulationContext(numVehicles, numAttackers, Seconds(0));
    
    // Node Creation
    NodeContainer vehicleNodes;
    vehicleNodes.Create(numVehicles);
    NodeContainer rsuNodes;
    rsuNodes.Create(numRsus);
    NodeContainer allNodes;
    allNodes.Add(vehicleNodes);
    allNodes.Add(rsuNodes);

    NS_LOG_INFO("Created " << numVehicles << " vehicle nodes and " << numRsus << " RSU nodes");

    // Mobility Setup
    Ns2MobilityHelper ns2Mobility(mobilityTraceFile);
    ns2Mobility.Install(vehicleNodes.Begin(), vehicleNodes.End());

    MobilityHelper rsuMobilityHelper;
    rsuMobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    rsuMobilityHelper.Install(rsuNodes);
    
    for (uint32_t i = 0; i < numRsus; ++i) {
        rsuNodes.Get(i)->GetObject<ConstantPositionMobilityModel>()->SetPosition(allRSUs[i].position);
    }

    CsmaHelper csmaHelper;
    csmaHelper.SetChannelAttribute("DataRate", StringValue("100Mbps"));    
    csmaHelper.SetChannelAttribute("Delay", TimeValue(NanoSeconds(1000))); 
    NetDeviceContainer rsuCsmaDevices = csmaHelper.Install(rsuNodes);
    
    ns3::ndn::WifiSetupHelper wifiSetupHelper;
    NetDeviceContainer allWifiDevices = wifiSetupHelper.ConfigureDevices(allNodes, false);
    
    NetDeviceContainer vehicleWifiDevices;
    NetDeviceContainer rsuWifiDevices;
    for (uint32_t i = 0; i < numVehicles; ++i) {
        vehicleWifiDevices.Add(allWifiDevices.Get(i));
    }
    for (uint32_t i = 0; i < numRsus; ++i) {
        rsuWifiDevices.Add(allWifiDevices.Get(numVehicles + i));
    }

    // NDN Stack Installation
    ns3::ndn::StackHelper ndnHelper;
    ndnHelper.setCsSize(10000); 
    ndnHelper.InstallAll();


    ApplicationContainer rsuApps;
    std::vector<std::string> rsuNodeNames;

    for (uint32_t i = 0; i < activeRsus; ++i) {
        std::string rsuName = "RSU-" + std::to_string(i);
        Names::Add(rsuName, rsuNodes.Get(i));
        rsuNodeNames.push_back(rsuName);

        Ptr<VanetBlockchainApp> rsuApp = CreateObject<VanetBlockchainApp>();
        rsuApp->SetAttribute("NodeName", StringValue(rsuName));
        rsuApp->SetNodeType(RSU_VALIDATOR);
        rsuApp->SetMetricsCollector(metricsCollector); 
        rsuApp->SetTotalVehicles(numVehicles);        
        rsuApp->SetTotalAttackers(numAttackers);         
        rsuNodes.Get(i)->AddApplication(rsuApp);
        rsuApps.Add(rsuApp);
    }
    
    for (uint32_t i = 0; i < activeRsus; ++i) {
        DynamicCast<VanetBlockchainApp>(rsuApps.Get(i))->SetRsuList(rsuNodeNames);
    }

    // Vehicle Application Installation
    ApplicationContainer vehicleApps;
    for (uint32_t i = 0; i < numVehicles; ++i) {
        std::string vehicleName = "V-" + std::to_string(i);
        Names::Add(vehicleName, vehicleNodes.Get(i));

        Ptr<VanetVehicleApp> vehApp = CreateObject<VanetVehicleApp>();
        vehApp->SetAttribute("VehicleID", StringValue(vehicleName));
        vehApp->SetTargetRsuName("RSU-0");
        vehApp->SetMetricsCollector(metricsCollector);
        vehicleNodes.Get(i)->AddApplication(vehApp);
        vehicleApps.Add(vehApp);
    }

    ns3::ndn::StrategyChoiceHelper::InstallAll("/", "/localhost/nfd/strategy/best-route");
    SetupFIB(vehicleNodes, rsuNodes, vehicleWifiDevices, rsuWifiDevices, 
                             rsuCsmaDevices, activeRsus, numVehicles);

    // Start Applications
    rsuApps.Start(Seconds(5.0));
    rsuApps.Stop(Seconds(simulationTime - 1.0));
    vehicleApps.Start(Seconds(10.0));
    vehicleApps.Stop(Seconds(simulationTime - 2.0));

    // Schedule vehicle assignment and simulation
    Simulator::Stop(Seconds(simulationTime));
    
    Simulator::Schedule(Seconds(20.0), [&vehicleNodes, &rsuNodes, &activeRSUPositions, &metricsCollector, 
                                      activeRsus, numVehicles, attackerPercentage, runId, &scenario]() {
        std::vector<VehicleAssignment> vehicleAssignments = AssignVehiclesEquallyToAllRSUs(vehicleNodes, activeRSUPositions);
        
        for (const auto& assignment : vehicleAssignments) {
            Ptr<VanetVehicleApp> vehApp = GetVehicleApp(vehicleNodes.Get(assignment.vehicleId));
            if (vehApp) {
                vehApp->SetTargetRsuName(assignment.assignedRsuName);
            }
        }
        
        RunSimulation(runId, attackerPercentage, numVehicles, 
                               vehicleNodes, rsuNodes, vehicleAssignments, metricsCollector, activeRsus, activeRSUPositions);
    });


    Simulator::Run();
    NS_LOG_INFO("\n=== GENERATING COMPREHENSIVE METRICS ANALYSIS ===");
    
    Ptr<VanetBlockchainApp> leaderRsu = DynamicCast<VanetBlockchainApp>(rsuNodes.Get(0)->GetApplication(0));
    if (leaderRsu) {
        // You'll need to add a getter method for the block processing times
        metricsCollector->SetBlockProcessingTimes(leaderRsu->GetBlockProcessingTimes());
    }

    metricsCollector->GenerateDetectionPerformanceReport();

    std::string scenarioBase = "scratch/Thesis_attack_final/results/superiority_data/" + scenario.scenarioName;
    

    metricsCollector->ExportSuperiorityMetrics(scenarioBase, runId, attackerPercentage, numVehicles);
    

    std::string eventDetectionFile = scenarioBase + "_detailed_event_detections.csv";
    metricsCollector->ExportEventDetectionAnalysis(eventDetectionFile);
    

    std::string adaptiveAnalysisFile = scenarioBase + "_adaptive_detection_analysis.csv";
    metricsCollector->ExportAdaptiveAttackerAnalysis(adaptiveAnalysisFile);
    
    

    std::string summaryFile = "scratch/Thesis_attack_final/results/summary_metrics.csv";
    
    metricsCollector->ExportResults(summaryFile, runId, attackerPercentage);

  
    metricsCollector->Reset();
    Names::Clear();
    Simulator::Destroy();
    
    NS_LOG_INFO("Enhanced scenario " << scenario.scenarioName << " completed with comprehensive metrics");
}



int main(int argc, char* argv[]) {
    PacketMetadata::Enable();


    uint32_t numRsus = 20;
    uint32_t activeRsus = 20; 
    double simulationTime = 2000; 
    std::string mobilityTraceFile = "scratch/Thesis_attack_final/ns2mobility.tcl";
    bool enablePcap = false;
    bool enableNetAnim = false;

    std::vector<uint32_t> vehicleCounts = {};
    std::vector<double> attackerPercentages = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4}; 
    
    // Command line options
    bool runFullAnalysis = true;  // Set to false for quick testing
    uint32_t fixedVehicles = 100; // For attacker-only analysis
    double fixedAttackerPct = 0.3; // For vehicle-count-only analysis
    
    CommandLine cmd;
    cmd.AddValue("simTime", "Total simulation time per run in seconds", simulationTime);
    cmd.AddValue("mobilityTrace", "Path to NS2 mobility trace file", mobilityTraceFile);
    cmd.AddValue("fullAnalysis", "Run full multi-dimensional analysis", runFullAnalysis);
    cmd.AddValue("fixedVehicles", "Fixed vehicle count for attacker analysis", fixedVehicles);
    cmd.AddValue("fixedAttackerPct", "Fixed attacker percentage for vehicle analysis", fixedAttackerPct);
    cmd.Parse(argc, argv);

    // Create results directories
    system("mkdir -p scratch/vanetBlockchain/results");

    // Enable logging
    //  LogComponentEnable("VanetSimulationLarge", LOG_LEVEL_INFO);
    //  LogComponentEnable("VanetBlockchainApp", LOG_LEVEL_INFO);
//     LogComponentEnable("VanetVehicleApp", LOG_LEVEL_INFO);
 //    LogComponentEnable("AdaptiveBatchManager", LOG_LEVEL_INFO);
//     LogComponentEnable("MetricsCollector", LOG_LEVEL_INFO);
//     LogComponentEnable("BehavioralInferenceSystem", LOG_LEVEL_DEBUG);

    LogComponentDisableAll(LOG_ALL);

    // RSU positions
    std::vector<RSUPosition> allRSUs = {
        {0, Vector(470.37, 1266.55, 0.0), "RSU-0"},
        {1, Vector(694.43, 867.11, 0.0), "RSU-1"},
        {2, Vector(659.06, 187.43, 0.0), "RSU-2"},
        {3, Vector(395.63, 993.77, 0.0), "RSU-3"},
        {4, Vector(715.34, 1309.30, 0.0), "RSU-4"},
        {5, Vector(273.15, 1145.37, 0.0), "RSU-5"},
        {6, Vector(495.01, 861.14, 0.0), "RSU-6"},
        {7, Vector(269.38, 854.55, 0.0), "RSU-7"},
        {8, Vector(685.45, 1013.04, 0.0), "RSU-8"},
        {9, Vector(236.04, 1264.37, 0.0), "RSU-9"},
        {10, Vector(511.81, 998.85, 0.0), "RSU-10"},
        {11, Vector(472.41, 1135.91, 0.0), "RSU-11"},
        {12, Vector(72.08, 871.39, 0.0), "RSU-12"},
        {13, Vector(626.52, 1150.23, 0.0), "RSU-13"},
        {14, Vector(737.59, 1165.92, 0.0), "RSU-14"},
        {15, Vector(245.93, 1012.13, 0.0), "RSU-15"},
        {16, Vector(306.65, 435.46, 0.0), "RSU-16"},
        {17, Vector(596.56, 1289.91, 0.0), "RSU-17"},
        {18, Vector(627.12, 537.01, 0.0), "RSU-18"},
        {19, Vector(347.60, 1263.95, 0.0), "RSU-19"}
    };

    std::vector<RSUPosition> activeRSUPositions(allRSUs.begin(), allRSUs.begin() + activeRsus);



    if (runFullAnalysis) {
        NS_LOG_INFO("FULL ANALYSIS MODE: Testing all vehicle counts and attacker percentages");
        std::cerr << "FULL ANALYSIS MODE: Testing all vehicle counts and attacker percentages\n";
        // Create comprehensive simulation scenarios
        std::vector<SimulationScenario> scenarios;
        uint32_t runCounter = 1;
        

        NS_LOG_INFO("Phase 1: Vehicle Count Scaling Analysis");
        for (uint32_t vehicles : vehicleCounts) {
            SimulationScenario scenario;
            scenario.vehicleCount = vehicles;
            scenario.attackerPercentage = fixedAttackerPct; // Fixed at 20%
            scenario.runId = runCounter++;
            scenario.scenarioName = "VehicleScaling_" + std::to_string(vehicles) + "V_" + 
                                   std::to_string(static_cast<int>(fixedAttackerPct * 100)) + "A";
            scenarios.push_back(scenario);
        }
        
        // 2. Attacker percentage analysis (fixed vehicle count)
        NS_LOG_INFO("Phase 2: Attacker Percentage Analysis");
        for (double attackerPct : attackerPercentages) {
            SimulationScenario scenario;
            scenario.vehicleCount = fixedVehicles; // Fixed at 150
            scenario.attackerPercentage = attackerPct;
            scenario.runId = runCounter++;
            scenario.scenarioName = "AttackerScaling_" + std::to_string(fixedVehicles) + "V_" + 
                                   std::to_string(static_cast<int>(attackerPct * 100)) + "A";
            scenarios.push_back(scenario);
        }
        
        NS_LOG_INFO("Total scenarios to run: " << scenarios.size());
        NS_LOG_INFO("Estimated time: " << (scenarios.size() * simulationTime / 60.0) << " minutes");
        
        // Run all scenarios
        for (size_t i = 0; i < scenarios.size(); ++i) {
            const auto& scenario = scenarios[i];
            
            NS_LOG_INFO("\n === SCENARIO " << (i + 1) << "/" << scenarios.size() << " ===");
            NS_LOG_INFO("Name: " << scenario.scenarioName);
            NS_LOG_INFO("Vehicles: " << scenario.vehicleCount);
            NS_LOG_INFO("Attackers: " << (scenario.attackerPercentage * 100) << "%");
            NS_LOG_INFO("Run ID: " << scenario.runId);

            std::cerr << "\n === SCENARIO " << (i + 1) << "/" << scenarios.size() << " ===\n";
            std::cerr << "Name: " << scenario.scenarioName << "\n";
            std::cerr << "Vehicles: " << scenario.vehicleCount << "\n";
            std::cerr << "Attackers: " << (scenario.attackerPercentage * 100) << "%\n";
            std::cerr << "Run ID: " << scenario.runId << "\n";
            
            // Run single scenario
            RunSingleScenario(scenario, allRSUs, activeRSUPositions, activeRsus, 
                             simulationTime, mobilityTraceFile);
            
      
            double progress = (static_cast<double>(i + 1) / scenarios.size()) * 100.0;
            std::cerr << "Scenario completed. Progress: " << progress << "%\n";
            NS_LOG_INFO("Scenario completed. Progress: " << progress << "%");
        }
        
    } else {
        NS_LOG_INFO("QUICK TEST MODE: Running default attacker percentage analysis");
        std::cerr << "QUICK TEST MODE: Running default attacker percentage analysis" << "\n";
        // Quick mode: just run attacker percentage analysis
        for (size_t i = 0; i < attackerPercentages.size(); ++i) {
            SimulationScenario scenario;
            scenario.vehicleCount = fixedVehicles;
            scenario.attackerPercentage = attackerPercentages[i];
            scenario.runId = i + 1;
            scenario.scenarioName = "QuickTest_" + std::to_string(static_cast<int>(attackerPercentages[i] * 100)) + "A";
            
            NS_LOG_INFO("\n === QUICK SCENARIO " << (i + 1) << "/" << attackerPercentages.size() << " ===");
            RunSingleScenario(scenario, allRSUs, activeRSUPositions, activeRsus, 
                             simulationTime, mobilityTraceFile);
        }
    }

    NS_LOG_INFO("\n === ALL SIMULATION SCENARIOS COMPLETED ===");
    
    return 0;
}


