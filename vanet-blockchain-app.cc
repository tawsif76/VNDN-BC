#include "VanetBlockchainApp.hpp"
#include "vanet-vehicle-app.hpp"
#include "VanetBlock.hpp"
#include "AttackerBehaviorPatterns.hpp"
#include "AdaptiveBatchManager.hpp"
#include "MetricsCollector.hpp"

#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/mgmt/fib-manager.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp" // For KeyChain
#include "ns3/random-variable-stream.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h" // Needed for Vector and MobilityModelsssss
#include <cmath> // For sqrt, std::abs
#include <ndn-cxx/encoding/block-helpers.hpp> // For makeBinaryBlock
#include <ndn-cxx/encoding/tlv.hpp>
#include <ndn-cxx/name.hpp>                 
#include <ndn-cxx/interest.hpp>             
#include <ndn-cxx/data.hpp>                 
#include <ndn-cxx/encoding/buffer.hpp>
#include <sstream> // Required for std::stringstream
#include <iomanip> // Required for std::fixed, std::setprecision
#include "ns3/node-list.h"



namespace ns3 {

NS_LOG_COMPONENT_DEFINE("VanetBlockchainApp");
NS_OBJECT_ENSURE_REGISTERED(VanetBlockchainApp);


uint32_t GetNonce() {
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    return rand->GetValue(0, std::numeric_limits<uint32_t>::max());
}

TypeId VanetBlockchainApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::VanetBlockchainApp")
        .SetParent<ndn::App>()
        .AddAttribute("NodeName", "The NDN name of this RSU node.",
                      StringValue("RSU-0"),
                      MakeStringAccessor(&VanetBlockchainApp::m_nodeName),
                      MakeStringChecker())
        .AddAttribute("DistanceThreshold", "Max distance for event clustering (m).",
                      DoubleValue(50.0),
                      MakeDoubleAccessor(&VanetBlockchainApp::m_distanceThreshold),
                      MakeDoubleChecker<double>())
        .AddAttribute("TimeThreshold", "Max time difference for event clustering (s).",
                      TimeValue(Seconds(30.0)), // Use TimeValue and ns3::Seconds()
                      MakeTimeAccessor(&VanetBlockchainApp::m_timeThreshold), // Use MakeTimeAccessor
                      MakeTimeChecker()) // Use MakeTimeChecker
        .AddAttribute("ClusterCheckDelay", "Delay after first report to check cluster (s).",
                      TimeValue(Seconds(5.0)),    // Use TimeValue and ns3::Seconds()
                      MakeTimeAccessor(&VanetBlockchainApp::m_clusterCheckDelay), // Use MakeTimeAccessor
                      MakeTimeChecker()) // Use MakeTimeChecker
         .AddAttribute("ThetaHigh", "High credibility threshold.",
                      DoubleValue(0.75),
                      MakeDoubleAccessor(&VanetBlockchainApp::m_thetaHigh),
                      MakeDoubleChecker<double>())
        .AddAttribute("ThetaLow", "Low credibility threshold.",
                      DoubleValue(0.25),
                      MakeDoubleAccessor(&VanetBlockchainApp::m_thetaLow),
                      MakeDoubleChecker<double>())
        .AddAttribute("MinWitnesses", "Minimum witnesses for event decision.",
                      UintegerValue(2),
                      MakeUintegerAccessor(&VanetBlockchainApp::m_nMin),
                      MakeUintegerChecker<uint32_t>())
        .AddAttribute("Alpha", "Reputation reward rate.",
                      DoubleValue(0.08),
                      MakeDoubleAccessor(&VanetBlockchainApp::m_alpha),
                      MakeDoubleChecker<double>())
        .AddAttribute("Beta", "Reputation penalty rate.",
                      DoubleValue(0.12),
                      MakeDoubleAccessor(&VanetBlockchainApp::m_beta),
                      MakeDoubleChecker<double>())
        .AddConstructor<VanetBlockchainApp>();
    return tid;
}

VanetBlockchainApp::VanetBlockchainApp()
    : m_nodeType(VEHICLE_CLIENT),
      m_n_rsus(0),
      m_f_rsus(0),
      m_pbftCurrentView(0),
      m_pbftCurrentSeqNum(0),
      m_pbftCurrentProposerIndex(0),
      m_lastProposedHeight(0),
      m_distanceThreshold(50.0),
      m_timeThreshold(Seconds(10.0)),
      m_clusterCheckDelay(Seconds(5.0)),
      m_thetaHigh(0.70),                  // TUNED: Increased high threshold (was 0.75)
      m_thetaLow(0.40),                   // TUNED: Increased low threshold (was 0.25) -> Widens "Uncertain" gap
      m_nMin(3),                          // TUNED: Require at least 3 witnesses (was 2)
      m_alpha(0.05),                      // TUNED: Slower reward rate (was 0.08)
      m_beta(0.35),                       // TUNED: Slower penalty rate (was 0.12)
      m_totalEventReportsProcessed(0),
      m_totalRegistrationsProcessed(0),
      m_lastBlockProposalTime(Seconds(0)),
      m_adaptiveBatchManager(this),
      m_totalVehicles(0),
      m_totalAttackers(0),
      m_communicationData(),
      m_lastTpsCalculationTime(Seconds(0)),
      m_transactionsSinceLastTps(0),
      m_currentTps(0.0),
      m_totalBlocksProposed(0),
      m_totalTransactionsProcessed(0),
      m_simulationStartTime(Seconds(0)),
      m_blockProcessingTimes() 
{
    // Initialize genesis block
    VanetBlock genesisBlock;
    genesisBlock.height = 0;
    genesisBlock.timestamp = 0;
    genesisBlock.previousHash = std::string(64, '0');
    genesisBlock.proposerId = "Genesis";
    genesisBlock.blockHash = genesisBlock.calculateHash();
    m_localBlockchain.push_back(genesisBlock);
    
    m_vehicleKeys.clear();
    m_vehicleReputations.clear();
    
    // Initialize adaptive learning tracking
    m_vehicleTotalReports.clear();
    m_vehicleCorrectReports.clear();
    
    // INITIALIZE NEW MEMBER VARIABLES:
    m_vehicleBaseAttackerStatus.clear();
    m_vehicleCurrentBehavior.clear();
    m_vehicleCurrentEventIndex.clear();
    m_tpsHistory.clear();
}


void VanetBlockchainApp::SetRsuList(const std::vector<std::string>& rsuList) {
    m_rsuList = rsuList;
    m_n_rsus = m_rsuList.size();
    if (m_n_rsus > 0) {
        m_f_rsus = (m_n_rsus - 1) / 3;
    } else {
        m_f_rsus = 0;
    }
    NFD_LOG_INFO("[" << m_nodeName << "] RSU List set. Total RSUs: " << m_n_rsus << ", Max Faulty (f): " << m_f_rsus);
}

void VanetBlockchainApp::SetNodeType(NodeType type) {
    m_nodeType = type;
}

// --- Start/Stop Application ---
void VanetBlockchainApp::StartApplication() {
    ndn::App::StartApplication();

    m_lastTpsCalculationTime = Seconds(0);
    m_transactionsSinceLastTps = 0;
    m_currentTps = 0.0;
    m_totalBlocksProposed = 0;
    m_simulationStartTime = Simulator::Now();

    if (m_nodeType == RSU_VALIDATOR) {
        if (m_nodeName == "RSU-0") {
            NFD_LOG_INFO("[" << m_nodeName << "] *** LEADER RSU STARTED *** Node ID: " << GetNode()->GetId());
            NFD_LOG_INFO("[" << m_nodeName << "] This RSU will propose all blocks and lead consensus");
            Simulator::Schedule(Seconds(60.0), &VanetBlockchainApp::SchedulePeriodicTpsCalculation, this);
        } else {
            NFD_LOG_INFO("[" << m_nodeName << "] Follower RSU Started. Node ID: " << GetNode()->GetId());
            NFD_LOG_INFO("[" << m_nodeName << "] This RSU will validate blocks proposed by RSU-0");
        }


        ndn::Name rsuServicePrefix("/vanet");
        rsuServicePrefix.append(m_nodeName);
        ndn::FibHelper::AddRoute(GetNode(), rsuServicePrefix, m_face, 0);

        ndn::Name pbftPrefix("/vanet/pbft");
        ndn::FibHelper::AddRoute(GetNode(), pbftPrefix, m_face, 0);
        Simulator::Schedule(Seconds(500.0), &VanetBlockchainApp::PrintStatus, this);
    }
}


void VanetBlockchainApp::SchedulePeriodicTpsCalculation() {
    CalculateCurrentTPS();
    

    if (m_metricsCollector && m_nodeName == "RSU-0") {
        Time currentTime = Simulator::Now();
        Time elapsedTime = currentTime - m_simulationStartTime;
        double overallTps = elapsedTime.GetSeconds() > 0 ? 
                           (static_cast<double>(m_totalTransactionsProcessed) / elapsedTime.GetSeconds()) : 0.0;
        
        // Get current adaptive batch size for logging
        uint32_t currentAdaptiveBatchSize = m_adaptiveBatchManager.GetCurrentAdaptiveBatchSize();
        
        NFD_LOG_INFO("[" << m_nodeName << "] ADAPTIVE THROUGHPUT STATUS:");
        NFD_LOG_INFO("  Current TPS: " << m_currentTps);
        NFD_LOG_INFO("  Overall TPS: " << overallTps);
        NFD_LOG_INFO("  Total Transactions: " << m_totalTransactionsProcessed);
        NFD_LOG_INFO("  Total Blocks: " << m_totalBlocksProposed);
        NFD_LOG_INFO("  Current Adaptive Batch Size: " << currentAdaptiveBatchSize);
        NFD_LOG_INFO("  Adaptive Batch Buffer: " << m_adaptiveBatchManager.GetBatchBufferSize());
        NFD_LOG_INFO("  Average Tx/Block: " << (m_totalBlocksProposed > 0 ? 
                     static_cast<double>(m_totalTransactionsProcessed) / m_totalBlocksProposed : 0.0));
    }
    

    Simulator::Schedule(Seconds(60.0), &VanetBlockchainApp::SchedulePeriodicTpsCalculation, this);
}


void VanetBlockchainApp::PrintStatus() {
    std::string roleStr = (m_nodeName == "RSU-0") ? "LEADER" : "FOLLOWER";
    
    NFD_LOG_INFO("[" << m_nodeName << "] === " << roleStr << " Enhanced Status Update ===");
    NFD_LOG_INFO("  Blockchain Height: " << m_localBlockchain.size() - 1);
    NFD_LOG_INFO("  Registered Vehicles: " << m_vehicleKeys.size());
    NFD_LOG_INFO("  Transaction Pool: " << m_transactionPool.size());
    NFD_LOG_INFO("  Batch Buffer (Leader): " << (m_nodeName == "RSU-0" ? std::to_string(m_adaptiveBatchManager.GetBatchBufferSize()) : "N/A"));
    NFD_LOG_INFO("  Active Consensus: " << m_pbftActiveConsensus.size());
    NFD_LOG_INFO("  Total Event Reports Processed: " << m_totalEventReportsProcessed);
    
    // NEW: Add adaptive batch processing debug info
    if (m_nodeName == "RSU-0") {
        uint32_t currentAdaptiveBatchSize = m_adaptiveBatchManager.GetCurrentAdaptiveBatchSize();
        NFD_LOG_INFO("  === ADAPTIVE BATCH DEBUG INFO ===");
        NFD_LOG_INFO("  Current Adaptive Batch Size: " << currentAdaptiveBatchSize);
        NFD_LOG_INFO("  Current TPS: " << std::fixed << std::setprecision(2) << m_currentTps);
        NFD_LOG_INFO("  Total Transactions Processed: " << m_totalTransactionsProcessed);
        NFD_LOG_INFO("  Total Blocks Proposed: " << m_totalBlocksProposed);
        
        // Calculate overall TPS since simulation start
        Time currentTime = Simulator::Now();
        Time elapsedTime = currentTime - m_simulationStartTime;
        double overallTps = elapsedTime.GetSeconds() > 0 ? 
                           (static_cast<double>(m_totalTransactionsProcessed) / elapsedTime.GetSeconds()) : 0.0;
        NFD_LOG_INFO("  Overall Average TPS: " << std::fixed << std::setprecision(2) << overallTps);
        
        // Additional debug info - show the factors affecting batch size calculation
        // Note: You might want to add getter methods to AdaptiveBatchManager to access these
        NFD_LOG_INFO("  === DABP ALGORITHM FACTORS ===");
        NFD_LOG_INFO("  Simulation Time: " << std::fixed << std::setprecision(1) << currentTime.GetSeconds() << "s");
    }
    
    // Console output for real-time monitoring
    {
        std::cerr << "[" << m_nodeName << "] === " << roleStr << " Enhanced Status Update ===" << std::endl;
        std::cerr << "  Blockchain Height: " << m_localBlockchain.size() - 1 << std::endl;
        std::cerr << "  Registered Vehicles: " << m_vehicleKeys.size() << std::endl;
        std::cerr << "  Transaction Pool: " << m_transactionPool.size() << std::endl;
        std::cerr << "  Batch Buffer (Leader): " << (m_nodeName == "RSU-0" ? std::to_string(m_adaptiveBatchManager.GetBatchBufferSize()) : "N/A") << std::endl;
        std::cerr << "  Active Consensus: " << m_pbftActiveConsensus.size() << std::endl;
        std::cerr << "  Total Event Reports Processed: " << m_totalEventReportsProcessed << std::endl;
        
        // NEW: Console output for adaptive batch and TPS debugging
        if (m_nodeName == "RSU-0") {
            uint32_t currentAdaptiveBatchSize = m_adaptiveBatchManager.GetCurrentAdaptiveBatchSize();
            std::cerr << "  === ADAPTIVE BATCH DEBUG INFO ===" << std::endl;
            std::cerr << "  Current Adaptive Batch Size: " << currentAdaptiveBatchSize << std::endl;
            std::cerr << "  Current TPS: " << std::fixed << std::setprecision(2) << m_currentTps << std::endl;
            std::cerr << "  Total Transactions Processed: " << m_totalTransactionsProcessed << std::endl;
            std::cerr << "  Total Blocks Proposed: " << m_totalBlocksProposed << std::endl;
            
            Time currentTime = Simulator::Now();
            Time elapsedTime = currentTime - m_simulationStartTime;
            double overallTps = elapsedTime.GetSeconds() > 0 ? 
                               (static_cast<double>(m_totalTransactionsProcessed) / elapsedTime.GetSeconds()) : 0.0;
            std::cerr << "  Overall Average TPS: " << std::fixed << std::setprecision(2) << overallTps << std::endl;
            std::cerr << "  Simulation Time: " << std::fixed << std::setprecision(1) << currentTime.GetSeconds() << "s" << std::endl;
        }
    }
    
    if (m_nodeName == "RSU-0") {
        NFD_LOG_INFO("  [LEADER] Ready to propose: " << (m_transactionPool.empty() && m_adaptiveBatchManager.GetBatchBufferSize() == 0 ? "NO" : "YES"));
        if (!m_transactionPool.empty() || m_adaptiveBatchManager.GetBatchBufferSize() > 0) {
            NFD_LOG_INFO("  [LEADER] Will propose block with " << m_transactionPool.size() << " (pool) + " << m_adaptiveBatchManager.GetBatchBufferSize() << " (batch) transactions");
        }
        
        // NEW: Show readiness based on adaptive batch size
        uint32_t currentAdaptiveBatchSize = m_adaptiveBatchManager.GetCurrentAdaptiveBatchSize();
        if (m_adaptiveBatchManager.GetBatchBufferSize() > 0) {
            double batchFillPercentage = (static_cast<double>(m_adaptiveBatchManager.GetBatchBufferSize()) / currentAdaptiveBatchSize) * 100.0;
            NFD_LOG_INFO("  [LEADER] Adaptive Batch Fill: " << std::fixed << std::setprecision(1) << batchFillPercentage << "% (" 
                        << m_adaptiveBatchManager.GetBatchBufferSize() << "/" << currentAdaptiveBatchSize << ")");
            std::cerr << "  [LEADER] Adaptive Batch Fill: " << std::fixed << std::setprecision(1) << batchFillPercentage << "% (" 
                     << m_adaptiveBatchManager.GetBatchBufferSize() << "/" << currentAdaptiveBatchSize << ")" << std::endl;
        }
    }
    
    Simulator::Schedule(Seconds(120.0), &VanetBlockchainApp::PrintStatus, this);
}


void VanetBlockchainApp::StopApplication() {
    NFD_LOG_INFO("[" << m_nodeName << "] App Stopping.");
    m_pbftActiveConsensus.clear();
    
    // FORCE process any remaining adaptive batch on shutdown
    if (m_nodeName == "RSU-0" && m_adaptiveBatchManager.GetBatchBufferSize() > 0) {
        NFD_LOG_INFO("[" << m_nodeName << "] Processing remaining " 
                     << m_adaptiveBatchManager.GetBatchBufferSize() << " transactions in adaptive batch on shutdown");
        m_adaptiveBatchManager.ProcessBatch();
    }
    
    ndn::App::StopApplication();
}


void VanetBlockchainApp::SendData(std::shared_ptr<ndn::Data> data) {
    ndn::StackHelper::getKeyChain().sign(*data);

    NFD_LOG_INFO("[" << m_nodeName << "] Sending Data: " << data->getName());


    if (m_metricsCollector) {
        std::string dataNameStr = data->getName().toUri();
        uint32_t dataSize = 500; 
        
        if (data->getContent().value_size() > 0) {
            dataSize += data->getContent().value_size();
        }
        
        std::string packetType = "Data_Unknown";
        std::string destination = "BROADCAST";
        bool isControl = true;
        

        if (dataNameStr.find("/register/") != std::string::npos && dataNameStr.find("/ack") != std::string::npos) {
            packetType = "Data_RegistrationAck";
            isControl = false;
            // Extract vehicle ID from data name
            size_t registerPos = dataNameStr.find("/register/");
            size_t vehicleStart = registerPos + 10; // Length of "/register/"
            size_t vehicleEnd = dataNameStr.find("/", vehicleStart);
            if (vehicleEnd != std::string::npos && vehicleStart < dataNameStr.length()) {
                destination = dataNameStr.substr(vehicleStart, vehicleEnd - vehicleStart);
            }
        }
        else if (dataNameStr.find("/blockchain/keys/") != std::string::npos) {
            packetType = "Data_BlockchainQuery";
            isControl = false;
            // Extract vehicle ID
            size_t keysPos = dataNameStr.find("/blockchain/keys/");
            size_t vehicleStart = keysPos + 17; // Length of "/blockchain/keys/"
            size_t vehicleEnd = dataNameStr.find("/", vehicleStart);
            if (vehicleEnd != std::string::npos && vehicleStart < dataNameStr.length()) {
                destination = dataNameStr.substr(vehicleStart, vehicleEnd - vehicleStart);
            }
        }
        else if (dataNameStr.find("/blockchain/reputation/") != std::string::npos) {
            packetType = "Data_ReputationQuery";
            isControl = false;
        }
        else if (dataNameStr.find("/blockchain/blocks/") != std::string::npos) {
            packetType = "Data_BlockQuery";
            isControl = false;
        }
        else if (dataNameStr.find("registration-confirmed") != std::string::npos) {
            packetType = "Data_RegistrationConfirmation";
            isControl = false;
        }
    
    }

    m_transmittedDatas(data, this, m_face);
    m_appLink->onReceiveData(*data);

    std::string dataNameStr = data->getName().toUri();

    // Extract vehicle ID from data name (e.g., from "/vanet/RSU-0/blockchain/keys/V-1/...")
    // if (dataNameStr.find("/blockchain/keys/") != std::string::npos) {
    //     size_t keysPos = dataNameStr.find("/blockchain/keys/");
    //     size_t vehicleStart = keysPos + 17; // Length of "/blockchain/keys/"
    //     size_t vehicleEnd = dataNameStr.find("/", vehicleStart);
    //     if (vehicleEnd == std::string::npos) vehicleEnd = dataNameStr.find("%", vehicleStart);

    //     if (vehicleStart < dataNameStr.length() && vehicleEnd > vehicleStart) {
    //         std::string targetVehicleId = dataNameStr.substr(vehicleStart, vehicleEnd - vehicleStart);

    //         NFD_LOG_INFO("[" << m_nodeName << "] *** DIRECT DELIVERY *** to " << targetVehicleId);

    //         for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i) {
    //             Ptr<Node> node = NodeList::GetNode(i);
    //             if (node && node->GetNApplications() > 0) {
    //                 Ptr<VanetVehicleApp> vehicleApp = DynamicCast<VanetVehicleApp>(node->GetApplication(0));
    //                 if (vehicleApp) {
    //                     std::string nodeVehicleId = Names::FindName(node);
    //                     if (nodeVehicleId == targetVehicleId) {
    //                         NFD_LOG_INFO("[" << m_nodeName << "] *** FOUND TARGET VEHICLE *** " << targetVehicleId);

    //                         vehicleApp->OnData(data);
    //                         break;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

}

void VanetBlockchainApp::SendInterest(std::shared_ptr<const ndn::Interest> interest) {
    NFD_LOG_INFO("[" << m_nodeName << "] Sending Interest: " << interest->getName());
    
    // ENHANCED: Record communication overhead
    if (m_metricsCollector) {
        std::string interestNameStr = interest->getName().toUri();
        uint32_t interestSize = 500; // Base NDN overhead
        
        if (interest->getApplicationParameters().value_size() > 0) {
            interestSize += interest->getApplicationParameters().value_size();
        }
        
        std::string packetType = "Interest_Unknown";
        std::string destination = "BROADCAST";
        bool isControl = true;
        
        // Classify interest packet type
        if (interestNameStr.find("/vanet/pbft/") != std::string::npos) {
            if (interestNameStr.find("adaptive-transaction-batch") != std::string::npos) {
                packetType = "Interest_AdaptiveBatch";
                isControl = false; // This packet's purpose is to carry application data
            } else if (interestNameStr.find("pre-prepare") != std::string::npos) {
                packetType = "Interest_PBFT_PrePrepare";
                isControl = true;
            } else if (interestNameStr.find("prepare") != std::string::npos) {
                packetType = "Interest_PBFT_Prepare";
                isControl = true;
            } else if (interestNameStr.find("commit") != std::string::npos) {
                packetType = "Interest_PBFT_Commit";
                isControl = true;
            } else if (interestNameStr.find("transaction-batch") != std::string::npos) {
                packetType = "Interest_PBFT_TransactionBatch";
                isControl = false;
            } else if (interestNameStr.find("transaction") != std::string::npos) {
                packetType = "Interest_PBFT_Transaction";
                isControl = true;
            }
            destination = "ALL_RSUS";
        }
        else if (interestNameStr.find("/register/") != std::string::npos) {
            packetType = "Interest_Registration";
            isControl = false;
            size_t vanetPos = interestNameStr.find("/vanet/");
            if (vanetPos != std::string::npos) {
                size_t rsuStart = vanetPos + 6; // Length of "/vanet/"
                size_t rsuEnd = interestNameStr.find("/", rsuStart);
                if (rsuEnd != std::string::npos && rsuStart < interestNameStr.length()) {
                    destination = interestNameStr.substr(rsuStart, rsuEnd - rsuStart);
                }
            }
        }
        else if (interestNameStr.find("/eventreport/") != std::string::npos) {
            packetType = "Interest_EventReport";
            isControl = false; // Event reports are application data
            size_t vanetPos = interestNameStr.find("/vanet/");
            if (vanetPos != std::string::npos) {
                size_t rsuStart = vanetPos + 6;
                size_t rsuEnd = interestNameStr.find("/", rsuStart);
                if (rsuEnd != std::string::npos && rsuStart < interestNameStr.length()) {
                    destination = interestNameStr.substr(rsuStart, rsuEnd - rsuStart);
                }
            }
        }
        else if (interestNameStr.find("/blockchain/") != std::string::npos) {
            if (interestNameStr.find("/keys/") != std::string::npos) {
                packetType = "Interest_BlockchainKeyQuery";
            } else if (interestNameStr.find("/reputation/") != std::string::npos) {
                packetType = "Interest_ReputationQuery";
            } else if (interestNameStr.find("/blocks/") != std::string::npos) {
                packetType = "Interest_BlockQuery";
            }
            isControl = false;
            // Extract RSU destination
            size_t vanetPos = interestNameStr.find("/vanet/");
            if (vanetPos != std::string::npos) {
                size_t rsuStart = vanetPos + 6;
                size_t rsuEnd = interestNameStr.find("/", rsuStart);
                if (rsuEnd != std::string::npos && rsuStart < interestNameStr.length()) {
                    destination = interestNameStr.substr(rsuStart, rsuEnd - rsuStart);
                }
            }
        }
        else if (interestNameStr.find("/forward-transaction") != std::string::npos) {
            packetType = "Interest_TransactionForward";
            isControl = true;
            size_t vanetPos = interestNameStr.find("/vanet/");
            if (vanetPos != std::string::npos) {
                size_t rsuStart = vanetPos + 6;
                size_t rsuEnd = interestNameStr.find("/", rsuStart);
                if (rsuEnd != std::string::npos && rsuStart < interestNameStr.length()) {
                    destination = interestNameStr.substr(rsuStart, rsuEnd - rsuStart);
                }
            }
        }
        
        m_metricsCollector->RecordCommunicationOverhead(packetType, interestSize, 
                                                       m_nodeName, destination, isControl);
    }
    
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
}

void VanetBlockchainApp::OnInterest(std::shared_ptr<const ndn::Interest> interest) {
    NFD_LOG_INFO("[" << m_nodeName << "] *** RECEIVED INTEREST *** " << interest->getName()
                << " at time " << Simulator::Now().GetSeconds() << "s");

    if (m_nodeType != RSU_VALIDATOR) return;

    const ndn::Name& name = interest->getName();

    if (name.size() < 2 || name.get(0).toUri() != "vanet") {
        NFD_LOG_WARN("[" << m_nodeName << "] Interest rejected - not VANET: " << name);
        return;
    }

    bool isPbftBroadcast = (name.size() >= 3 && name.get(1).toUri() == "pbft");
    bool forThisRsuDirectly = (name.size() >= 2 && name.get(1).toUri() == m_nodeName);

    if (isPbftBroadcast) {
        std::string phase = name.get(2).toUri();
        if (phase == "pre-prepare") HandlePrePrepare(interest);
        else if (phase == "prepare") HandlePrepare(interest);
        else if (phase == "commit") HandleCommit(interest);
        else if (phase == "transaction") HandleTransactionBroadcast(interest);
        else if (phase == "transaction-with-ack") HandleTransactionWithAckBroadcast(interest);
        else if (phase == "transaction-batch") HandleTransactionBatch(interest);
        else if (phase == "adaptive-transaction-batch") HandleAdaptiveBatch(interest); 
    }
    else if (forThisRsuDirectly) {
        if (name.size() < 3) {
            NFD_LOG_WARN("[" << m_nodeName << "] Direct interest too short: " << name);
            return;
        }

        std::string service = name.get(2).toUri();

        if (service == "register" && name.size() >= 5) {
            HandleRegistrationInterest(interest);
        }
        else if (service == "eventreport" && name.size() >= 4) {
            HandleEventReportInterest(interest);
        }
        else if (service == "forward-transaction" && name.size() >= 4) {
            HandleForwardedTransaction(interest);
        }
        else if (service == "blockchain" && name.size() >= 5) {
            std::string type = name.get(3).toUri();

            if (type == "keys" && name.size() >= 5) {
                std::string vehicleId = name.get(4).toUri();
                NFD_LOG_INFO("[" << m_nodeName << "] *** BLOCKCHAIN KEY QUERY *** for vehicle: '"
                            << vehicleId << "'");
                HandleKeyRequest(interest, vehicleId);
            }
            else if (type == "reputation" && name.size() >= 5) {
                std::string vehicleId = name.get(4).toUri();
                HandleReputationRequest(interest, vehicleId);
            }
            else if (type == "blocks" && name.size() >= 5) {
                std::string blockId = name.get(4).toUri();
                HandleBlockRequest(interest, blockId);
            }
        } else if (service == "location-query" && name.size() >= 5) {
            HandleLocationQueryInterest(interest);
        }
    }
}

void VanetBlockchainApp::HandleAdaptiveBatch(std::shared_ptr<const ndn::Interest> interest) {
    if (m_nodeName == "RSU-0") {
        NS_LOG_DEBUG("[" << m_nodeName << "] Leader ignoring own adaptive batch broadcast");
        return;
    }
    
    const ndn::Name& name = interest->getName();
    if (name.size() < 6) { // adaptive-transaction-batch/RSU-0/size/rate/latency/timestamp
        NS_LOG_WARN("[" << m_nodeName << "] Invalid adaptive batch Interest format");
        return;
    }
    
    std::string sender = name.get(3).toUri();
    if (sender != "RSU-0") {
        NS_LOG_WARN("[" << m_nodeName << "] Ignoring adaptive batch from non-leader: " << sender);
        return;
    }
    

    uint32_t batchSize = name.get(4).toNumber();
    double senderRate = static_cast<double>(name.get(5).toNumber()) / 100.0; // Rate info
    double senderLatency = static_cast<double>(name.get(6).toNumber()) / 1000.0; // Latency info
    
    if (interest->getApplicationParameters().value_size() == 0) {
        NS_LOG_WARN("[" << m_nodeName << "] Empty adaptive batch from " << sender);
        return;
    }
    
    const auto& appParams = interest->getApplicationParameters();
    std::string batchData(reinterpret_cast<const char*>(appParams.value()), 
                         appParams.value_size());
    
    std::vector<Transaction> parsedTransactions = ParseAdaptiveBatch(batchData);
    
    NS_LOG_INFO("[" << m_nodeName << "] Received ADAPTIVE batch from " << sender 
               << ": " << parsedTransactions.size() << "/" << batchSize << " transactions"
               << " [Rate: " << senderRate << " tx/s, Latency: " << senderLatency << "s]");
    

    m_adaptiveBatchManager.UpdateNetworkParameters(m_totalVehicles, senderLatency);
    
    // Add transactions to pool with duplicate checking
    size_t successCount = 0;
    for (const auto& tx : parsedTransactions) {
        bool isDuplicate = false;
        std::string newTxSerialized = tx.serialize();
        
        for (const auto& poolTx : m_transactionPool) {
            if (poolTx.serialize() == newTxSerialized) {
                isDuplicate = true;
                break;
            }
        }
        
        if (!isDuplicate) {
            AddTransactionToPool(tx);
            successCount++;
        }
    }
    
    NS_LOG_INFO("[" << m_nodeName << "] Added " << successCount << "/" << parsedTransactions.size() 
               << " new adaptive transactions. Pool size: " << m_transactionPool.size());
}

std::vector<Transaction> VanetBlockchainApp::ParseAdaptiveBatch(const std::string& batchData) {
    std::vector<Transaction> transactions;
    
    std::vector<std::string> parts;
    std::stringstream ss(batchData);
    std::string part;
    while (std::getline(ss, part, '|')) {
        parts.push_back(part);
    }
    
    if (parts.empty() || parts[0].find("ADAPTIVE_BATCH:") != 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid adaptive batch format");
        return transactions;
    }
    
    // Extract batch size
    size_t batchSize = 0;
    try {
        batchSize = std::stoul(parts[0].substr(15)); // Remove "ADAPTIVE_BATCH:"
    } catch (const std::exception& e) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid adaptive batch size: " << e.what());
        return transactions;
    }
    
    // Extract network parameters (optional - for logging/debugging)
    double networkRate = 0.0, networkLatency = 0.0, congestionFactor = 1.0;
    size_t txStartIndex = 1;
    
    for (size_t i = 1; i < parts.size() && i < 4; ++i) {
        if (parts[i].find("RATE:") == 0) {
            networkRate = std::stod(parts[i].substr(5));
            txStartIndex = i + 1;
        } else if (parts[i].find("LATENCY:") == 0) {
            networkLatency = std::stod(parts[i].substr(8));
            txStartIndex = i + 1;
        } else if (parts[i].find("CONGESTION:") == 0) {
            congestionFactor = std::stod(parts[i].substr(11));
            txStartIndex = i + 1;
        } else {
            break;
        }
    }
    
    NFD_LOG_DEBUG("[" << m_nodeName << "] Adaptive batch metadata: Rate=" << networkRate 
                 << " tx/s, Latency=" << networkLatency << "s, Congestion=" << congestionFactor);
    
    // Parse transactions starting from txStartIndex
    for (size_t i = txStartIndex; i < parts.size() && (i - txStartIndex) < batchSize; ++i) {
        std::string txPart = parts[i];
        
        // Remove TX prefix: "TX0:REG:..." -> "REG:..."
        size_t colonPos = txPart.find(':');
        if (colonPos == std::string::npos || txPart.length() <= colonPos + 1) {
            NFD_LOG_WARN("[" << m_nodeName << "] Malformed adaptive TX part: " << txPart);
            continue;
        }
        
        std::string txData = txPart.substr(colonPos + 1);
        Transaction tx;
        bool parsed = false;
        
        // Parse transaction data (same logic as original ParseTransactionBatch)
        if (txData.find("REG:") == 0) {
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 4) {
                try {
                    tx.type = REGISTRATION;
                    tx.vehicleId_reg = txParts[1];
                    tx.publicKey = txParts[2];
                    tx.initialReputation = std::stod(txParts[3]);
                    tx.timestamp = Simulator::Now().GetTimeStep();
                    parsed = true;
                } catch (const std::exception& e) {
                    NFD_LOG_WARN("[" << m_nodeName << "] Error parsing adaptive REG transaction: " << e.what());
                }
            }
        }
        else if (txData.find("EVT:") == 0) {
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 3) {
                tx.type = EVENT_DECISION;
                tx.eventId_dec = txParts[1];
                tx.eventVerdict = txParts[2];
                tx.timestamp = Simulator::Now().GetTimeStep();
                tx.eventType = "Unknown";
                tx.eventLocation = "Unknown";
                tx.eventTimestamp = 0;
                tx.eventCredibility = 0.0;
                parsed = true;
            }
        }
        else if (txData.find("REP:") == 0) {
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 4) {
                try {
                    tx.type = REPUTATION_UPDATE;
                    tx.vehicleId_rep = txParts[1];
                    tx.oldReputation = std::stod(txParts[2]);
                    tx.newReputation = std::stod(txParts[3]);
                    tx.timestamp = Simulator::Now().GetTimeStep();
                    tx.eventId_rep = "Unknown";
                    parsed = true;
                } catch (const std::exception& e) {
                    NFD_LOG_WARN("[" << m_nodeName << "] Error parsing adaptive REP transaction: " << e.what());
                }
            }
        }
        
        if (parsed) {
            transactions.push_back(tx);
        } else {
            NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse adaptive transaction: " << txData);
        }
    }
    
    if (transactions.size() != batchSize) {
        NFD_LOG_WARN("[" << m_nodeName << "] Adaptive batch size mismatch: expected " << batchSize 
                     << ", parsed " << transactions.size());
    }
    
    return transactions;
}


void VanetBlockchainApp::HandleTransactionBroadcast(std::shared_ptr<const ndn::Interest> interest) {
    // Only non-leader RSUs should process broadcasted transactions
    if (m_nodeName == "RSU-0") {
        NFD_LOG_DEBUG("[" << m_nodeName << "] Leader ignoring own broadcast");
        return;
    }

    if (interest->getApplicationParameters().value_size() == 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Transaction broadcast has no payload");
        return;
    }

    const auto& appParams = interest->getApplicationParameters();
    std::string txData(reinterpret_cast<const char*>(appParams.value()), appParams.value_size());

    NFD_LOG_INFO("[" << m_nodeName << "] Received transaction broadcast from leader: " << txData);

    Transaction tx;
    bool parsed = false;

    if (txData.find("REG:") == 0) {
        // Parse: "REG:vehicleId:publicKey:initialRep"
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }

        if (parts.size() >= 4) {
            tx.type = REGISTRATION;
            tx.vehicleId_reg = parts[1];
            tx.publicKey = parts[2];
            tx.initialReputation = std::stod(parts[3]);
            tx.timestamp = Simulator::Now().GetTimeStep();
            parsed = true;
        }
    }
    else if (txData.find("EVT:") == 0) {
        // Parse: "EVT:eventId:verdict"
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }

        if (parts.size() >= 3) {
            tx.type = EVENT_DECISION;
            tx.eventId_dec = parts[1];
            tx.eventVerdict = parts[2];
            tx.timestamp = Simulator::Now().GetTimeStep();
            // Set default values for other fields
            tx.eventType = "Unknown";
            tx.eventLocation = "Unknown";
            tx.eventTimestamp = 0;
            tx.eventCredibility = 0.0;
            parsed = true;
        }
    }
    else if (txData.find("REP:") == 0) {
        // Parse: "REP:vehicleId:oldRep:newRep"
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }

        if (parts.size() >= 4) {
            tx.type = REPUTATION_UPDATE;
            tx.vehicleId_rep = parts[1];
            tx.oldReputation = std::stod(parts[2]);
            tx.newReputation = std::stod(parts[3]);
            tx.timestamp = Simulator::Now().GetTimeStep();
            // Set default values for other fields
            tx.eventId_rep = "Unknown";
            parsed = true;
        }
    }

    if (!parsed) {
        NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse transaction: " << txData);
        return;
    }

    // Check if we already have this transaction
    bool alreadyHave = false;
    for (const auto& poolTx : m_transactionPool) {
        if (poolTx.type == tx.type) {
            if (tx.type == REGISTRATION && poolTx.vehicleId_reg == tx.vehicleId_reg) {
                alreadyHave = true;
                break;
            } else if (tx.type == EVENT_DECISION && poolTx.eventId_dec == tx.eventId_dec) {
                alreadyHave = true;
                break;
            } else if (tx.type == REPUTATION_UPDATE && poolTx.vehicleId_rep == tx.vehicleId_rep) {
                alreadyHave = true;
                break;
            }
        }
    }

    if (!alreadyHave) {
        AddTransactionToPool(tx);
        NFD_LOG_INFO("[" << m_nodeName << "] Added broadcasted transaction. Pool size: " << m_transactionPool.size());
    } else {
        NFD_LOG_DEBUG("[" << m_nodeName << "] Transaction already in pool, ignoring");
    }
}

void VanetBlockchainApp::OnData(std::shared_ptr<const ndn::Data> data) {
    ndn::App::OnData(data);
    if (m_nodeType != RSU_VALIDATOR) return;

    const ndn::Name& dataName = data->getName();
    NFD_LOG_INFO("[" << m_nodeName << "] Received Data: " << dataName);

    // Check if this Data is for a pending block request (from HandlePrePrepare)
    std::string dataNameUri = dataName.toUri();
    if (m_pendingBlockRequests.count(dataNameUri)) {
        std::string requestedBlockHash = m_pendingBlockRequests[dataNameUri];
        m_pendingBlockRequests.erase(dataNameUri);

        if (m_pendingPbftBlocks.count(requestedBlockHash)) {
            VanetBlock receivedBlock = m_pendingPbftBlocks[requestedBlockHash];
            std::string proposerId = m_pbftActiveConsensus.count(requestedBlockHash) ?
                                     m_pbftActiveConsensus[requestedBlockHash].proposerId :
                                     "Unknown"; // Should get proposer from pre-prepare
            ProcessReceivedBlockForPbft(receivedBlock, proposerId);
        } else {
             NFD_LOG_WARN("[" << m_nodeName << "] Received Data for block " << requestedBlockHash.substr(0,8) << " but block content not found/deserialized.");
        }
    }
}


void VanetBlockchainApp::HandleRegistrationInterest(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();
    std::string vehicleId = name.get(3).toUri();
    std::string publicKey = name.get(4).toUri();
    
    Time requestTime = Simulator::Now(); // Record when we received the request
    
    m_totalRegistrationsProcessed++;

    NFD_LOG_INFO("[" << m_nodeName << "] Registration request #" << m_totalRegistrationsProcessed
                 << " from " << vehicleId);

    if (m_metricsCollector) {
        uint32_t interestSize = 500; // Estimate
        m_metricsCollector->RecordCommunicationOverhead("Interest_Registration", interestSize, 
                                                       vehicleId, m_nodeName, false);
    }

    // Check if vehicle is already registered
    if (IsVehicleRegistered(vehicleId)) {
        NFD_LOG_INFO("[" << m_nodeName << "] Vehicle " << vehicleId 
                     << " already registered, but will send final ACK after next consensus round");
    }

    // Store pending registration info with request time
    PendingRegistration pending;
    pending.vehicleId = vehicleId;
    pending.publicKey = publicKey;
    pending.originalInterestName = name;
    pending.requestTime = requestTime; // Store the request time
    pending.requestingRsu = m_nodeName;

    m_pendingRegistrations[vehicleId] = pending;

    // Create registration transaction
    Transaction tx;
    tx.type = REGISTRATION;
    tx.timestamp = Simulator::Now().GetTimeStep();
    tx.vehicleId_reg = vehicleId;
    tx.publicKey = publicKey;
    tx.initialReputation = 0.5;

    // Forward to leader (or process if we are leader)
    ForwardTransactionToLeader(tx);

    NFD_LOG_INFO("[" << m_nodeName << "] Registration forwarded for " << vehicleId
                 << " - ACK will be sent AFTER consensus");
}

void VanetBlockchainApp::HandleEventReportInterest(std::shared_ptr<const ndn::Interest> interest) {
    if (interest->getApplicationParameters().value_size() == 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Event report Interest " << interest->getName() << " has no payload.");
        return;
    }

    const auto& appParams = interest->getApplicationParameters();
    std::string payload(reinterpret_cast<const char*>(appParams.value()), appParams.value_size());

    std::stringstream ss(payload);
    std::string segment;
    std::vector<std::string> parts;
    while(std::getline(ss, segment, '|')) {
       parts.push_back(segment);
    }

    if (parts.size() >= 7) {
        EventReport report;
        report.vehicleId = parts[0];
        report.reportedEventType = parts[1];

        // Parse location
        size_t underscore_pos = parts[2].find('_');
        if (underscore_pos != std::string::npos) {
            report.location.x = std::stod(parts[2].substr(0, underscore_pos));
            report.location.y = std::stod(parts[2].substr(underscore_pos + 1));
            report.location.z = 0.0;
        } else {
            NFD_LOG_WARN("Bad location format: " << parts[2]);
            return;
        }

        report.timestamp = Seconds(static_cast<double>(std::stoll(parts[3])));
        report.seqNum = std::stoul(parts[4]);
        report.signature = parts[5];
        report.originalEventType = parts[6];
        report.receivedTime = Simulator::Now();

        // Increment counter for tracking
        m_totalEventReportsProcessed++;

        NFD_LOG_INFO("[" << m_nodeName << "] Processing event report #" << m_totalEventReportsProcessed
                     << " from " << report.vehicleId
                     << " for " << report.reportedEventType << " at location " << parts[2]);

        // Log every 50th report for progress tracking in long simulation
        if (m_totalEventReportsProcessed % 50 == 0) {
            NFD_LOG_INFO("[" << m_nodeName << "] *** MILESTONE: Processed "
                         << m_totalEventReportsProcessed << " event reports ***");
        }

        // Process the event report (clustering and credibility calculation)
        ProcessEventReport(report);

    } else {
        NFD_LOG_WARN("[" << m_nodeName << "] Malformed event payload: " << payload);
    }
}



void VanetBlockchainApp::HandleKeyRequest(std::shared_ptr<const ndn::Interest> interest,
                                         const std::string& vehicleId_param) {
    Time requestTime = Simulator::Now(); // Record request time
    
    NFD_LOG_INFO("[" << m_nodeName << "] *** KEY REQUEST *** for '"
                 << vehicleId_param << "' at " << Simulator::Now().GetSeconds() << "s");

    std::string response_content;
    bool found = m_vehicleKeys.count(vehicleId_param) > 0;

    if (found) {
        response_content = m_vehicleKeys.at(vehicleId_param);
        NFD_LOG_INFO("[" << m_nodeName << "] *** FOUND *** " << vehicleId_param);
    } else {
        response_content = "NOT_FOUND";
        NFD_LOG_INFO("[" << m_nodeName << "] *** NOT FOUND *** " << vehicleId_param);
    }

    // Use exact Interest name for Data response
    auto data = std::make_shared<ndn::Data>(interest->getName());
    data->setFreshnessPeriod(ndn::time::milliseconds(1));
    data->setContent(std::make_shared<::ndn::Buffer>(response_content.begin(),
                                                     response_content.end()));

    NFD_LOG_INFO("[" << m_nodeName << "] Sending response to " << vehicleId_param);
    SendData(data);
    if (m_metricsCollector) {
        
        // Record communication overhead
        uint32_t packetSize = 500; // Estimate with NDN headers
        m_metricsCollector->RecordCommunicationOverhead("Data_BlockchainQuery", packetSize, 
                                                       m_nodeName, vehicleId_param, false);
    }
    
}

void VanetBlockchainApp::HandleReputationRequest(std::shared_ptr<const ndn::Interest> interest, const std::string& vehicleId) {
    double reputation = m_vehicleReputations.count(vehicleId) ? m_vehicleReputations[vehicleId] : -1.0;
    std::string repStr = std::to_string(reputation);
    auto data = std::make_shared<ndn::Data>(interest->getName());
    data->setFreshnessPeriod(ndn::time::minutes(1));
    data->setContent(std::make_shared<::ndn::Buffer>(repStr.begin(), repStr.end()));
    SendData(data);
}

void VanetBlockchainApp::HandleBlockRequest(std::shared_ptr<const ndn::Interest> interest, const std::string& blockHash) {
    if (m_pendingPbftBlocks.count(blockHash)) {
        // TODO: Serialize block to content
        std::string blockContentPlaceholder = "BlockData_for_" + blockHash;
        auto data = std::make_shared<ndn::Data>(interest->getName());
        data->setFreshnessPeriod(ndn::time::minutes(1));
        data->setContent(std::make_shared<::ndn::Buffer>(blockContentPlaceholder.begin(), blockContentPlaceholder.end()));
        SendData(data);
    } else {
        NFD_LOG_WARN("[" << m_nodeName << "] BlockRequest for " << blockHash.substr(0,8) << " but block not found.");
    }
}

void VanetBlockchainApp::InitiateEventDecision(const std::string& eventId, const std::string& eventType,
                                           const std::string& location, uint64_t eventTime,
                                           const std::vector<std::pair<std::string, std::string>>& reports,
                                           const std::string& verdict, double credibility) {
    Transaction tx;
    tx.type = EVENT_DECISION;
    tx.timestamp = Simulator::Now().GetTimeStep();
    tx.eventId_dec = eventId;
    tx.eventType = eventType;
    tx.eventLocation = location;
    tx.eventTimestamp = eventTime;
    tx.eventReports = reports;
    tx.eventVerdict = verdict;
    tx.eventCredibility = credibility;

    // Forward to leader (or process if we are leader, will go to batch)
    ForwardTransactionToLeader(tx);
}

void VanetBlockchainApp::InitiateReputationUpdate(const std::string& vehicleId, const std::string& eventId,
                                              double oldRep, double newRep) {
    Transaction tx;
    tx.type = REPUTATION_UPDATE;
    tx.timestamp = Simulator::Now().GetTimeStep();
    tx.vehicleId_rep = vehicleId;
    tx.eventId_rep = eventId;
    tx.oldReputation = oldRep;
    tx.newReputation = newRep;

    ForwardTransactionToLeader(tx);
}
void VanetBlockchainApp::AddTransactionToPool(const Transaction& tx) {
    m_transactionPool.push_back(tx);

    
    NFD_LOG_INFO("[" << m_nodeName << "] Transaction added to pool. Size: " << m_transactionPool.size());
}

VanetBlock VanetBlockchainApp::CreateCandidateBlock() {
    VanetBlock b; b.height = m_localBlockchain.back().height + 1;
    b.timestamp = Simulator::Now().GetTimeStep();
    b.previousHash = m_localBlockchain.back().blockHash;
    b.proposerId = m_nodeName; b.transactions = m_transactionPool; // Uses the current transaction pool
    b.blockHash = b.calculateHash(); 
    m_blockCreationTimes[b.blockHash] = Simulator::Now();
    return b;

}

void VanetBlockchainApp::TryProposeNewBlock() {
    NFD_LOG_INFO("[" << m_nodeName << "] TryProposeNewBlock called. Pool size: " 
                 << m_transactionPool.size());
    
    
    if (m_transactionPool.empty()) {
        NFD_LOG_DEBUG("[" << m_nodeName << "] No transactions to propose");
        return;
    }
    
    if (m_nodeName != "RSU-0") {
        NFD_LOG_DEBUG("[" << m_nodeName << "] Not RSU-0, cannot propose blocks");
        return;
    }
    
    uint32_t nextHeight = m_localBlockchain.back().height + 1;
    
    // Check if already processing this height
    for (const auto& pair : m_pbftActiveConsensus) {
        if (pair.second.block.height == nextHeight) {
            NFD_LOG_WARN("[" << m_nodeName << "] Already processing height " << nextHeight);
            return;
        }
    }
    

    Time currentTime = Simulator::Now();
    Time cooldownPeriod = Seconds(0);
    Time nextAllowedTime = m_lastBlockProposalTime + cooldownPeriod;
    
    if (currentTime < nextAllowedTime) {
        Time delayNeeded = nextAllowedTime - currentTime + MilliSeconds(0);
        NFD_LOG_INFO("[" << m_nodeName << "] Cooldown active. Rescheduling in " 
                     << delayNeeded.GetSeconds() << "s");
        Simulator::Schedule(delayNeeded, &VanetBlockchainApp::TryProposeNewBlock, this);
        return;
    }
    
    NFD_LOG_INFO("[" << m_nodeName << "] *** REGULAR BLOCK PROPOSAL *** height " << nextHeight
                 << " with " << m_transactionPool.size() << " transactions");
    
    m_lastBlockProposalTime = currentTime;
    VanetBlock newBlock = CreateCandidateBlock();
    StartPbft(newBlock);
}


double VanetBlockchainApp::CalculateDistance(const Vector& pos1, const Vector& pos2) {
    return ns3::CalculateDistance(pos1, pos2);
}

std::string VanetBlockchainApp::FindMatchingCluster(const EventReport& report) {
    for (auto& pair : m_activeClusters) {
        EventCluster& cluster = pair.second;
        if (cluster.decisionMade) continue;

        double dist = CalculateDistance(report.location, cluster.centerLocation);
        Time timeDiff = (report.timestamp > cluster.centerTime) ?
                       (report.timestamp - cluster.centerTime) :
                       (cluster.centerTime - report.timestamp);

        // Enhanced matching for multiple event types
        bool typeMatch = (report.originalEventType == cluster.eventType);

        // Allow clustering of similar event types
        if (!typeMatch) {
            if ((report.originalEventType == "Accident" && cluster.eventType == "Breakdown") ||
                (report.originalEventType == "Breakdown" && cluster.eventType == "Accident") ||
                (report.originalEventType == "Jam" && cluster.eventType == "Construction") ||
                (report.originalEventType == "Construction" && cluster.eventType == "Jam") ||
                (report.originalEventType == "Roadwork" && cluster.eventType == "Construction") ||
                (report.originalEventType == "Construction" && cluster.eventType == "Roadwork")) {
                typeMatch = true;
            }
        }

        if (typeMatch && dist < m_distanceThreshold && timeDiff < m_timeThreshold) {
            return pair.first;
        }
    }
    return "";
}
void VanetBlockchainApp::ProcessEventReport(const EventReport& report) {

    if (m_nodeType != RSU_VALIDATOR) {
        return;
    }

    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    Time processingDelay = MilliSeconds(rand->GetValue(0, 0));

    NFD_LOG_INFO("[" << m_nodeName << "] Received event report from " << report.vehicleId 
                 << ". Simulating signature verification delay of " << processingDelay.GetMilliSeconds() << " ms.");

    Simulator::Schedule(processingDelay, [this, report]() {

        //extra delay
        if (!VerifyVehicleSignature(report)) {
            NFD_LOG_WARN("[" << m_nodeName << "] Invalid signature for " << report.vehicleId << ". Dropping report.");
            return;
        }

        std::string clusterId = FindMatchingCluster(report);
        if (clusterId.empty()) {
            EventCluster newCluster;
            newCluster.eventId = "Evt_" + report.vehicleId + "_" + std::to_string(report.timestamp.GetTimeStep());
            newCluster.eventType = report.originalEventType;
            newCluster.centerLocation = report.location;
            newCluster.centerTime = report.timestamp;
            newCluster.reports.push_back(report);
            newCluster.creationTime = Simulator::Now();
            m_activeClusters[newCluster.eventId] = newCluster;
            clusterId = newCluster.eventId;
            
            
            ScheduleClusterCheck(clusterId);
        } else {
            m_activeClusters[clusterId].reports.push_back(report);
            
            const EventCluster& cluster = m_activeClusters[clusterId];
        }
    });

}
void VanetBlockchainApp::ScheduleClusterCheck(const std::string& eventId) {
    if (!m_activeClusters.count(eventId) || m_activeClusters[eventId].decisionMade) return;
    EventCluster& cluster = m_activeClusters[eventId];
    cluster.decisionEvent = Simulator::Schedule(m_clusterCheckDelay, &VanetBlockchainApp::CheckCluster, this, eventId);
    NFD_LOG_INFO("Scheduled check for " << eventId);
}

void VanetBlockchainApp::CheckCluster(const std::string& eventId) {
    if (!m_activeClusters.count(eventId) || m_activeClusters[eventId].decisionMade) return;
    EventCluster& cluster = m_activeClusters[eventId];
    cluster.decisionMade = true;
    NFD_LOG_INFO("Checking Cluster " << eventId);
    CalculateEventCredibility(cluster);
}

double VanetBlockchainApp::CalculateSuspicionScore(const std::string& vehicleId) {
    // If no history, assume neutral behavior
    if (m_vehicleReportHistory.find(vehicleId) == m_vehicleReportHistory.end() || 
        m_vehicleReportHistory[vehicleId].empty()) {
        return 0.0; // 0 suspicion initially
    }

    const std::deque<int>& history = m_vehicleReportHistory[vehicleId];
    int N = history.size();

    double numerator_rpf = 0.0;
    double denominator_rpf = 0.0;

    for (int i = 0; i < N; ++i) {

        double weight = std::pow(DECAY_GAMMA, N - 1 - i);
        numerator_rpf += history[i] * weight;
        denominator_rpf += weight;
    }

    double RPF = (denominator_rpf > 0) ? (numerator_rpf / denominator_rpf) : 0.0;

    // --- 2. Volatility Factor (F_vol) - Formula (4) ---
    double F_vol = 0.0;
    if (N > 1) {
        double volatility_sum = 0.0;
        for (int i = 1; i < N; ++i) {
            volatility_sum += std::abs(history[i] - history[i-1]);
        }
        F_vol = volatility_sum / (N - 1);
    }

    // --- 3. Suspicion Score (R_suspicion) - Formula (6) ---
    double R_suspicion = (WEIGHT_RPF * (1.0 - RPF)) + (WEIGHT_VOL * F_vol);

    return R_suspicion;
}


void VanetBlockchainApp::UpdateLocalHistory(const std::string& vehicleId, bool isCorrect) {
    int outcome = isCorrect ? 1 : 0;
    m_vehicleReportHistory[vehicleId].push_back(outcome);

    // Maintain sliding window size N
    if (m_vehicleReportHistory[vehicleId].size() > HISTORY_WINDOW_N) {
        m_vehicleReportHistory[vehicleId].pop_front();
    }
}

void VanetBlockchainApp::CalculateEventCredibility(EventCluster& cluster) {
    if (cluster.reports.empty()) return;

    // --- Step 1: Calculate Raw Trust Factors (T_factor) ---
    std::map<std::string, double> rawTrustFactors;
    double T_min = 1.0;
    double T_max = 0.0;

    for (const auto& r : cluster.reports) {
        double baseRep = m_vehicleReputations.count(r.vehicleId) ? m_vehicleReputations[r.vehicleId] : 0.5;
        
        // Calculate dynamic suspicion based on history (RPF & Volatility)
        double R_suspicion = CalculateSuspicionScore(r.vehicleId);

        // Formula (8): T_factor = BaseRep * (1 - R_suspicion)
        double T_factor = baseRep * (1.0 - R_suspicion);
        
        rawTrustFactors[r.vehicleId] = T_factor;

        // Track Min/Max for normalization
        if (T_factor < T_min) T_min = T_factor;
        if (T_factor > T_max) T_max = T_factor;
    }

    // --- Step 2: Normalization (T_norm) Formula (7) ---
    std::map<std::string, double> normTrustScores;
    double sumT_all = 0.0;

    for (const auto& r : cluster.reports) {
        double T_norm = 0.0;
        if (std::abs(T_max - T_min) < 1e-6) {
            T_norm = 0.5; // Avoid division by zero if all scores are identical
        } else {
            T_norm = (rawTrustFactors[r.vehicleId] - T_min) / (T_max - T_min);
        }
        
        normTrustScores[r.vehicleId] = T_norm;
        sumT_all += T_norm;
    }

    // --- Step 3: Credibility-Based Event Decision (Algorithm 2) ---
    
    // Group normalized trust by Claim Type
    std::map<std::string, double> claimConfidenceMap;
    std::vector<std::string> uniqueClaims;

    // Identify unique claims
    for (const auto& r : cluster.reports) {
        if (claimConfidenceMap.find(r.reportedEventType) == claimConfidenceMap.end()) {
            uniqueClaims.push_back(r.reportedEventType);
            claimConfidenceMap[r.reportedEventType] = 0.0;
        }
    }

    // Calculate Confidence Score (C_claim)
    for (const auto& claim : uniqueClaims) {
        double sumT_supporters = 0.0;
        for (const auto& r : cluster.reports) {
            if (r.reportedEventType == claim) {
                sumT_supporters += normTrustScores[r.vehicleId];
            }
        }
        
        // Formula (Algorithm 2): C_claim = Sum(Supporters) / Sum(All)
        double C_claim = (sumT_all > 0) ? (sumT_supporters / sumT_all) : 0.0;
        claimConfidenceMap[claim] = C_claim;
    }

    // Find C_max and Winning Claim
    std::string winningClaim = "Unknown";
    double C_max = -1.0;

    for (const auto& pair : claimConfidenceMap) {
        if (pair.second > C_max) {
            C_max = pair.second;
            winningClaim = pair.first;
        }
    }

    // --- Step 4: Final Verdict based on Threshold ---
    std::string verdict = "Uncertain";
    
    // Check against Confidence Threshold (tau = 0.70)
    if (C_max > CONFIDENCE_THRESHOLD_TAU) {
        verdict = "VALIDATED";
    } else {
        verdict = "UNCERTAIN";
    }

    NFD_LOG_INFO("[FMD] Event " << cluster.eventId << " | Winner: " << winningClaim 
                 << " | Conf: " << C_max << " | Verdict: " << verdict);

    // Prepare transaction data
    std::vector<std::pair<std::string, std::string>> reportListTx;
    for (const auto& r : cluster.reports) {
        reportListTx.push_back({r.vehicleId, r.reportedEventType});
    }


    InitiateEventDecision(cluster.eventId, winningClaim, 
                          std::to_string(cluster.centerLocation.x) + "_" + std::to_string(cluster.centerLocation.y),
                          cluster.centerTime.GetTimeStep(), reportListTx, verdict, C_max);

    // Update Reputations (Only if Validated)
    UpdateReputations(cluster, verdict, winningClaim);
}
void VanetBlockchainApp::UpdateReputations(const EventCluster& cluster, 
                                           const std::string& verdict, 
                                           const std::string& groundTruth) {

    if (verdict != "VALIDATED") {
        NFD_LOG_INFO("[FMD] Verdict UNCERTAIN. Skipping reputation updates.");
        return;
    }

    NFD_LOG_INFO("[FMD] Updating Reputations. Ground Truth: " << groundTruth);

    for (const auto& report : cluster.reports) {
        double oldRep = m_vehicleReputations.count(report.vehicleId) ? m_vehicleReputations[report.vehicleId] : 0.5;
        
        bool isCorrect = (report.reportedEventType == groundTruth);
        double delta_R = 0.0;


        if (isCorrect) {

            delta_R = m_alpha * (1.0 - oldRep);
        } else {
            // Penalty: -beta * OldRep
            delta_R = -m_beta * oldRep;
        }

        double newRep = oldRep + delta_R;

        newRep = std::max(0.0, std::min(1.0, newRep));

        // Update Memory
        m_vehicleReputations[report.vehicleId] = newRep;


        UpdateLocalHistory(report.vehicleId, isCorrect);

        NFD_LOG_INFO("[FMD] Vehicle " << report.vehicleId << " | Correct: " << isCorrect 
                     << " | Delta: " << delta_R << " | NewRep: " << newRep);

        InitiateReputationUpdate(report.vehicleId, cluster.eventId, oldRep, newRep);

        if (m_metricsCollector) {
            m_metricsCollector->RecordReputationUpdate(
                report.vehicleId, newRep, IsVehicleActuallyAttacker(report.vehicleId), 
                (isCorrect ? "REWARD" : "PENALTY"), cluster.eventId
            );
        }
    }
}

std::string VanetBlockchainApp::ClassifyDetectionResult(bool wasActuallyMalicious, bool rsuAccepted) {
    if (wasActuallyMalicious && !rsuAccepted) {
        return "TP"; // True Positive: Malicious report correctly rejected
    } else if (wasActuallyMalicious && rsuAccepted) {
        return "FN"; // False Negative: Malicious report incorrectly accepted
    } else if (!wasActuallyMalicious && rsuAccepted) {
        return "TN"; // True Negative: Honest report correctly accepted
    } else if (!wasActuallyMalicious && !rsuAccepted) {
        return "FP"; // False Positive: Honest report incorrectly rejected
    }
    return "UNKNOWN";
}

void VanetBlockchainApp::StartPbft(const VanetBlock& newBlock) {
    if (m_pbftActiveConsensus.count(newBlock.blockHash)) return;


    NFD_LOG_INFO("[" << m_nodeName << "] Starting PBFT for block " << newBlock.blockHash.substr(0,8) << " H:" << newBlock.height);
    PbftBlockState newState;
    newState.block = newBlock;
    newState.phase = PRE_PREPARE_SENT;
    newState.view = m_pbftCurrentView;
    newState.seqNum = m_pbftCurrentSeqNum++;
    newState.proposerId = m_nodeName;
    newState.prepareVotes[m_nodeName] = SignString(newBlock.blockHash + "PREPARE");
    newState.commitVotes[m_nodeName] = SignString(newBlock.blockHash + "COMMIT");
    m_pbftActiveConsensus[newBlock.blockHash] = newState;
    m_pendingPbftBlocks[newBlock.blockHash] = newBlock;
    BroadcastPrePrepare(newBlock);
}



void VanetBlockchainApp::BroadcastPrePrepare(const VanetBlock& block) {


    ndn::Name interestName("/vanet/pbft/pre-prepare");
    interestName.append(block.blockHash);
    interestName.append(m_nodeName);
    interestName.appendNumber(m_pbftCurrentView);
    interestName.appendNumber(m_pbftCurrentSeqNum - 1);

    auto interest = std::make_shared<ndn::Interest>(interestName);
    interest->setNonce(GetNonce());
    interest->setInterestLifetime(ndn::time::seconds(15));

    std::stringstream blockData;
    blockData << "BLOCK|" << block.height << "|" << block.previousHash << "|" << block.transactions.size();
    for (const auto& tx : block.transactions) {
        blockData << "|" << tx.serialize();
    }

   if (m_metricsCollector) {
        uint32_t messageSize = 500; // Estimate with NDN headers
        m_metricsCollector->RecordCommunicationOverhead("PBFT_PrePrepare", messageSize, 
                                                       m_nodeName, "ALL_RSUS", true);
    }

    std::string blockDataString = blockData.str();
    auto buffer = std::make_shared<::ndn::Buffer>(
        reinterpret_cast<const uint8_t*>(blockDataString.c_str()),
        blockDataString.size()
    );

    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);

    NFD_LOG_INFO("[" << m_nodeName << "] Broadcasting PRE-PREPARE for block "
                 << block.blockHash.substr(0,8) << " at height " << block.height
                 << " with " << block.transactions.size() << " transactions");

    SendInterest(interest);
}



std::string VanetBlockchainApp::DetermineProposer(uint32_t blockHeight) {
    return "RSU-0";
}


void VanetBlockchainApp::HandlePrePrepare(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();
    if (name.size() < 7) return;

    std::string blockHash = name.get(3).toUri();
    std::string proposerId = name.get(4).toUri();
    uint32_t view = name.get(5).toNumber();
    uint32_t seqNum = name.get(6).toNumber();

    NFD_LOG_INFO("[" << m_nodeName << "] Received PRE-PREPARE from " << proposerId
                 << " for block " << blockHash.substr(0,8));

    // Skip if this is from ourselves
    if (proposerId == m_nodeName) return;

    // SIMPLIFIED: Only accept proposals from RSU-0
    if (proposerId != "RSU-0") {
        NFD_LOG_WARN("[" << m_nodeName << "] Rejecting PRE-PREPARE from " << proposerId
                     << ". Only RSU-0 can propose blocks.");
        return;
    }

    if (m_pbftActiveConsensus.count(blockHash)) {
        NFD_LOG_DEBUG("[" << m_nodeName << "] Already have consensus state for " << blockHash.substr(0,8));
        return;
    }

    // Deserialize block from ApplicationParameters
    VanetBlock receivedBlock;
    if (interest->getApplicationParameters().value_size() > 0) {
        const auto& appParams = interest->getApplicationParameters();
        std::string blockData(reinterpret_cast<const char*>(appParams.value()), appParams.value_size());

        NFD_LOG_INFO("[" << m_nodeName << "] Block data: " << blockData);

        // Parse block data: "BLOCK|height|previousHash|txCount|tx1|tx2|..."
        std::vector<std::string> parts;
        std::stringstream ss(blockData);
        std::string part;
        while (std::getline(ss, part, '|')) {
            parts.push_back(part);
        }

        if (parts.size() >= 4 && parts[0] == "BLOCK") {
            receivedBlock.height = std::stoul(parts[1]);
            receivedBlock.previousHash = parts[2];
            uint32_t txCount = std::stoul(parts[3]);

            NFD_LOG_INFO("[" << m_nodeName << "] Parsing block with " << txCount << " transactions");

            // Parse transactions
            for (uint32_t i = 0; i < txCount && (4 + i) < parts.size(); i++) {
                Transaction tx;
                std::string txStr = parts[4 + i];

                NFD_LOG_DEBUG("[" << m_nodeName << "] Parsing transaction: " << txStr);

                bool txParsed = false;

                if (txStr.find("REG:") == 0) {
                    // Parse: "REG:vehicleId:publicKey:initialRep"
                    std::vector<std::string> txParts;
                    std::stringstream txSS(txStr);
                    std::string txPart;
                    while (std::getline(txSS, txPart, ':')) {
                        txParts.push_back(txPart);
                    }
                    if (txParts.size() >= 4) {
                        tx.type = REGISTRATION;
                        tx.vehicleId_reg = txParts[1];
                        tx.publicKey = txParts[2];
                        tx.initialReputation = std::stod(txParts[3]);
                        tx.timestamp = Simulator::Now().GetTimeStep();
                        txParsed = true;
                    }
                }
                else if (txStr.find("EVT:") == 0) {
                    // Parse: "EVT:eventId:verdict"
                    std::vector<std::string> txParts;
                    std::stringstream txSS(txStr);
                    std::string txPart;
                    while (std::getline(txSS, txPart, ':')) {
                        txParts.push_back(txPart);
                    }
                    if (txParts.size() >= 3) {
                        tx.type = EVENT_DECISION;
                        tx.eventId_dec = txParts[1];
                        tx.eventVerdict = txParts[2];
                        tx.timestamp = Simulator::Now().GetTimeStep();
                        // Set defaults for other fields
                        tx.eventType = "Unknown";
                        tx.eventLocation = "Unknown";
                        tx.eventTimestamp = 0;
                        tx.eventCredibility = 0.0;
                        txParsed = true;
                    }
                }
                else if (txStr.find("REP:") == 0) {
                    // Parse: "REP:vehicleId:oldRep:newRep"
                    std::vector<std::string> txParts;
                    std::stringstream txSS(txStr);
                    std::string txPart;
                    while (std::getline(txSS, txPart, ':')) {
                        txParts.push_back(txPart);
                    }
                    if (txParts.size() >= 4) {
                        tx.type = REPUTATION_UPDATE;
                        tx.vehicleId_rep = txParts[1];
                        tx.oldReputation = std::stod(txParts[2]);
                        tx.newReputation = std::stod(txParts[3]);
                        tx.timestamp = Simulator::Now().GetTimeStep();
                        tx.eventId_rep = "Unknown";
                        txParsed = true;
                    }
                }

                if (txParsed) {
                    receivedBlock.transactions.push_back(tx);
                    NFD_LOG_INFO("[" << m_nodeName << "] Successfully parsed transaction " << (i+1) << "/" << txCount);
                } else {
                    NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse transaction " << (i+1) << ": " << txStr);
                }
            }

            receivedBlock.blockHash = blockHash;
            receivedBlock.proposerId = proposerId;
            receivedBlock.timestamp = Simulator::Now().GetTimeStep(); // Or use a timestamp from the block if available

            NFD_LOG_INFO("[" << m_nodeName << "] Block parsed successfully. Transactions: "
                         << receivedBlock.transactions.size() << "/" << txCount);
        } else {
             NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse block data from PrePrepare: " << blockData);
            return;
        }
    } else {
        NFD_LOG_WARN("[" << m_nodeName << "] PrePrepare for " << blockHash.substr(0,8) << " has no block payload.");
        return; // No block data to process
    }


    // Validate block
    if (receivedBlock.previousHash != m_localBlockchain.back().blockHash) {
        NFD_LOG_WARN("[" << m_nodeName << "] Block has wrong previous hash. Expected: "
                     << m_localBlockchain.back().blockHash << ", Got: " << receivedBlock.previousHash);
        return;
    }

    // Accept and move to prepare phase
    PbftBlockState newState;
    newState.block = receivedBlock;
    newState.phase = PRE_PREPARE_RECEIVED;
    newState.view = view;
    newState.seqNum = seqNum;
    newState.proposerId = proposerId;

    m_pbftActiveConsensus[blockHash] = newState;
    m_pendingPbftBlocks[blockHash] = receivedBlock;

    NFD_LOG_INFO("[" << m_nodeName << "] Accepted PRE-PREPARE from RSU-0 for block " << blockHash.substr(0,8)
                 << " with " << receivedBlock.transactions.size() << " transactions. Moving to PREPARE.");

    BroadcastPrepare(blockHash);
}

void VanetBlockchainApp::BroadcastPrepare(const std::string& blockHash) {
    if (!m_pbftActiveConsensus.count(blockHash)) return;


    PbftBlockState& currentState = m_pbftActiveConsensus[blockHash];
    currentState.phase = PREPARE_SENT;
    currentState.prepareVotes[m_nodeName] = SignString(blockHash + "PREPARE");
    ndn::Name interestName("/vanet/pbft/prepare");
    interestName.append(blockHash).append(m_nodeName);
    auto interest = std::make_shared<ndn::Interest>(interestName);
    interest->setNonce(GetNonce());
    interest->setInterestLifetime(ndn::time::seconds(15));
    if (m_metricsCollector) {
        uint32_t messageSize = 500;
        m_metricsCollector->RecordCommunicationOverhead("PBFT_Prepare", messageSize, 
                                                       m_nodeName, "ALL_RSUS", true);
    }
    SendInterest(interest);
}

void VanetBlockchainApp::HandlePrepare(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();
    std::string blockHash = name.get(3).toUri();
    std::string voterId = name.get(4).toUri();

    NFD_LOG_INFO("[" << m_nodeName << "] Received PREPARE from " << voterId
                 << " for " << blockHash.substr(0,8));

    if (!m_pbftActiveConsensus.count(blockHash)) {
        NFD_LOG_DEBUG("[" << m_nodeName << "] No active consensus for " << blockHash.substr(0,8));
        return;
    }

    PbftBlockState& state = m_pbftActiveConsensus[blockHash];

    // Don't count duplicate votes
    if (state.prepareVotes.count(voterId)) {
        return;
    }

    // Add the vote
    state.prepareVotes[voterId] = "prepare_vote";

    uint32_t prepareCount = state.prepareVotes.size();
    uint32_t required = 2 * m_f_rsus + 1;

    NFD_LOG_INFO("[" << m_nodeName << "] Block " << blockHash.substr(0,8)
                 << " has " << prepareCount << "/" << required << " prepare votes");

    // Check if we have enough votes and haven't sent commit yet
    if (prepareCount >= required && state.phase < COMMIT_SENT) {
        NFD_LOG_INFO("[" << m_nodeName << "] Enough PREPARE votes. Moving to COMMIT phase.");
        BroadcastCommit(blockHash);
    }
}

void VanetBlockchainApp::BroadcastCommit(const std::string& blockHash) {
    if (!m_pbftActiveConsensus.count(blockHash)) return;



    PbftBlockState& currentState = m_pbftActiveConsensus[blockHash];
    currentState.phase = COMMIT_SENT;
    currentState.commitVotes[m_nodeName] = SignString(blockHash + "COMMIT");
    ndn::Name interestName("/vanet/pbft/commit");
    interestName.append(blockHash).append(m_nodeName);
    auto interest = std::make_shared<ndn::Interest>(interestName);
    interest->setNonce(GetNonce());
    interest->setInterestLifetime(ndn::time::seconds(15));
    if (m_metricsCollector) {
        uint32_t messageSize = 500;
        m_metricsCollector->RecordCommunicationOverhead("PBFT_Commit", messageSize, 
                                                       m_nodeName, "ALL_RSUS", true);
    }
    SendInterest(interest);
}

void VanetBlockchainApp::HandleCommit(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();
    std::string blockHash = name.get(3).toUri();
    std::string voterId = name.get(4).toUri();

    NFD_LOG_INFO("[" << m_nodeName << "] Received COMMIT from " << voterId
                 << " for " << blockHash.substr(0,8));

    if (!m_pbftActiveConsensus.count(blockHash)) {
        return;
    }

    PbftBlockState& state = m_pbftActiveConsensus[blockHash];

    if (state.phase == COMMITTED) {
        return;
    }

    if (state.commitVotes.count(voterId)) {
        return;
    }

    state.commitVotes[voterId] = "commit_vote";

    uint32_t commitCount = state.commitVotes.size();
    uint32_t required = 2 * m_f_rsus + 1;

    NFD_LOG_INFO("[" << m_nodeName << "] Block " << blockHash.substr(0,8)
                 << " has " << commitCount << "/" << required << " commit votes");

    if (commitCount >= required) {
        NFD_LOG_INFO("[" << m_nodeName << "] === CONSENSUS ACHIEVED === for block "
                     << blockHash.substr(0,8) << " at height " << state.block.height);

        state.phase = COMMITTED;


        AddBlockToChain(state.block);
        // Clear only the transactions included in this block from the pool
        std::vector<Transaction> remainingTxs;
        for (const auto& poolTx : m_transactionPool) {
            bool included = false;
            for (const auto& blockTx : state.block.transactions) {
                // This serialization needs to be consistent and robust for comparison
                if (poolTx.serialize() == blockTx.serialize()) {
                    included = true;
                    break;
                }
            }
            if (!included) {
                remainingTxs.push_back(poolTx);
            }
        }
        m_transactionPool = remainingTxs;

        NFD_LOG_INFO("[" << m_nodeName << "] Cleaned transaction pool. Remaining: "
                     << m_transactionPool.size());

        m_pbftActiveConsensus.erase(blockHash);
        m_pendingPbftBlocks.erase(blockHash);
    }
}

std::string Transaction::serialize() const {
    std::stringstream ss;

    if (type == REGISTRATION) {
        if (vehicleId_reg.empty() || publicKey.empty()) {
            NS_LOG_ERROR("Empty fields in registration transaction!"); // Use NS_LOG_ERROR for ns-3 logging
            return "REG:ERROR:ERROR:0.5";
        }
        ss << "REG:" << vehicleId_reg << ":" << publicKey << ":" << initialReputation;
    }
    else if (type == REPUTATION_UPDATE) {
        ss << "REP:" << vehicleId_rep << ":" << oldReputation << ":" << newReputation;
    }
    else if (type == EVENT_DECISION) {
        ss << "EVT:" << eventId_dec << ":" << eventVerdict; // Simplified for now
    }
    else {
        ss << "UNK:UNKNOWN";
        NS_LOG_WARN("Transaction::serialize - Unknown type: " << static_cast<int>(type)); // Use NS_LOG_WARN
    }

    std::string result = ss.str();
    if (result.empty()) {
        NS_LOG_ERROR("Empty serialization result!"); // Use NS_LOG_ERROR
        return "ERROR:EMPTY";
    }

    return result;
}

bool VanetBlockchainApp::ValidateBlock(const VanetBlock& block) {
    if (block.blockHash != block.calculateHash()) { NFD_LOG_WARN("Hash mismatch!"); return false; }
    return true;
}

void VanetBlockchainApp::AddBlockToChain(const VanetBlock& block) {

    if (block.previousHash != m_localBlockchain.back().blockHash) {
        NFD_LOG_ERROR("[" << m_nodeName << "] Block previous hash mismatch!");
        return;
    }

    for (const auto& existingBlock : m_localBlockchain) {
        if (existingBlock.blockHash == block.blockHash) {
            NFD_LOG_DEBUG("[" << m_nodeName << "] Block already in chain");
            return;
        }
    }

    Time blockAddTime = Simulator::Now();
    Time blockCreationTime = blockAddTime; 

    auto it = m_blockCreationTimes.find(block.blockHash);
    if (it != m_blockCreationTimes.end()) {
        blockCreationTime = it->second;
        m_blockCreationTimes.erase(it);
    }
    
    double blockProcessingTime = (double)blockAddTime.GetSeconds() - (double)blockCreationTime.GetSeconds();
  
    m_blockProcessingTimes.push_back(blockProcessingTime + 1);

    m_localBlockchain.push_back(block);


    m_transactionsSinceLastTps += block.transactions.size();
    if (m_nodeName == "RSU-0") {
        m_totalBlocksProposed++;
    }

    std::string roleStr = (m_nodeName == "RSU-0") ? "LEADER" : "FOLLOWER";
    NFD_LOG_INFO("[" << m_nodeName << "] ===== " << roleStr << " BLOCK " << block.height << " ADDED ===== "
                 << "Hash: " << block.blockHash.substr(0,8) << ", "
                 << "Transactions: " << block.transactions.size() << ", "
                 << "Chain Length: " << m_localBlockchain.size());


    std::vector<std::string> newlyRegisteredVehicles;

    for (const auto& tx : block.transactions) {
        if (tx.type == REGISTRATION) {
            m_vehicleKeys[tx.vehicleId_reg] = tx.publicKey;
            m_vehicleReputations[tx.vehicleId_reg] = tx.initialReputation;
            newlyRegisteredVehicles.push_back(tx.vehicleId_reg);


            NFD_LOG_INFO("[" << m_nodeName << "] " << roleStr << " REGISTERED: " << tx.vehicleId_reg
                        << " with key " << tx.publicKey.substr(0, 20) << "... and reputation " << tx.initialReputation);
        }
        else if (tx.type == REPUTATION_UPDATE) {
            m_vehicleReputations[tx.vehicleId_rep] = tx.newReputation; // This should already be updated by UpdateReputations before tx creation
            NFD_LOG_INFO("[" << m_nodeName << "] " << roleStr << " REPUTATION UPDATE (from block): " << tx.vehicleId_rep
                         << " new reputation " << tx.newReputation);
        }
        else if (tx.type == EVENT_DECISION) {
            NFD_LOG_INFO("[" << m_nodeName << "] " << roleStr << " EVENT DECISION (from block): " << tx.eventId_dec
                         << " verdict: " << tx.eventVerdict);
        }
        if (m_metricsCollector) {
            std::string txId = GenerateTransactionId(tx);
            std::string txType = TransactionTypeToString(tx.type);
            
            Time submissionTime = Seconds(static_cast<double>(tx.timestamp) / 1e9);
            Time processedTime = Simulator::Now();
            Time blockInclusionTime = Simulator::Now();
            
            // Calculate transaction latency
            Time latency = blockInclusionTime - submissionTime;
            
            m_metricsCollector->RecordTransaction(
                txId, txType, submissionTime, processedTime,
                blockInclusionTime, true, block.height, 
                block.proposerId, block.transactions.size()
            );
            
        
            uint32_t txSize = 500;
            m_metricsCollector->RecordCommunicationOverhead(
                "Transaction_" + txType, txSize, 
                block.proposerId, "ALL_RSUS", false
            );
            
            NFD_LOG_DEBUG("[" << m_nodeName << "] Transaction recorded: " << txId 
                         << " latency: " << latency.GetMilliSeconds() << "ms");
        }
    }

    CalculateCurrentTPS();

    if (m_metricsCollector) {
        uint32_t blockSize = EstimateBlockSize(block);
        m_metricsCollector->RecordCommunicationOverhead("Block", blockSize, block.proposerId, 
                                                       "ALL_RSUS", true);
        
        // Record throughput snapshot
        uint32_t pendingTxs = m_transactionPool.size();
        double avgReputation = CalculateAverageReputation(); // Calculate if needed
        m_metricsCollector->RecordPerformanceSnapshot(m_vehicleKeys.size(), block.height,
                                                     pendingTxs, avgReputation, 0.4, 0.6);
    }
    // Send deferred ACKs AND broadcast confirmations for newly registered vehicles
    for (const std::string& vehicleId : newlyRegisteredVehicles) {
        SendDeferredRegistrationAck(vehicleId);
    }

    // Log current state
    NFD_LOG_INFO("[" << m_nodeName << "] " << roleStr << " Blockchain State - Height: " << m_localBlockchain.size() - 1
                 << ", Registered Vehicles: " << m_vehicleKeys.size());
}

std::string VanetBlockchainApp::SignString(const std::string& data) {
    std::hash<std::string> hasher; // Placeholder
    return "Sig(" + data + ")_by_" + m_nodeName + "_" + std::to_string(hasher(data + m_nodeName + "_privKey"));
}


void VanetBlockchainApp::ProcessReceivedBlockForPbft(const VanetBlock& block, const std::string& originalProposerId) {
    if (m_nodeType != RSU_VALIDATOR) return;

    NFD_LOG_INFO("[" << m_nodeName << "] Processing received block " << block.blockHash.substr(0,8)
                     << " (Proposed by: " << block.proposerId << ", Received from via PrePrepare: " << originalProposerId
                     << ") for PBFT.");

    // Basic validation of the received block
    if (!ValidateBlock(block)) {
        NFD_LOG_WARN("[" << m_nodeName << "] PRE-PREPARE: Received Block " << block.blockHash.substr(0,8) << " validation failed.");
        return;
    }

    if (block.previousHash != m_localBlockchain.back().blockHash) {
        NFD_LOG_WARN("[" << m_nodeName << "] PRE-PREPARE: Received Block " << block.blockHash.substr(0,8)
                        << " has invalid previousHash. Expected: " << m_localBlockchain.back().blockHash
                        << ", Got: " << block.previousHash);
        return;
    }

    if (block.proposerId != originalProposerId) {
        NFD_LOG_WARN("[" << m_nodeName << "] PRE-PREPARE: Block's proposerId (" << block.proposerId
                        << ") does not match the Pre-Prepare sender (" << originalProposerId << "). Ignoring.");
        return;
    }


    PbftBlockState newState; // Ensure this type matches your struct definition in .hpp
    newState.block = block;
    newState.phase = PRE_PREPARE_RECEIVED; // Mark that we've got the block and validated the pre-prepare
    newState.proposerId = originalProposerId;

    newState.view = m_pbftActiveConsensus.count(block.blockHash) ? m_pbftActiveConsensus[block.blockHash].view : m_pbftCurrentView;
    newState.seqNum = m_pbftActiveConsensus.count(block.blockHash) ? m_pbftActiveConsensus[block.blockHash].seqNum : m_pbftCurrentSeqNum; // This needs careful handling

    m_pbftActiveConsensus[block.blockHash] = newState;
    m_pendingPbftBlocks[block.blockHash] = block; // Store the actual block data

    NFD_LOG_INFO("[" << m_nodeName << "] PRE-PREPARE: Block " << block.blockHash.substr(0,8)
                     << " (H:" << block.height << ") processed. Broadcasting PREPARE.");
    BroadcastPrepare(block.blockHash);
}


bool VanetBlockchainApp::VerifyVehicleSignature(const EventReport& report) {
    if (m_nodeType != RSU_VALIDATOR) {
        NFD_LOG_ERROR("[" << m_nodeName << "] VerifyVehicleSignature called by non-RSU node.");
        return false;
    }

    auto it = m_vehicleKeys.find(report.vehicleId);
    if (it == m_vehicleKeys.end()) {
        NFD_LOG_WARN("[" << m_nodeName << "] Signature check failed: Vehicle " << report.vehicleId
                     << " not registered (no public key found). Current registered vehicles: "
                     << m_vehicleKeys.size() << ". Dropping report.");

        // Log currently registered vehicles for debugging
        NFD_LOG_DEBUG("[" << m_nodeName << "] Currently registered vehicles:");
        for (const auto& pair : m_vehicleKeys) {
            NFD_LOG_DEBUG("  - " << pair.first);
        }
        return false;
    }
    std::string publicKey = it->second;
    NFD_LOG_DEBUG("[" << m_nodeName << "] Found public key for " << report.vehicleId << ": "
                 << publicKey.substr(0, 20) << "...");

    std::string locationStr;
    std::stringstream locStream;

    if (report.location.x == static_cast<int>(report.location.x) &&
        report.location.y == static_cast<int>(report.location.y)) {
        locStream << static_cast<int>(report.location.x) << "_" << static_cast<int>(report.location.y);
    } else {
        locStream << std::fixed << std::setprecision(2) << report.location.x << "_" << report.location.y;
    }
    locationStr = locStream.str();

    std::string dataToVerify = report.vehicleId + ";" + report.reportedEventType + ";" +
                               locationStr + ";" +
                               std::to_string(static_cast<long long>(report.timestamp.GetSeconds())) + ";" +
                               std::to_string(report.seqNum);

    std::hash<std::string> hasher;
    std::string pseudoPrivateKeyComponent = "PrivKey_" + report.vehicleId + "_" + std::to_string(hasher(report.vehicleId + "priv"));
    std::string expectedSignature = "Sig(" + dataToVerify + ")_by_" + report.vehicleId + "_" + std::to_string(hasher(dataToVerify + pseudoPrivateKeyComponent));

    if (report.signature == expectedSignature) {
        NFD_LOG_INFO("[" << m_nodeName << "] *** SIGNATURE VERIFIED *** for " << report.vehicleId
                     << " (registered vehicle with " << m_vehicleKeys.size() << " total registered)");
        return true;
    } else {
        NFD_LOG_WARN("[" << m_nodeName << "] Signature verification FAILED for " << report.vehicleId
                     << ". Expected: " << expectedSignature.substr(0, 50) << "..."
                     << ", Got: " << report.signature.substr(0, 50) << "...");
        return false;
    }
}


void VanetBlockchainApp::ForwardTransactionToLeader(const Transaction& tx) {
    if (m_nodeName == "RSU-0") {

        NS_LOG_INFO("[" << m_nodeName << "] Leader received transaction, adding to adaptive batch");
        m_adaptiveBatchManager.AddTransaction(tx); // CHANGED: Use adaptive batch manager
        return;
    }
    
    // Non-leaders forward to RSU-0
    NS_LOG_INFO("[" << m_nodeName << "] Forwarding transaction to RSU-0");
    SendTransactionForward(tx, "RSU-0");
}

void VanetBlockchainApp::SendTransactionForward(const Transaction& tx, const std::string& targetRsu) {
    NFD_LOG_INFO("[" << m_nodeName << "] *** SENDING TRANSACTION FORWARD *** to " << targetRsu);

    ndn::Name forwardName("/vanet");
    forwardName.append(targetRsu);
    forwardName.append("forward-transaction");
    forwardName.append(m_nodeName); // Source RSU
    forwardName.appendTimestamp();

    auto interest = std::make_shared<ndn::Interest>(forwardName);

    std::string txData = tx.serialize();
    auto buffer = std::make_shared<::ndn::Buffer>(txData.begin(), txData.end());
    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);

    interest->setNonce(GetNonce());
    interest->setInterestLifetime(ndn::time::seconds(15)); // Longer timeout for debugging

    NFD_LOG_INFO("[" << m_nodeName << "] Forward Interest details:");
    NFD_LOG_INFO("  Name: " << interest->getName());
    NFD_LOG_INFO("  Nonce: " << std::hex << interest->getNonce() << std::dec);
    NFD_LOG_INFO("  Lifetime: " << interest->getInterestLifetime());
    NFD_LOG_INFO("  Transaction: " << txData);
    NFD_LOG_INFO("  Target: " << targetRsu);

    // DEBUG: Check our CSMA face setup
    Ptr<Node> myNode = GetNode();
    Ptr<ns3::ndn::L3Protocol> l3 = myNode->GetObject<ns3::ndn::L3Protocol>();

    if (l3) {
        NFD_LOG_INFO("[" << m_nodeName << "] Checking CSMA connectivity for forwarding:");

        // Find CSMA device
        Ptr<NetDevice> csmaDevice = nullptr;
        for (uint32_t d = 0; d < myNode->GetNDevices(); ++d) {
            Ptr<NetDevice> dev = myNode->GetDevice(d);
            // NFD_LOG_INFO("  Device " << d << ": " << dev->GetInstanceTypeId().GetName());
            if (dev && dev->GetInstanceTypeId().GetName().find("CsmaNetDevice") != std::string::npos) {
                csmaDevice = dev;
                NFD_LOG_INFO(" Found CSMA device at index " << d << " Type: " << dev->GetInstanceTypeId().GetName());
                break;
            }
        }

        if (csmaDevice) {
            auto csmaFace = l3->getFaceByNetDevice(csmaDevice);
            if (csmaFace) {
                NFD_LOG_INFO("[" << m_nodeName << "]  Using CSMA face ID: " << csmaFace->getId()
                             << " for forwarding to " << targetRsu);
                NFD_LOG_INFO("  Face URI: " << csmaFace->getRemoteUri()); 
            } else {
                NFD_LOG_ERROR("[" << m_nodeName << "]  CSMA device found but no NDN face - CRITICAL!");
                return;
            }
        } else {
            NFD_LOG_ERROR("[" << m_nodeName << "] NO CSMA device found for inter-RSU communication - CRITICAL!");

            NFD_LOG_ERROR("[" << m_nodeName << "] Available devices:");
            for (uint32_t d = 0; d < myNode->GetNDevices(); ++d) {
                Ptr<NetDevice> dev = myNode->GetDevice(d);
                NFD_LOG_ERROR("  Device " << d << ": " << dev->GetInstanceTypeId().GetName());
            }
            return; // Don't send if no CSMA device
        }
    } else {
        NFD_LOG_ERROR("[" << m_nodeName << "] No NDN L3 protocol - cannot forward transaction!");
        return;
    }

    // Send the Interest
    NFD_LOG_INFO("[" << m_nodeName << "] Transmitting forward Interest...");
    m_transmittedInterests(interest, this, m_face); 
    m_appLink->onReceiveInterest(*interest);

    NFD_LOG_INFO("[" << m_nodeName << "] *** TRANSACTION FORWARD SENT *** Interest transmitted");
}


void VanetBlockchainApp::HandleForwardedTransaction(std::shared_ptr<const ndn::Interest> interest) {
    NFD_LOG_INFO("[" << m_nodeName << "] Received forwarded transaction");
    
    if (m_nodeName != "RSU-0") {
        NFD_LOG_WARN("[" << m_nodeName << "] Received forwarded transaction but not leader. Ignoring.");
        return;
    }
    
    const ndn::Name& name = interest->getName();
    if (name.size() < 4) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid forwarded transaction Interest format");
        return;
    }
    
    std::string sourceRsu = name.get(3).toUri();
    
    if (interest->getApplicationParameters().value_size() == 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Forwarded transaction has no payload from " << sourceRsu);
        return;
    }
    
    const auto& appParams = interest->getApplicationParameters();
    std::string txData(reinterpret_cast<const char*>(appParams.value()), appParams.value_size());
    
    NFD_LOG_INFO("[" << m_nodeName << "] Leader received transaction from " << sourceRsu << ": " << txData);
    
    // Parse and process transaction (existing logic...)
    Transaction tx;
    bool parsed = false;
    std::string vehicleId = "";
    
    // Parse transaction logic
    if (txData.find("REG:") == 0) {
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }
        
        if (parts.size() >= 4) {
            try {
                tx.type = REGISTRATION;
                tx.vehicleId_reg = parts[1];
                tx.publicKey = parts[2];
                tx.initialReputation = std::stod(parts[3]);
                tx.timestamp = Simulator::Now().GetTimeStep();
                vehicleId = tx.vehicleId_reg;
                parsed = true;
                
                // Track which RSU should send the ACK
                m_vehicleAckTargets[vehicleId] = sourceRsu;
                NFD_LOG_INFO("[" << m_nodeName << "] Marked " << sourceRsu 
                             << " as ACK target for " << vehicleId);
            } catch (const std::exception& e) {
                NFD_LOG_WARN("[" << m_nodeName << "] Error parsing registration: " << e.what());
            }
        }
    }
    else if (txData.find("EVT:") == 0) {
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }
        
        if (parts.size() >= 3) {
            tx.type = EVENT_DECISION;
            tx.eventId_dec = parts[1];
            tx.eventVerdict = parts[2];
            tx.timestamp = Simulator::Now().GetTimeStep();
            tx.eventType = "Unknown";
            tx.eventLocation = "Unknown";
            tx.eventTimestamp = 0;
            tx.eventCredibility = 0.0;
            parsed = true;
        }
    }
    else if (txData.find("REP:") == 0) {
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }
        
        if (parts.size() >= 4) {
            try {
                tx.type = REPUTATION_UPDATE;
                tx.vehicleId_rep = parts[1];
                tx.oldReputation = std::stod(parts[2]);
                tx.newReputation = std::stod(parts[3]);
                tx.timestamp = Simulator::Now().GetTimeStep();
                tx.eventId_rep = "Unknown";
                parsed = true;
            } catch (const std::exception& e) {
                NFD_LOG_WARN("[" << m_nodeName << "] Error parsing reputation update: " << e.what());
            }
        }
    }
    
    if (parsed) {
        // Add to leader's ADAPTIVE batch manager
        m_adaptiveBatchManager.AddTransaction(tx); // CHANGED: Use adaptive batch manager
        NFD_LOG_INFO("[" << m_nodeName << "] Transaction successfully added to adaptive batch from " << sourceRsu
                     << ". Batch size: " << m_adaptiveBatchManager.GetBatchBufferSize());
        
        // Broadcast ACK info for registrations
        if (tx.type == REGISTRATION) {
            BroadcastTransactionWithAckInfo(tx, sourceRsu);
        }
    } else {
        NFD_LOG_ERROR("[" << m_nodeName << "] Failed to parse forwarded transaction from " << sourceRsu);
    }
}

void VanetBlockchainApp::UpdateAdaptiveNetworkParameters() {
    if (m_nodeName != "RSU-0") {
        return; // Only leader needs to track network-wide parameters
    }
    
    // Calculate current network latency based on recent transaction processing times
    double estimatedLatency = 0.5; // Base latency
    
    // Adjust based on transaction pool size (congestion indicator)
    if (m_transactionPool.size() > 20) {
        estimatedLatency += (m_transactionPool.size() - 20) * 0.05; // +50ms per extra transaction
    }
    
    // Adjust based on recent TPS (lower TPS = higher latency)
    if (m_currentTps > 0 && m_currentTps < 10.0) {
        estimatedLatency += (10.0 - m_currentTps) * 0.1; // Penalty for low TPS
    }
    
    // Update adaptive batch manager with current network conditions
    m_adaptiveBatchManager.UpdateNetworkParameters(m_totalVehicles, estimatedLatency);
    
    NS_LOG_DEBUG("[" << m_nodeName << "] Updated adaptive network parameters: "
                 << "vehicles=" << m_totalVehicles 
                 << ", estimated latency=" << estimatedLatency << "s"
                 << ", TPS=" << m_currentTps
                 << ", pool size=" << m_transactionPool.size());
}

void VanetBlockchainApp::BroadcastTransactionWithAckInfo(const Transaction& tx, const std::string& ackTarget) {
    if (m_nodeName != "RSU-0") {
        NFD_LOG_DEBUG("[" << m_nodeName << "] Not leader, not broadcasting transaction with ACK info");
        return;
    }

    ndn::Name txInterestName("/vanet/pbft/transaction-with-ack");
    txInterestName.append(m_nodeName); // RSU-0
    txInterestName.append(ackTarget);  // Which RSU should send ACK
    txInterestName.appendTimestamp();

    auto interest = std::make_shared<ndn::Interest>(txInterestName);

    std::string txData = tx.serialize();
    auto buffer = std::make_shared<::ndn::Buffer>(txData.begin(), txData.end());
    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);

    interest->setNonce(GetNonce());
    interest->setInterestLifetime(ndn::time::seconds(15));

    NFD_LOG_INFO("[" << m_nodeName << "] Broadcasting transaction with ACK target "
                 << ackTarget << ": " << txData);
    SendInterest(interest);
}

void VanetBlockchainApp::HandleTransactionWithAckBroadcast(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();

    if (name.size() < 5) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid transaction-with-ack Interest format");
        return;
    }

    std::string leader = name.get(3).toUri();
    std::string ackTarget = name.get(4).toUri();

    if (interest->getApplicationParameters().value_size() == 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Transaction-with-ack broadcast has no payload");
        return;
    }

    const auto& appParams = interest->getApplicationParameters();
    std::string txData(reinterpret_cast<const char*>(appParams.value()), appParams.value_size());

    NFD_LOG_INFO("[" << m_nodeName << "] Received transaction-with-ack from " << leader
                 << ", ACK target: " << ackTarget << ", tx: " << txData);

    // Parse transaction (same logic as HandleTransactionBroadcast)
    Transaction tx;
    bool parsed = false;
    std::string vehicleId = "";

    if (txData.find("REG:") == 0) {
        std::vector<std::string> parts;
        std::stringstream ss(txData);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }

        if (parts.size() >= 4) {
            tx.type = REGISTRATION;
            tx.vehicleId_reg = parts[1];
            tx.publicKey = parts[2];
            tx.initialReputation = std::stod(parts[3]);
            tx.timestamp = Simulator::Now().GetTimeStep();
            vehicleId = tx.vehicleId_reg;
            parsed = true;

            // FIXED: Store ACK target info on ALL RSUs using the correct ackTarget
            m_vehicleAckTargets[vehicleId] = ackTarget;
            NFD_LOG_INFO("[" << m_nodeName << "] Set ACK target for " << vehicleId
                         << " to " << ackTarget);
        }
    }


    if (parsed && tx.type == REGISTRATION) {
        bool alreadyHave = false;
        for (const auto& poolTx : m_transactionPool) {
            if (poolTx.type == tx.type && tx.type == REGISTRATION &&
                poolTx.vehicleId_reg == tx.vehicleId_reg) {
                alreadyHave = true;
                break;
            }
        }

        if (!alreadyHave) {
            AddTransactionToPool(tx); // Followers add to their pool to validate against the block
            NFD_LOG_INFO("[" << m_nodeName << "] Added transaction with ACK target. Pool size: "
                         << m_transactionPool.size());
        }
    } else if (parsed) {
        NFD_LOG_INFO("[" << m_nodeName << "] Parsed non-registration tx with ACK info, but not adding to follower pool directly: " << txData);
    } else {
        NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse tx from transaction-with-ack: " << txData);
    }
}


void VanetBlockchainApp::SendDeferredRegistrationAck(const std::string& vehicleId) {
    auto ackTargetIt = m_vehicleAckTargets.find(vehicleId);
    if (ackTargetIt != m_vehicleAckTargets.end() && ackTargetIt->second != m_nodeName) {
        NFD_LOG_DEBUG("[" << m_nodeName << "] ACK for " << vehicleId
                     << " should be sent by " << ackTargetIt->second << ", not us");
        return;
    }

    auto it = m_pendingRegistrations.find(vehicleId);
    ndn::Name ackName;
    Time requestTime = Simulator::Now(); // Default if not found

    if (it != m_pendingRegistrations.end()) {
        // Use original interest name if available
        const PendingRegistration& pending = it->second;
        ackName = pending.originalInterestName;
        ackName.append("ack");
        requestTime = pending.requestTime; // Get the actual request time
        m_pendingRegistrations.erase(it); // Clean up pending registration
    } else {
        // Create a generic ACK name
        ackName = ndn::Name("/vanet");
        ackName.append(m_nodeName);
        ackName.append("register");
        ackName.append(vehicleId);
        ackName.append("KEY_CONFIRMED");
        ackName.append("ack");
        NFD_LOG_INFO("[" << m_nodeName << "] Original interest not found for " << vehicleId 
                     << ", using generic ACK name: " << ackName);
    }


    if (!IsVehicleRegistered(vehicleId)) {
        NFD_LOG_WARN("[" << m_nodeName << "] Vehicle " << vehicleId
                     << " still not registered after consensus, cannot send ACK");
        return;
    }

    auto data = std::make_shared<ndn::Data>(ackName);
    data->setFreshnessPeriod(ndn::time::seconds(10));


    std::string ackContent = "REG_CONFIRMED_BY_" + m_nodeName + "_AFTER_CONSENSUS";
    data->setContent(std::make_shared<::ndn::Buffer>(
        reinterpret_cast<const uint8_t*>(ackContent.c_str()), ackContent.size()));

    SendData(data);
    
    if (m_metricsCollector) {
        std::string ackContent = "REG_CONFIRMED_BY_" + m_nodeName + "_AFTER_CONSENSUS";
        uint32_t ackSize = 500;
        m_metricsCollector->RecordCommunicationOverhead("Data_RegistrationAck", ackSize, 
                                                       m_nodeName, vehicleId, false);
    }



    NFD_LOG_INFO("[" << m_nodeName << "] *** DEFERRED ACK SENT *** to " << vehicleId
                 << " after consensus completion with name: " << ackName);
}



// Check if vehicle is registered
bool VanetBlockchainApp::IsVehicleRegistered(const std::string& vehicleId) const {
    return m_vehicleKeys.find(vehicleId) != m_vehicleKeys.end();
}


void VanetBlockchainApp::HandleTransactionBatch(std::shared_ptr<const ndn::Interest> interest) {
    if (m_nodeName == "RSU-0") {
        NS_LOG_DEBUG("[" << m_nodeName << "] Leader ignoring own batch broadcast");
        return;
    }
    
    const ndn::Name& name = interest->getName();
    if (name.size() < 4) {
        NS_LOG_WARN("[" << m_nodeName << "] Invalid batch Interest format");
        return;
    }
    
    std::string sender = name.get(3).toUri();
    if (sender != "RSU-0") {
        NS_LOG_WARN("[" << m_nodeName << "] Ignoring batch from non-leader: " << sender);
        return;
    }
    
    if (interest->getApplicationParameters().value_size() == 0) {
        NS_LOG_WARN("[" << m_nodeName << "] Empty batch from " << sender);
        return;
    }
    
    const auto& appParams = interest->getApplicationParameters();
    std::string batchData(reinterpret_cast<const char*>(appParams.value()), 
                         appParams.value_size());
    
    std::vector<Transaction> parsedTransactions = ParseTransactionBatch(batchData);
    
    NS_LOG_INFO("[" << m_nodeName << "] Received batch from " << sender 
               << ": " << parsedTransactions.size() << " transactions");
    
    // Add transactions to pool with duplicate checking
    size_t successCount = 0;
    for (const auto& tx : parsedTransactions) {
        bool isDuplicate = false;
        std::string newTxSerialized = tx.serialize();
        
        for (const auto& poolTx : m_transactionPool) {
            if (poolTx.serialize() == newTxSerialized) {
                isDuplicate = true;
                break;
            }
        }
        
        if (!isDuplicate) {
            AddTransactionToPool(tx);
            successCount++;
        }
    }
    
    NS_LOG_INFO("[" << m_nodeName << "] Added " << successCount << "/" << parsedTransactions.size() 
               << " new transactions. Pool size: " << m_transactionPool.size());
}


void VanetBlockchainApp::ProposeBlockForBatch() {
    if (m_transactionPool.empty()) {
        NS_LOG_DEBUG("[" << m_nodeName << "] No transactions for batch proposal");
        return;
    }
    
    if (m_nodeName != "RSU-0") {
        NS_LOG_DEBUG("[" << m_nodeName << "] Not RSU-0, cannot propose blocks");
        return;
    }
    
    uint32_t nextHeight = m_localBlockchain.back().height + 1;
    
    // Check for existing consensus at this height
    for (const auto& pair : m_pbftActiveConsensus) {
        if (pair.second.block.height == nextHeight) {
            NS_LOG_WARN("[" << m_nodeName << "] Already processing height " << nextHeight);
            return;
        }
    }
    
    // BATCH PROPOSALS BYPASS COOLDOWNS for efficiency
    NS_LOG_INFO("[" << m_nodeName << "] *** BATCH BLOCK PROPOSAL *** height " << nextHeight
                << " with " << m_transactionPool.size() << " transactions");
    
    m_lastBlockProposalTime = Simulator::Now();
    VanetBlock newBlock = CreateCandidateBlock();
    StartPbft(newBlock);
}


std::vector<Transaction> VanetBlockchainApp::ParseTransactionBatch(const std::string& batchData) {
    std::vector<Transaction> transactions;
    
    std::vector<std::string> parts;
    std::stringstream ss(batchData);
    std::string part;
    while (std::getline(ss, part, '|')) {
        parts.push_back(part);
    }
    
    if (parts.empty() || parts[0].find("BATCH:") != 0) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid batch format");
        return transactions;
    }
    
    // Extract batch size with error checking
    if (parts[0].length() <= 6) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid BATCH prefix");
        return transactions;
    }
    
    size_t batchSize = 0;
    try {
        batchSize = std::stoul(parts[0].substr(6)); // Remove "BATCH:"
    } catch (const std::exception& e) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid batch size: " << e.what());
        return transactions;
    }
    
    // Parse each transaction
    for (size_t i = 1; i < parts.size() && (i-1) < batchSize; ++i) {
        std::string txPart = parts[i];
        
        // Remove TX prefix: "TX0:REG:..." -> "REG:..."
        size_t colonPos = txPart.find(':');
        if (colonPos == std::string::npos || txPart.length() <= colonPos + 1) {
            NFD_LOG_WARN("[" << m_nodeName << "] Malformed TX part: " << txPart);
            continue;
        }
        
        std::string txData = txPart.substr(colonPos + 1);
        Transaction tx;
        bool parsed = false;
        
        if (txData.find("REG:") == 0) {
            // Parse: "REG:vehicleId:publicKey:initialRep"
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 4) {
                try {
                    tx.type = REGISTRATION;
                    tx.vehicleId_reg = txParts[1];
                    tx.publicKey = txParts[2];
                    tx.initialReputation = std::stod(txParts[3]);
                    tx.timestamp = Simulator::Now().GetTimeStep();
                    parsed = true;
                } catch (const std::exception& e) {
                    NFD_LOG_WARN("[" << m_nodeName << "] Error parsing REG transaction: " << e.what());
                }
            }
        }
        else if (txData.find("EVT:") == 0) {
            // Parse: "EVT:eventId:verdict"
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 3) {
                tx.type = EVENT_DECISION;
                tx.eventId_dec = txParts[1];
                tx.eventVerdict = txParts[2];
                tx.timestamp = Simulator::Now().GetTimeStep();
                tx.eventType = "Unknown";
                tx.eventLocation = "Unknown";
                tx.eventTimestamp = 0;
                tx.eventCredibility = 0.0;
                parsed = true;
            }
        }
        else if (txData.find("REP:") == 0) {
            // Parse: "REP:vehicleId:oldRep:newRep"
            std::vector<std::string> txParts;
            std::stringstream txSS(txData);
            std::string txSegment;
            while (std::getline(txSS, txSegment, ':')) {
                txParts.push_back(txSegment);
            }
            if (txParts.size() >= 4) {
                try {
                    tx.type = REPUTATION_UPDATE;
                    tx.vehicleId_rep = txParts[1];
                    tx.oldReputation = std::stod(txParts[2]);
                    tx.newReputation = std::stod(txParts[3]);
                    tx.timestamp = Simulator::Now().GetTimeStep();
                    tx.eventId_rep = "Unknown";
                    parsed = true;
                } catch (const std::exception& e) {
                    NFD_LOG_WARN("[" << m_nodeName << "] Error parsing REP transaction: " << e.what());
                }
            }
        }
        
        if (parsed) {
            transactions.push_back(tx);
        } else {
            NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse transaction: " << txData);
        }
    }
    
    if (transactions.size() != batchSize) {
        NFD_LOG_WARN("[" << m_nodeName << "] Batch size mismatch: expected " << batchSize 
                     << ", parsed " << transactions.size());
    }
    
    return transactions;
}


// ADD GetNodeName method to VanetBlockchainApp:
std::string VanetBlockchainApp::GetNodeName() const {
    return m_nodeName;
}


void VanetBlockchainApp::RecordCommunication(const std::string& packetType, uint32_t packetSize) {
    m_communicationData.totalNdnPackets++;
    m_communicationData.totalDataSize += packetSize;
    
    if (packetType.find("Interest") != std::string::npos) {
        m_communicationData.interestPackets++;
    } else if (packetType.find("Data") != std::string::npos) {
        m_communicationData.dataPackets++;
    }
    
    if (packetType.find("Registration") != std::string::npos || packetType.find("register") != std::string::npos) {
        m_communicationData.registrationMessages++;
    } else if (packetType.find("EventReport") != std::string::npos || packetType.find("eventreport") != std::string::npos) {
        m_communicationData.eventReportMessages++;
    } else if (packetType.find("PBFT") != std::string::npos || packetType.find("pbft") != std::string::npos) {
        m_communicationData.pbftMessages++;
    } else if (packetType.find("blockchain") != std::string::npos) {
        m_communicationData.blockchainQueryMessages++;
    }
    
    
    // Update network utilization estimate
    static Time lastUpdate = Seconds(0);
    Time now = Simulator::Now();
    if (now - lastUpdate >= Seconds(1.0)) { // Update every second
        double utilizationEstimate = std::min<double>(1.0, static_cast<double>(m_communicationData.totalNdnPackets) / 10000.0);
        lastUpdate = now;
    }
}

void VanetBlockchainApp::SetTotalVehicles(uint32_t totalVehicles) {
    m_totalVehicles = totalVehicles;
}

// Add method to set total attackers for metrics context
void VanetBlockchainApp::SetTotalAttackers(uint32_t totalAttackers) {
    m_totalAttackers = totalAttackers;
}

bool VanetBlockchainApp::IsVehicleActuallyAttacker(const std::string& vehicleId) const {
    // Return the BASE attacker status (original assignment)
    auto it = m_vehicleBaseAttackerStatus.find(vehicleId);
    if (it != m_vehicleBaseAttackerStatus.end()) {
        return it->second;
    }
    
    // Fallback: use the old method for vehicles not explicitly tracked
    if (vehicleId.length() < 3 || vehicleId.substr(0, 2) != "V-") {
        return false;
    }
    
    try {
        uint32_t vehicleIndex = std::stoi(vehicleId.substr(2));
        uint32_t numAttackers = m_totalAttackers;
        if (m_totalVehicles > 0 && numAttackers > 0) {
            uint32_t firstAttackerIndex = m_totalVehicles - numAttackers;
            return vehicleIndex >= firstAttackerIndex;
        }
    } catch (const std::exception& e) {
        NFD_LOG_WARN("[" << m_nodeName << "] Failed to parse vehicle index from " << vehicleId);
    }
    
    return false;
}

bool VanetBlockchainApp::IsVehicleCurrentlyActingMalicious(const std::string& vehicleId) const {
    // Check if we have current behavior override
    auto it = m_vehicleCurrentBehavior.find(vehicleId);
    if (it != m_vehicleCurrentBehavior.end()) {
        NFD_LOG_DEBUG("[" << m_nodeName << "] " << vehicleId << " current behavior: " 
                     << (it->second ? "MALICIOUS" : "HONEST"));
        return it->second;
    }
    
    // Fallback to base attacker status
    bool isBaseAttacker = IsVehicleActuallyAttacker(vehicleId);
    NFD_LOG_DEBUG("[" << m_nodeName << "] " << vehicleId << " using base status: "
                 << (isBaseAttacker ? "ATTACKER" : "HONEST"));
    return isBaseAttacker;
}


std::string VanetBlockchainApp::GenerateTransactionId(const Transaction& tx) const {
    std::stringstream ss;
    ss << tx.type << "_" << tx.timestamp << "_";
    if (tx.type == REGISTRATION) {
        ss << tx.vehicleId_reg;
    } else if (tx.type == EVENT_DECISION) {
        ss << tx.eventId_dec;
    } else if (tx.type == REPUTATION_UPDATE) {
        ss << tx.vehicleId_rep;
    }
    return ss.str();
}

std::string VanetBlockchainApp::TransactionTypeToString(TransactionType type) const {
    switch (type) {
        case REGISTRATION: return "REGISTRATION";
        case EVENT_DECISION: return "EVENT_DECISION";
        case REPUTATION_UPDATE: return "REPUTATION_UPDATE";
        default: return "UNKNOWN";
    }
}

uint32_t VanetBlockchainApp::EstimateBlockSize(const VanetBlock& block) const {
    uint32_t size = 1; // Base block headers
    for (const auto& tx : block.transactions) {
        size += 5; // Transaction + overhead
    }
    return size;
}

void VanetBlockchainApp::SetMetricsCollector(Ptr<MetricsCollector> collector) {
    m_metricsCollector = collector;
    NS_LOG_INFO("[" << m_nodeName << "] MetricsCollector set");
}

void VanetBlockchainApp::SetVehicleCurrentEventIndex(const std::string& vehicleId, uint32_t eventIndex) {
    m_vehicleCurrentEventIndex[vehicleId] = eventIndex;
    NFD_LOG_DEBUG("[" << m_nodeName << "] Set event index for " << vehicleId << " to " << eventIndex);
}

void VanetBlockchainApp::Reset() {
    m_adaptiveBatchManager.Reset(); // ADD THIS LINE: Reset adaptive batch manager
    
    NS_LOG_INFO("VanetBlockchainApp reset with oracle-free behavioral inference and adaptive batch processing");
}

void VanetBlockchainApp::CalculateCurrentTPS() {
    Time currentTime = Simulator::Now();
    
    if (m_lastTpsCalculationTime > Seconds(0)) {
        double timeDiff = (currentTime - m_lastTpsCalculationTime).GetSeconds();
        if (timeDiff >= 60.0) { // Calculate TPS every minute
            m_currentTps = static_cast<double>(m_transactionsSinceLastTps) / timeDiff;
            m_tpsHistory.push_back(m_currentTps);
            
            NFD_LOG_INFO("[" << m_nodeName << "] REAL-TIME TPS: " << m_currentTps 
                       << " (Processed: " << m_transactionsSinceLastTps 
                       << " txs in " << timeDiff << "s)");
            
            // Record performance snapshot
            if (m_metricsCollector) {
                double avgReputation = CalculateAverageReputation();
                double attackerRep = CalculateAverageAttackerReputation();
                double honestRep = CalculateAverageHonestReputation();
                
                m_metricsCollector->RecordPerformanceSnapshot(
                    m_vehicleKeys.size(),           // registered vehicles
                    m_localBlockchain.size() - 1,  // blockchain height
                    m_transactionPool.size(),       // pending transactions
                    avgReputation,                  // average reputation
                    attackerRep,                    // average attacker reputation
                    honestRep                       // average honest reputation
                );
            }
            
            // Reset counters
            m_transactionsSinceLastTps = 0;
            m_lastTpsCalculationTime = currentTime;
        }
    } else {
        // First calculation
        m_lastTpsCalculationTime = currentTime;
        m_transactionsSinceLastTps = 0;
        m_currentTps = 0.0;
    }
}

double VanetBlockchainApp::CalculateAverageReputation() const {
    if (m_vehicleReputations.empty()) return 0.5;
    
    double total = 0.0;
    for (const auto& pair : m_vehicleReputations) {
        total += pair.second;
    }
    return total / m_vehicleReputations.size();
}

double VanetBlockchainApp::CalculateAverageAttackerReputation() const {
    double total = 0.0;
    uint32_t count = 0;
    
    for (const auto& pair : m_vehicleReputations) {
        if (IsVehicleActuallyAttacker(pair.first)) {
            total += pair.second;
            count++;
        }
    }
    return count > 0 ? (total / count) : 0.5;
}

double VanetBlockchainApp::CalculateAverageHonestReputation() const {
    double total = 0.0;
    uint32_t count = 0;
    
    for (const auto& pair : m_vehicleReputations) {
        if (!IsVehicleActuallyAttacker(pair.first)) {
            total += pair.second;
            count++;
        }
    }
    return count > 0 ? (total / count) : 0.5;
}

void VanetBlockchainApp::HandleLocationQueryInterest(std::shared_ptr<const ndn::Interest> interest) {
    const ndn::Name& name = interest->getName();
    
    // Parse: /vanet/{RSU}/location-query/{vehicleId}/{location}/{timestamp}
    if (name.size() < 6) {
        NFD_LOG_WARN("[" << m_nodeName << "] Invalid location query format: " << name);
        return;
    }
    
    std::string vehicleId = name.get(3).toUri();
    std::string queryLocation = name.get(4).toUri();
    Time requestTime = Simulator::Now();
    
    NFD_LOG_INFO("[" << m_nodeName << "] *** LOCATION QUERY *** from " << vehicleId 
                 << " about location: " << queryLocation);

    // Search blockchain for events at this location
    std::string responseContent = SearchLocationEvents(queryLocation);
    
    // Create response data
    auto data = std::make_shared<ndn::Data>(interest->getName());
    data->setFreshnessPeriod(ndn::time::seconds(60));
    
    if (!responseContent.empty()) {
        data->setContent(std::make_shared<::ndn::Buffer>(responseContent.begin(), responseContent.end()));
        NFD_LOG_INFO("[" << m_nodeName << "] Location query response for " << queryLocation 
                     << ": " << responseContent.length() << " characters");
    } else {
        std::string noDataResponse = "NO_DATA_FOR_LOCATION:" + queryLocation;
        data->setContent(std::make_shared<::ndn::Buffer>(noDataResponse.begin(), noDataResponse.end()));
        NFD_LOG_INFO("[" << m_nodeName << "] No data found for location " << queryLocation);
    }

    // Record communication overhead
    if (m_metricsCollector) {
        uint32_t responseSize = 500;
        m_metricsCollector->RecordCommunicationOverhead("Data_LocationQueryResponse", responseSize, 
                                                       m_nodeName, vehicleId, false);
    }

    SendData(data);
}

std::string VanetBlockchainApp::SearchLocationEvents(const std::string& queryLocation) const {
    std::vector<Transaction> relevantEvents;
    
    NFD_LOG_INFO("[" << m_nodeName << "] Searching blockchain for events at location: " << queryLocation);
    
    // Search through all blocks in the blockchain
    for (const auto& block : m_localBlockchain) {
        for (const auto& tx : block.transactions) {
            if (tx.type == EVENT_DECISION) {
                // Check if event location matches query (with tolerance for GPS coordinates)
                if (tx.eventLocation == queryLocation || 
                    IsLocationNearby(tx.eventLocation, queryLocation, 100.0)) { // 100m tolerance
                    relevantEvents.push_back(tx);
                    NFD_LOG_DEBUG("[" << m_nodeName << "] Found relevant event: " << tx.eventId_dec 
                                 << " at " << tx.eventLocation << " (verdict: " << tx.eventVerdict << ")");
                }
            }
        }
    }
    
    return FormatLocationQueryResponse(relevantEvents, queryLocation);
}

std::string VanetBlockchainApp::FormatLocationQueryResponse(const std::vector<Transaction>& relevantEvents, 
                                                          const std::string& queryLocation) const {
    if (relevantEvents.empty()) {
        return ""; 
    }
    
    std::stringstream response;
    response << "LOCATION_EVENTS:" << queryLocation << "|";
    response << "COUNT:" << relevantEvents.size() << "|";
    
    for (size_t i = 0; i < relevantEvents.size(); ++i) {
        const auto& event = relevantEvents[i];
        response << "EVENT" << i << ":"
                 << "ID=" << event.eventId_dec << ","
                 << "TYPE=" << event.eventType << ","
                 << "TIME=" << event.eventTimestamp << ","
                 << "VERDICT=" << event.eventVerdict << ","
                 << "CREDIBILITY=" << std::fixed << std::setprecision(2) << event.eventCredibility << ","
                 << "REPORTS=" << event.eventReports.size();
        
        if (i < relevantEvents.size() - 1) {
            response << "|";
        }
    }
    
    std::string result = response.str();
    NFD_LOG_INFO("[" << m_nodeName << "] Formatted response with " << relevantEvents.size() 
                 << " events, length: " << result.length() << " chars");
    
    return result;
}


bool VanetBlockchainApp::IsLocationNearby(const std::string& loc1, const std::string& loc2, double toleranceMeters) const {
    // Parse locations "x_y" format
    auto parseLocation = [](const std::string& loc) -> std::pair<double, double> {
        size_t underscorePos = loc.find('_');
        if (underscorePos != std::string::npos) {
            double x = std::stod(loc.substr(0, underscorePos));
            double y = std::stod(loc.substr(underscorePos + 1));
            return {x, y};
        }
        return {0.0, 0.0};
    };
    
    auto [x1, y1] = parseLocation(loc1);
    auto [x2, y2] = parseLocation(loc2);
    
    double distance = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    return distance <= toleranceMeters;
}


}

