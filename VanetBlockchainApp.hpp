#ifndef VANET_BLOCKCHAIN_APP_HPP
#define VANET_BLOCKCHAIN_APP_HPP

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "AttackerBehaviorPatterns.hpp" 
#include "AdaptiveBatchManager.hpp" 

#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "VanetBlock.hpp"
#include <list>
#include <unordered_map>
#include <vector>
#include <string>
#include "ns3/core-module.h"
#include "MetricsCollector.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace ns3 {

// Forward declaration
class MobilityModel;
class VanetBlockchainApp;

enum NodeType { RSU_VALIDATOR, VEHICLE_CLIENT };

struct EventReport {
    std::string vehicleId;
    std::string reportedEventType;
    std::string originalEventType;
    Vector location;
    Time timestamp;
    std::string signature;
    uint32_t seqNum;
    Time receivedTime;
};

struct BatchAckTracker {
    std::string batchId;
    uint32_t expectedAcks;
    uint32_t receivedAcks;
    Time sentTime;
    bool completed = false;
};

struct EventCluster {
    std::string eventId;
    std::string eventType;
    Vector centerLocation;
    Time centerTime;
    std::vector<EventReport> reports;
    Time creationTime;
    bool decisionMade = false;
    ns3::EventId decisionEvent;
    double c_event_value;     // Result of weighted vote consensus C_event(E)
    double consensusStrength; // Consensus Strength CS(E)
    double finalCredibility;  // Final Credibility Score C_final(E)
};

struct CommunicationDataMetrics {
    uint32_t totalNdnPackets = 0;
    uint64_t totalDataSize = 0; // Use uint64_t for potentially large data sizes
    uint32_t interestPackets = 0;
    uint32_t dataPackets = 0;
    uint32_t registrationMessages = 0;
    uint32_t eventReportMessages = 0;
    uint32_t pbftMessages = 0;
    uint32_t blockchainQueryMessages = 0;
};

class AdaptiveLearningRateCalculator {
public:
    struct LearningRates {
        double adaptive_alpha;
        double adaptive_beta;
        std::string reasoning;
    };

    LearningRates CalculateAdaptiveRates(
        const std::string& vehicleId,
        const std::unordered_map<std::string, double>& vehicleReputations,
        const std::unordered_map<std::string, uint32_t>& vehicleTotalReports,
        const std::unordered_map<std::string, uint32_t>& vehicleCorrectReports,
        double event_credibility,
        double consensus_strength,
        bool isCorrectReport);

private:
    double CalculateExperienceFactor(uint32_t totalReports, uint32_t correctReports);
    double CalculateConfidenceFactor(double event_credibility, double consensus_strength);
    double CalculateReputationFactor(double currentReputation, bool isCorrectReport);
};

// REMOVED: SimpleBatchManager class - replaced with AdaptiveBatchManager

class VanetBlockchainApp : public ndn::App {
public:
    static TypeId GetTypeId();
    VanetBlockchainApp();
    virtual void StartApplication();
    virtual void StopApplication();
    virtual void OnInterest(std::shared_ptr<const ndn::Interest> interest);
    virtual void OnData(std::shared_ptr<const ndn::Data> contentObject);

    void InitiateRegistration(const std::string& vehicleId, const std::string& publicKey);
    void InitiateEventDecision(const std::string& eventId, const std::string& eventType,
                               const std::string& location, uint64_t eventTime,
                               const std::vector<std::pair<std::string, std::string>>& reports,
                               const std::string& verdict, double credibility);
    void InitiateReputationUpdate(const std::string& vehicleId, const std::string& eventId,
                                  double oldRep, double newRep);
    void TryProposeNewBlock();
    void SetRsuList(const std::vector<std::string>& rsus);
    void SetNodeType(NodeType type);
    void SetMetricsCollector(Ptr<MetricsCollector> collector);
    std::string GetNodeName() const;
    void SendInterest(std::shared_ptr<const ndn::Interest> interest);
    void HandleTransactionBatch(std::shared_ptr<const ndn::Interest> interest);
    void HandleAdaptiveBatch(std::shared_ptr<const ndn::Interest> interest); // NEW: Handle adaptive batches
    void AddTransactionToPool(const Transaction& tx);
    size_t GetTransactionPoolSize() const { return m_transactionPool.size(); }
    void RecordCommunication(const std::string& packetType, uint32_t packetSize);
    void SetTotalVehicles(uint32_t totalVehicles);
    void SetTotalAttackers(uint32_t totalAttackers);
    uint32_t m_totalVehicles;          // To store the total number of vehicles
    uint32_t m_totalAttackers;         // To store the total number of attackers
    bool IsVehicleActuallyAttacker(const std::string& vehicleId) const;
    std::string GenerateTransactionId(const Transaction& tx) const;
    std::string TransactionTypeToString(TransactionType type) const;
    uint32_t EstimateBlockSize(const VanetBlock& block) const;
    // NEW: Adaptive batch processing methods
    void ProposeBlockForBatch();
    void UpdateAdaptiveNetworkParameters(); // NEW: Update network parameters for adaptive processing
    void SetVehicleCurrentEventIndex(const std::string& vehicleId, uint32_t eventIndex);
    std::map<std::string, bool> m_vehicleBaseAttackerStatus; 
    std::map<std::string, bool> m_vehicleCurrentBehavior; 
    void CalculateCurrentTPS();
    void SchedulePeriodicTpsCalculation();
    double CalculateAverageReputation() const;
    double CalculateAverageAttackerReputation() const;
    double CalculateAverageHonestReputation() const;
    // --- Metrics and Registration Management ---
    Ptr<MetricsCollector> m_metricsCollector;
    const std::vector<double>& GetBlockProcessingTimes() const;
    void SetParameterOverrides(const std::map<std::string, double>& overrides);
private:
    void SendData(std::shared_ptr<ndn::Data> data);

    // --- Blockchain Data ---
    std::list<VanetBlock> m_localBlockchain;
    std::vector<Transaction> m_transactionPool;
    std::unordered_map<std::string, std::string> m_vehicleKeys;
    std::unordered_map<std::string, double> m_vehicleReputations;

    // --- Node Info ---
    std::string m_nodeName;
    NodeType m_nodeType = VEHICLE_CLIENT;
    std::vector<std::string> m_rsuList;
    int m_n_rsus = 0;
    int m_f_rsus = 0;

    // --- PBFT State ---
    enum PbftPhase { IDLE, PRE_PREPARE_SENT, PRE_PREPARE_RECEIVED, PREPARE_SENT, COMMIT_SENT, COMMITTED };
    struct PbftBlockState {
        PbftPhase phase = IDLE;
        VanetBlock block;
        uint64_t view = 0;
        uint64_t seqNum = 0;
        std::string proposerId = "";
        std::unordered_map<std::string, std::string> prepareVotes;
        std::unordered_map<std::string, std::string> commitVotes;
    };
    std::unordered_map<std::string, PbftBlockState> m_pbftActiveConsensus;
    std::unordered_map<std::string, VanetBlock> m_pendingPbftBlocks;
    std::unordered_map<std::string, std::string> m_pendingBlockRequests;
    uint64_t m_pbftCurrentView;
    uint64_t m_pbftCurrentSeqNum;
    uint32_t m_pbftCurrentProposerIndex;
    uint64_t m_lastProposedHeight;

    // --- PBFT Handlers ---
    void HandlePrePrepare(std::shared_ptr<const ndn::Interest> interest);
    void HandlePrepare(std::shared_ptr<const ndn::Interest> interest);
    void HandleCommit(std::shared_ptr<const ndn::Interest> interest);
    void StartPbft(const VanetBlock& newBlock);
    void BroadcastPrePrepare(const VanetBlock& block);
    void BroadcastPrepare(const std::string& blockHash);
    void BroadcastCommit(const std::string& blockHash);
    bool ValidateBlock(const VanetBlock& block);
    void AddBlockToChain(const VanetBlock& block);
    void ProcessReceivedBlockForPbft(const VanetBlock& block, const std::string& originalProposerId);
    void PrintStatus();
    std::string DetermineProposer(uint32_t blockHeight);
    void BroadcastTransaction(const Transaction& tx);
    void HandleTransactionBroadcast(std::shared_ptr<const ndn::Interest> interest);

    // --- NDN Handlers ---
    void HandleKeyRequest(std::shared_ptr<const ndn::Interest> interest, const std::string& vehicleId);
    void HandleReputationRequest(std::shared_ptr<const ndn::Interest> interest, const std::string& vehicleId);
    void HandleBlockRequest(std::shared_ptr<const ndn::Interest> interest, const std::string& blockHash);
    void HandleRegistrationInterest(std::shared_ptr<const ndn::Interest> interest);
    void HandleEventReportInterest(std::shared_ptr<const ndn::Interest> interest);

    // --- Event Processing ---
    std::vector<EventReport> m_pendingReports;
    std::unordered_map<std::string, EventCluster> m_activeClusters;
    double m_distanceThreshold = 50.0;
    Time m_timeThreshold = Seconds(10.0);
    Time m_clusterCheckDelay = Seconds(5.0);

    void ProcessEventReport(const EventReport& report);
    std::string FindMatchingCluster(const EventReport& report);
    void ScheduleClusterCheck(const std::string& eventId);
    void CheckCluster(const std::string& eventId);
    double CalculateDistance(const Vector& pos1, const Vector& pos2);

    // --- Reputation System ---
    double m_thetaHigh = 0.70;
    double m_thetaLow = 0.40;
    uint32_t m_nMin = 2;
    double m_alpha = 0.1;
    double m_beta = 0.2;

    void CalculateEventCredibility(EventCluster& cluster);
    void UpdateReputations(const EventCluster& cluster, const std::string& verdict);
    // --- Utility ---
    std::string SignString(const std::string& data);
    bool VerifyVehicleSignature(const EventReport& report);
    VanetBlock CreateCandidateBlock();
    void ForwardTransactionToLeader(const Transaction& tx);
    void HandleForwardedTransaction(std::shared_ptr<const ndn::Interest> interest);
    void SendTransactionForward(const Transaction& tx, const std::string& targetRsu);


    struct PendingRegistration {
        std::string vehicleId;
        std::string publicKey;
        ndn::Name originalInterestName;
        Time requestTime;        // Add this field
        std::string requestingRsu;
        std::string ackTarget;
    };

    std::map<std::string, PendingRegistration> m_pendingRegistrations;
    std::map<std::string, std::string> m_vehicleAckTargets;

    void SendDeferredRegistrationAck(const std::string& vehicleId);
    bool IsVehicleRegistered(const std::string& vehicleId) const;
    void BroadcastTransactionWithAckInfo(const Transaction& tx, const std::string& ackTarget);
    void HandleTransactionWithAckBroadcast(std::shared_ptr<const ndn::Interest> interest);
    void SendGenericRegistrationAck(const std::string& vehicleId);


    // --- Performance Tracking ---
    uint32_t m_totalEventReportsProcessed;
    uint32_t m_totalRegistrationsProcessed;
    Time m_lastBlockProposalTime;

    CommunicationDataMetrics m_communicationData; // To store communication metrics
    // --- ADAPTIVE LEARNING AND BATCH PROCESSING ---
    AdaptiveLearningRateCalculator m_adaptiveLearningCalculator;
    std::unordered_map<std::string, uint32_t> m_vehicleTotalReports;
    std::unordered_map<std::string, uint32_t> m_vehicleCorrectReports;
    AdaptiveBatchManager m_adaptiveBatchManager; 

    // --- ADAPTIVE BATCH PROCESSING METHODS ---
    void SendTransactionBatch(const std::vector<Transaction>& batch);
    std::vector<Transaction> ParseTransactionBatch(const std::string& batchData);
    std::vector<Transaction> ParseAdaptiveBatch(const std::string& batchData);
    Time m_lastQueryTime;
    std::map<std::string, uint32_t> m_currentEventIndex;         // Track current event for each vehicle
    // NEW: Batch acknowledgment tracking
    std::map<std::string, BatchAckTracker> m_pendingBatchAcks;
    std::string ClassifyDetectionResult(bool wasActuallyMalicious, bool rsuAccepted);
    std::map<std::string, uint32_t> m_vehicleCurrentEventIndex;
    std::map<std::string, uint32_t> m_vehicleEventParticipation;
    std::map<std::string, Time> m_vehicleLastActivity;
    bool DetermineIfReportWasMaliciousOracleFree(const EventReport& report, const std::string& groundTruth);
    bool CalculateRsuAcceptanceDecisionOracleFree(const EventReport& report, double reputation, 
                                                                 bool contentMatches, double suspicionConfidence);
    
    std::string GenerateUpdateContextOracleFree(const EventReport& report, const std::string& inferredPattern,
                                                               bool wasActuallyMalicious, bool reportMatchesGroundTruth, 
                                                               const std::string& verdict);
    bool IsVehicleSuspiciousForEventOracleFree(const std::string& vehicleId, uint32_t eventIndex);
    void Reset();
    bool IsVehicleCurrentlyActingMalicious(const std::string& vehicleId) const;
    Time m_lastTpsCalculationTime;
    uint32_t m_transactionsSinceLastTps;
    double m_currentTps;
    std::vector<double> m_tpsHistory;
    uint32_t m_totalBlocksProposed;
    Time m_simulationStartTime;
    uint32_t m_totalTransactionsProcessed; 

    void HandleLocationQueryInterest(std::shared_ptr<const ndn::Interest> interest);
    std::string SearchLocationEvents(const std::string& queryLocation) const;
    std::string FormatLocationQueryResponse(const std::vector<Transaction>& relevantEvents, 
                                          const std::string& queryLocation) const;
    bool IsLocationNearby(const std::string& loc1, const std::string& loc2, double toleranceMeters) const;
    std::vector<double> m_blockProcessingTimes;
    std::map<std::string, Time> m_blockCreationTimes; 
    void UpdateReputations(const EventCluster& cluster, 
                                           const std::string& verdict, 
                                           const std::string& groundTruth);
    double CalculateSuspicionScore(const std::string& vehicleId);
    double GetParameterOrDefault(const std::string& paramName, double defaultValue) const;
    void UpdateLocalHistory(const std::string& vehicleId, bool isCorrect);
    std::map<std::string, double> m_parameterOverrides;
  // Store outcome history: 1 for correct, 0 for incorrect
    std::map<std::string, std::deque<int>> m_vehicleReportHistory; 

    // Constants from the paper
    const int HISTORY_WINDOW_N = 20;
    const double WEIGHT_RPF = 0.6;
    const double WEIGHT_VOL = 0.4;
    const double DECAY_GAMMA = 0.90;
    const double CONFIDENCE_THRESHOLD_TAU = 0.70;
};

}

#endif // VANET_BLOCKCHAIN_APP_HPP