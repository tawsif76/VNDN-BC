#ifndef METRICS_COLLECTOR_HPP
#define METRICS_COLLECTOR_HPP

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace ns3 {

// Enhanced structures for event-centric detection
struct EventDetectionRecord {
    std::string eventId;
    std::string vehicleId;
    Time detectionTime;
    
    // Ground Truth (what actually happened)
    bool wasActuallyMaliciousReport;    // Was this report actually fake?
    std::string actualEventType;       // What really happened
    std::string reportedEventType;     // What vehicle claimed happened
    
    // System's Verdict (what RSU decided)
    std::string rsuVerdict;            // "True", "False", or "Uncertain"
    bool rsuAcceptedReport;            // Did RSU accept this specific report?
    double eventCredibility;           // Credibility score for this event
    double reporterReputation;         // Reporter's reputation at time of report
    
    // Classification (calculated automatically)
    std::string detectionClass;       // "TP", "FP", "TN", "FN", or "UNCERTAIN"
    
    // Additional context
    std::string attackerPattern;      // Pattern if vehicle is attacker
    bool isAdaptiveAttacker;          // Is this an adaptive attacker?
    uint32_t eventIndex;              // Which event in the sequence
};

// Data structures for other types of metrics (keeping existing ones)
struct AccessDelayRecord {
    std::string vehicleId;
    Time requestTime;
    Time responseTime;
    Time delay;
    std::string requestType;
    bool successful;
    std::string targetRsu;
};

struct CommunicationOverheadRecord {
    Time timestamp;
    std::string packetType;
    uint32_t packetSize;
    std::string source;
    std::string destination;
    bool isControlMessage;
};

struct ReputationRecord {
    std::string vehicleId;
    Time timestamp;
    double reputation;
    bool isKnownAttacker;
    std::string updateReason;
    std::string eventId;
};

struct TransactionRecord {
    std::string transactionId;
    std::string transactionType;
    Time submissionTime;
    Time processedTime;
    Time blockInclusionTime;
    bool successful;
    uint32_t blockHeight;
    std::string proposerRsu;
    uint32_t batchSize;
    Time latency;
};

struct ThroughputMeasurement {
    Time windowStart;
    Time windowEnd;
    uint32_t transactionsProcessed;
    uint32_t totalVehicles;
    double tps;
};

struct PerformanceSnapshot {
    Time timestamp;
    uint32_t totalVehicles;
    uint32_t totalAttackers;
    uint32_t registeredVehicles;
    uint32_t blockchainHeight;
    uint32_t pendingTransactions;
    double averageReputation;
    double averageAttackerReputation;
    double averageHonestReputation;
};

struct CommunicationAnalysisResult {
    uint64_t totalBytes = 0;
    uint64_t controlBytes = 0;
    uint64_t applicationBytes = 0;
    uint32_t controlMessages = 0;
    uint32_t applicationMessages = 0;
    double overheadRatio = 0.0;
};

// Event-centric detection metrics structure
struct EventDetectionMetrics {
    uint32_t totalReports = 0;
    uint32_t maliciousReports = 0;
    uint32_t honestReports = 0;
    uint32_t uncertainReports = 0;
    
    // Confusion Matrix
    uint32_t truePositives = 0;    // Fake reports correctly identified as False
    uint32_t falseNegatives = 0;   // Fake reports incorrectly accepted as True
    uint32_t trueNegatives = 0;    // Real reports correctly accepted as True
    uint32_t falsePositives = 0;   // Real reports incorrectly flagged as False
    
    // Calculated Metrics
    double detectionRate = 0.0;     // TP / (TP + FN) - Recall
    double falsePositiveRate = 0.0; // FP / (FP + TN)
    double falseNegativeRate = 0.0; // FN / (FN + TP)
    double precision = 0.0;         // TP / (TP + FP)
    double accuracy = 0.0;          // (TP + TN) / (TP + FP + TN + FN)
    double f1Score = 0.0;          // 2 * (Precision * Recall) / (Precision + Recall)
};

class MetricsCollector : public Object {
public:
    static TypeId GetTypeId();
    MetricsCollector();
    virtual ~MetricsCollector();

    // Simulation context setup
    void SetSimulationContext(uint32_t totalVehicles, uint32_t totalAttackers, Time startTime);
    
    // NEW: Event-centric detection recording
    void RecordEventDetection(const std::string& eventId, const std::string& vehicleId,
                             bool wasActuallyMaliciousReport, const std::string& actualEventType,
                             const std::string& reportedEventType, const std::string& rsuVerdict,
                             double eventCredibility, double reporterReputation,
                             const std::string& attackerPattern = "", bool isAdaptiveAttacker = false,
                             uint32_t eventIndex = 0);
    
    // Existing data collection methods
    void RecordAccessDelay(const std::string& vehicleId, Time requestTime, Time responseTime, 
                          const std::string& requestType, bool successful, const std::string& targetRsu = "");
    void RecordCommunicationOverhead(const std::string& packetType, uint32_t packetSize, 
                                   const std::string& source, const std::string& destination, 
                                   bool isControlMessage = true);
    void RecordReputationUpdate(const std::string& vehicleId, double newReputation, 
                               bool isKnownAttacker, const std::string& updateReason, 
                               const std::string& eventId = "");
    void RecordTransaction(const std::string& transactionId, const std::string& transactionType,
                          Time submissionTime, Time processedTime, Time blockInclusionTime,
                          bool successful, uint32_t blockHeight, const std::string& proposerRsu,
                          uint32_t batchSize = 1);
    void RecordPerformanceSnapshot(uint32_t registeredVehicles, uint32_t blockchainHeight,
                                  uint32_t pendingTransactions, double averageReputation,
                                  double averageAttackerReputation, double averageHonestReputation);

    // NEW: Event-centric analysis methods
    EventDetectionMetrics CalculateEventDetectionMetrics() const;
    EventDetectionMetrics CalculateEventDetectionMetricsByPattern(const std::string& pattern) const;
    std::map<std::string, EventDetectionMetrics> CalculateEventDetectionMetricsByAllPatterns() const;
    
    // Existing analysis methods
    double CalculateAverageAccessDelay(uint32_t networkSize) const;
    double CalculateCommunicationOverhead(Time windowStart, Time windowEnd) const;
    double CalculateTransactionThroughput(Time windowStart, Time windowEnd) const;
    std::vector<ThroughputMeasurement> CalculateThroughputOverTime(Time windowSize = Seconds(120)) const;

    // DEPRECATED: Legacy detection methods (keep for compatibility but mark as deprecated)
    double CalculateDetectionRate() const __attribute__((deprecated("Use CalculateEventDetectionMetrics() instead")));
    double CalculateFalsePositiveRate() const __attribute__((deprecated("Use CalculateEventDetectionMetrics() instead")));
    double CalculateFalseNegativeRate() const __attribute__((deprecated("Use CalculateEventDetectionMetrics() instead")));

    // Export methods
    void ExportResults(const std::string& baseFilename, uint32_t runId, double attackerPercentage);
    void ExportDetailedResults(const std::string& baseFilename, uint32_t runId, double attackerPercentage);
    void ExportSuperiorityMetrics(const std::string& baseFilename, uint32_t runId, 
                                 double attackerPercentage, uint32_t networkSize);
    void ExportEventDetectionAnalysis(const std::string& outputFile) const;
    void ExportAdaptiveAttackerAnalysis(const std::string& outputFile) const;
    
    // Periodic reporting
    void SchedulePeriodicReporting(Time interval);
    void GeneratePeriodicReport();
    
    // Ground truth management
    void SetVehicleGroundTruth(const std::string& vehicleId, bool isAttacker, const std::string& pattern);
    void SetVehicleCurrentBehavior(const std::string& vehicleId, bool willActMalicious, uint32_t eventIndex);
    bool GetVehicleGroundTruth(const std::string& vehicleId) const;
    bool GetVehicleCurrentBehavior(const std::string& vehicleId, uint32_t eventIndex) const;
    void GenerateDetectionPerformanceReport();    // Reset for new simulation run
    void Reset();
    double CalculateRealTimeThroughput(Time windowStart, Time windowEnd) const;
    CommunicationAnalysisResult CalculateDetailedCommunicationMetrics() const;
    void SetBlockProcessingTimes(const std::vector<double>& processingTimes);

private:
    // Simulation context
    uint32_t m_totalVehicles;
    uint32_t m_totalAttackers;
    Time m_simulationStartTime;
    Time m_lastReportTime;
    
    // NEW: Event-centric detection data storage
    std::vector<EventDetectionRecord> m_eventDetectionRecords;
    
    // Existing data storage
    std::vector<AccessDelayRecord> m_accessDelayRecords;
    std::vector<CommunicationOverheadRecord> m_communicationRecords;
    std::vector<ReputationRecord> m_reputationRecords;
    std::vector<TransactionRecord> m_transactionRecords;
    std::vector<PerformanceSnapshot> m_performanceSnapshots;
    
    // Ground truth tracking
    std::map<std::string, bool> m_vehicleGroundTruth;
    std::map<std::string, std::string> m_vehicleAttackerPatterns;

    
    // Performance tracking
    uint32_t m_totalTransactionsProcessed;
    Time m_mxTime;
    uint32_t m_totalSuccessfulQueries;
    uint32_t m_totalFailedQueries;
    uint64_t m_totalCommunicationBytes;
    uint32_t m_totalControlMessages;
    
    // NEW: Helper methods for event-centric detection
    std::string ClassifyDetection(bool wasActuallyMalicious, bool rsuAcceptedReport, 
                                 const std::string& rsuVerdict) const;
    void UpdateConfusionMatrix(const EventDetectionRecord& record, EventDetectionMetrics& metrics) const;
    void CalculateEventMetricsFromMatrix(EventDetectionMetrics& metrics) const;
    
    // Existing helper methods
    std::vector<AccessDelayRecord> FilterAccessDelaysByNetworkSize(uint32_t networkSize) const;
    std::vector<CommunicationOverheadRecord> FilterCommunicationByTimeWindow(Time start, Time end) const;
    
    // File writing helpers
    void WriteCSVHeader(std::ofstream& file, const std::vector<std::string>& headers) const;
    void WriteCSVRow(std::ofstream& file, const std::vector<std::string>& values) const;
    std::string TimeToString(Time t) const;
    std::string DoubleToString(double value, int precision = 3) const;
    std::map<std::pair<std::string, uint32_t>, bool> m_vehicleBehaviorPerEvent;

    // Event scheduling
    EventId m_periodicReportEvent;
    std::vector<double> m_blockProcessingTimes;
};

} // namespace ns3

#endif // METRICS_COLLECTOR_HPP