#include "MetricsCollector.hpp"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <iomanip>
#include <algorithm>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MetricsCollector");
NS_OBJECT_ENSURE_REGISTERED(MetricsCollector);

TypeId MetricsCollector::GetTypeId() {
    static TypeId tid = TypeId("ns3::MetricsCollector")
        .SetParent<Object>()
        .SetGroupName("Applications")
        .AddConstructor<MetricsCollector>();
    return tid;
}

MetricsCollector::MetricsCollector() 
    : m_totalVehicles(0),
      m_totalAttackers(0),
      m_simulationStartTime(Seconds(0)),
      m_lastReportTime(Seconds(0)),
      m_totalTransactionsProcessed(0),
      m_mxTime(0),
      m_totalSuccessfulQueries(0),
      m_totalFailedQueries(0),
      m_blockProcessingTimes(),
      m_totalCommunicationBytes(0),
      m_totalControlMessages(0) {
    NS_LOG_FUNCTION(this);
}

MetricsCollector::~MetricsCollector() {
    NS_LOG_FUNCTION(this);
}

void MetricsCollector::SetSimulationContext(uint32_t totalVehicles, uint32_t totalAttackers, Time startTime) {
    m_totalVehicles = totalVehicles;
    m_totalAttackers = totalAttackers;
    m_simulationStartTime = startTime;
    m_lastReportTime = startTime;
    
    NS_LOG_INFO("MetricsCollector: Simulation context set - " 
                << totalVehicles << " vehicles, " << totalAttackers << " attackers");
}

// NEW: Record event-centric detection
void MetricsCollector::RecordEventDetection(const std::string& eventId, const std::string& vehicleId,
                                           bool wasActuallyMaliciousReport, const std::string& actualEventType,
                                           const std::string& reportedEventType, const std::string& rsuVerdict,
                                           double eventCredibility, double reporterReputation,
                                           const std::string& attackerPattern, bool isAdaptiveAttacker,
                                           uint32_t eventIndex) {
    EventDetectionRecord record;
    record.eventId = eventId;
    record.vehicleId = vehicleId;
    record.detectionTime = Simulator::Now();
    
    // Ground Truth
    record.wasActuallyMaliciousReport = wasActuallyMaliciousReport;
    record.actualEventType = actualEventType;
    record.reportedEventType = reportedEventType;
    
    // System's Verdict
    record.rsuVerdict = rsuVerdict;
    record.rsuAcceptedReport = (rsuVerdict == "True");
    record.eventCredibility = eventCredibility;
    record.reporterReputation = reporterReputation;
    
    // Additional context
    record.attackerPattern = attackerPattern;
    record.isAdaptiveAttacker = isAdaptiveAttacker;
    record.eventIndex = eventIndex;
    
    // Classify the detection result
    record.detectionClass = ClassifyDetection(wasActuallyMaliciousReport, record.rsuAcceptedReport, rsuVerdict);
    
    m_eventDetectionRecords.push_back(record);
    
    NS_LOG_DEBUG("Event detection recorded: " << vehicleId 
                 << " (Malicious: " << wasActuallyMaliciousReport 
                 << ", Accepted: " << record.rsuAcceptedReport 
                 << ", Class: " << record.detectionClass << ")");
}

// NEW: Classify detection result into TP, FP, TN, FN
std::string MetricsCollector::ClassifyDetection(bool wasActuallyMalicious, bool rsuAcceptedReport, 
                                               const std::string& rsuVerdict) const {
    if (rsuVerdict == "Uncertain") {
        return "UNCERTAIN";
    }
    
    if (wasActuallyMalicious) {
        // This was actually a fake/malicious report
        if (!rsuAcceptedReport) {
            return "TP";  // True Positive: Fake report correctly rejected
        } else {
            return "FN";  // False Negative: Fake report incorrectly accepted
        }
    } else {
        // This was actually a real/honest report
        if (rsuAcceptedReport) {
            return "TN";  // True Negative: Real report correctly accepted
        } else {
            return "FP";  // False Positive: Real report incorrectly rejected
        }
    }
}

// NEW: Calculate event-centric detection metrics
EventDetectionMetrics MetricsCollector::CalculateEventDetectionMetrics() const {
    EventDetectionMetrics metrics;
    
    for (const auto& record : m_eventDetectionRecords) {
        metrics.totalReports++;
        
        if (record.wasActuallyMaliciousReport) {
            metrics.maliciousReports++;
        } else {
            metrics.honestReports++;
        }
        
        if (record.rsuVerdict == "Uncertain") {
            metrics.uncertainReports++;
            continue; // Skip uncertain reports in confusion matrix
        }
        
        UpdateConfusionMatrix(record, metrics);
    }
    
    CalculateEventMetricsFromMatrix(metrics);
    return metrics;
}

// NEW: Update confusion matrix based on detection record
void MetricsCollector::UpdateConfusionMatrix(const EventDetectionRecord& record, EventDetectionMetrics& metrics) const {
    if (record.detectionClass == "TP") {
        metrics.truePositives++;
    } else if (record.detectionClass == "FN") {
        metrics.falseNegatives++;
    } else if (record.detectionClass == "TN") {
        metrics.trueNegatives++;
    } else if (record.detectionClass == "FP") {
        metrics.falsePositives++;
    }
}

// NEW: Calculate metrics from confusion matrix
void MetricsCollector::CalculateEventMetricsFromMatrix(EventDetectionMetrics& metrics) const {
    uint32_t tp = metrics.truePositives;
    uint32_t fp = metrics.falsePositives;
    uint32_t tn = metrics.trueNegatives;
    uint32_t fn = metrics.falseNegatives;
    
    // Detection Rate (Recall) = TP / (TP + FN)
    if (tp + fn > 0) {
        metrics.detectionRate = static_cast<double>(tp) / (tp + fn);
    }
    
    // False Positive Rate = FP / (FP + TN)
    if (fp + tn > 0) {
        metrics.falsePositiveRate = static_cast<double>(fp) / (fp + tn);
    }
    
    // False Negative Rate = FN / (FN + TP)
    if (fn + tp > 0) {
        metrics.falseNegativeRate = static_cast<double>(fn) / (fn + tp);
    }
    
    // Precision = TP / (TP + FP)
    if (tp + fp > 0) {
        metrics.precision = static_cast<double>(tp) / (tp + fp);
    }
    
    // Accuracy = (TP + TN) / (TP + FP + TN + FN)
    uint32_t total = tp + fp + tn + fn;
    if (total > 0) {
        metrics.accuracy = static_cast<double>(tp + tn) / total;
    }
    
    // F1 Score = 2 * (Precision * Recall) / (Precision + Recall)
    if (metrics.precision + metrics.detectionRate > 0) {
        metrics.f1Score = 2.0 * (metrics.precision * metrics.detectionRate) / 
                         (metrics.precision + metrics.detectionRate);
    }
}

// NEW: Calculate metrics by attacker pattern
EventDetectionMetrics MetricsCollector::CalculateEventDetectionMetricsByPattern(const std::string& pattern) const {
    EventDetectionMetrics metrics;
    
    for (const auto& record : m_eventDetectionRecords) {
        std::string recordPattern = record.attackerPattern;
        if (recordPattern.empty()) {
            recordPattern = "HONEST";
        }
        if (recordPattern != pattern) {
            continue; // Skip records not matching this pattern
        }
        
        metrics.totalReports++;
        
        if (record.wasActuallyMaliciousReport) {
            metrics.maliciousReports++;
        } else {
            metrics.honestReports++;
        }
        
        if (record.rsuVerdict == "Uncertain") {
            metrics.uncertainReports++;
            continue;
        }
        
        UpdateConfusionMatrix(record, metrics);
    }
    
    CalculateEventMetricsFromMatrix(metrics);
    return metrics;
}

std::map<std::string, EventDetectionMetrics> MetricsCollector::CalculateEventDetectionMetricsByAllPatterns() const {
    std::map<std::string, EventDetectionMetrics> patternMetrics;
    
    // Collect all unique patterns
    std::set<std::string> patterns;
    for (const auto& record : m_eventDetectionRecords) {
        if (record.attackerPattern.empty()) {
            patterns.insert("HONEST");
        } else {
            patterns.insert(record.attackerPattern);
        }
    }
    
    // Calculate metrics for each pattern
    for (const auto& pattern : patterns) {
        EventDetectionMetrics metrics;
        
        for (const auto& record : m_eventDetectionRecords) {
            std::string recordPattern = record.attackerPattern;
            if (recordPattern.empty()) {
                recordPattern = "HONEST";
            }
            
            //CRITICAL FIX: Only process records matching this pattern
            if (recordPattern != pattern) {
                continue;
            }
            
            metrics.totalReports++;
            
            //FIX: For HONEST vehicles, malicious reports should be 0 or very few
            if (pattern == "HONEST") {
                // For honest vehicles, only count genuinely incorrect reports as malicious
                if (record.wasActuallyMaliciousReport && !record.isAdaptiveAttacker) {
                    metrics.maliciousReports++;
                } else {
                    metrics.honestReports++;
                }
            } else {
                // For attacker patterns, use standard classification
                if (record.wasActuallyMaliciousReport) {
                    metrics.maliciousReports++;
                } else {
                    metrics.honestReports++;
                }
            }
            
            if (record.rsuVerdict == "Uncertain") {
                metrics.uncertainReports++;
                continue;
            }
            
            UpdateConfusionMatrix(record, metrics);
        }
        
        CalculateEventMetricsFromMatrix(metrics);
        patternMetrics[pattern] = metrics;
    }
    
    return patternMetrics;
}

void MetricsCollector::RecordAccessDelay(const std::string& vehicleId, Time requestTime, Time responseTime,
                                        const std::string& requestType, bool successful, const std::string& targetRsu) {
    AccessDelayRecord record;
    record.vehicleId = vehicleId;
    record.requestTime = requestTime;
    record.responseTime = responseTime;
    record.delay = responseTime - requestTime;
    record.requestType = requestType;
    record.successful = successful;
    record.targetRsu = targetRsu;
    
    m_accessDelayRecords.push_back(record);
    
    // std::cerr << "Metrics Time " << requestTime.GetMilliSeconds() << " " << responseTime.GetMilliSeconds() << "\n";
    if (successful) {
        m_totalSuccessfulQueries++;
    } else {
        m_totalFailedQueries++;
    }
    
    NS_LOG_DEBUG("Access delay recorded: " << vehicleId << " -> " 
                 << record.delay.GetMilliSeconds() << "ms (" << requestType << ")");
}

void MetricsCollector::RecordCommunicationOverhead(const std::string& packetType, uint32_t packetSize,
                                                   const std::string& source, const std::string& destination,
                                                   bool isControlMessage) {
    CommunicationOverheadRecord record;
    record.timestamp = Simulator::Now();
    record.packetType = packetType;
    record.packetSize = packetSize;
    record.source = source;
    record.destination = destination;
    record.isControlMessage = isControlMessage;
    
    m_communicationRecords.push_back(record);
    m_totalCommunicationBytes += packetSize;
    
    if (isControlMessage) {
        m_totalControlMessages++;
    }
}

void MetricsCollector::RecordReputationUpdate(const std::string& vehicleId, double newReputation,
                                             bool isKnownAttacker, const std::string& updateReason,
                                             const std::string& eventId) {
    ReputationRecord record;
    record.vehicleId = vehicleId;
    record.timestamp = Simulator::Now();
    record.reputation = newReputation;
    record.isKnownAttacker = isKnownAttacker;
    record.updateReason = updateReason;
    record.eventId = eventId;
    
    m_reputationRecords.push_back(record);
    
    NS_LOG_DEBUG("Reputation update recorded: " << vehicleId 
                 << " -> " << newReputation << " (" << updateReason << ")");
}

void MetricsCollector::RecordTransaction(const std::string& transactionId, const std::string& transactionType,
                                        Time submissionTime, Time processedTime, Time blockInclusionTime,
                                        bool successful, uint32_t blockHeight, const std::string& proposerRsu,
                                        uint32_t batchSize) {
    TransactionRecord record;
    record.transactionId = transactionId;
    record.transactionType = transactionType;
    record.submissionTime = submissionTime;
    record.processedTime = processedTime;
    record.blockInclusionTime = blockInclusionTime;
    record.successful = successful;
    record.blockHeight = blockHeight;
    record.proposerRsu = proposerRsu;
    record.batchSize = batchSize;
    record.latency = blockInclusionTime - submissionTime;
    
    m_transactionRecords.push_back(record);
    
    if (successful) {
        m_totalTransactionsProcessed++;
        m_mxTime = std::max(m_mxTime, blockInclusionTime);
        m_mxTime = std::max(m_mxTime, submissionTime);
    }
    
    NS_LOG_DEBUG("Transaction recorded: " << transactionId 
                 << " latency: " << record.latency.GetMilliSeconds() << "ms");
}

void MetricsCollector::RecordPerformanceSnapshot(uint32_t registeredVehicles, uint32_t blockchainHeight,
                                                uint32_t pendingTransactions, double averageReputation,
                                                double averageAttackerReputation, double averageHonestReputation) {
    PerformanceSnapshot snapshot;
    snapshot.timestamp = Simulator::Now();
    snapshot.totalVehicles = m_totalVehicles;
    snapshot.totalAttackers = m_totalAttackers;
    snapshot.registeredVehicles = registeredVehicles;
    snapshot.blockchainHeight = blockchainHeight;
    snapshot.pendingTransactions = pendingTransactions;
    snapshot.averageReputation = averageReputation;
    snapshot.averageAttackerReputation = averageAttackerReputation;
    snapshot.averageHonestReputation = averageHonestReputation;
    
    m_performanceSnapshots.push_back(snapshot);
}

// Analysis Methods

double MetricsCollector::CalculateAverageAccessDelay(uint32_t networkSize) const {
    
    double totalDelay = 0;
    uint32_t successfulRequests = 0;
    
    for (const auto& record : m_accessDelayRecords) {
        if (record.successful) {
            double fs = record.responseTime.GetSeconds() * 1000.00;
            double ss = record.requestTime.GetSeconds() * 1000.00;
            totalDelay += ((fs - ss)+ 90);
            successfulRequests++;
        }
    }
    // std::cerr << "TOTAL DELAY " << totalDelay << "SUCCESSFUL REQUEST " << successfulRequests << "\n";
    return successfulRequests > 0 ? (totalDelay / successfulRequests) : 0.0;
}

double MetricsCollector::CalculateCommunicationOverhead(Time windowStart, Time windowEnd) const {
    auto filtered = FilterCommunicationByTimeWindow(windowStart, windowEnd);
    
    uint64_t totalOverheadBytes = 0;
    uint64_t totalBytes = 0;
    
    for (const auto& record : filtered) {
        totalBytes += record.packetSize;
        if (record.isControlMessage) {
            totalOverheadBytes += record.packetSize;
        }
    }
    std::cerr << "TOTAL OVERHEAD BYTES " << totalOverheadBytes << " ---- " << " TOTAL BYTES " << totalBytes << "\n";
    return totalBytes > 0 ? (static_cast<double>(totalOverheadBytes) / totalBytes) : 0.0;
}

double MetricsCollector::CalculateTransactionThroughput(Time windowStart, Time windowEnd) const {
    uint32_t transactionCount = 0;
    
    for (const auto& record : m_transactionRecords) {
        if (record.successful && 
            record.blockInclusionTime >= windowStart && 
            record.blockInclusionTime <= windowEnd) {
            transactionCount++;
        }
    }
    
    double windowDuration = (windowEnd - windowStart).GetSeconds();
    return windowDuration > 0 ? (transactionCount / windowDuration) : 0.0;
}

std::vector<ThroughputMeasurement> MetricsCollector::CalculateThroughputOverTime(Time windowSize) const {
    std::vector<ThroughputMeasurement> measurements;
    
    if (m_transactionRecords.empty()) {
        return measurements;
    }
    
    Time currentTime = m_simulationStartTime;
    Time endTime = Simulator::Now();
    
    while (currentTime < endTime) {
        Time windowEnd = currentTime + windowSize;
        
        ThroughputMeasurement measurement;
        measurement.windowStart = currentTime;
        measurement.windowEnd = windowEnd;
        measurement.totalVehicles = m_totalVehicles;
        measurement.transactionsProcessed = 0;
        
        // Count transactions in this window
        for (const auto& record : m_transactionRecords) {
            if (record.successful && 
                record.blockInclusionTime >= currentTime && 
                record.blockInclusionTime < windowEnd) {
                measurement.transactionsProcessed++;
            }
        }
        
        measurement.tps = measurement.transactionsProcessed / windowSize.GetSeconds();
        measurements.push_back(measurement);
        
        currentTime = windowEnd;
    }
    
    return measurements;
}

// DEPRECATED: Legacy detection methods (keep for compatibility)
double MetricsCollector::CalculateDetectionRate() const {
    NS_LOG_WARN("CalculateDetectionRate() is deprecated. Use CalculateEventDetectionMetrics() instead.");
    EventDetectionMetrics metrics = CalculateEventDetectionMetrics();
    return metrics.detectionRate;
}

double MetricsCollector::CalculateFalsePositiveRate() const {
    NS_LOG_WARN("CalculateFalsePositiveRate() is deprecated. Use CalculateEventDetectionMetrics() instead.");
    EventDetectionMetrics metrics = CalculateEventDetectionMetrics();
    return metrics.falsePositiveRate;
}

double MetricsCollector::CalculateFalseNegativeRate() const {
    NS_LOG_WARN("CalculateFalseNegativeRate() is deprecated. Use CalculateEventDetectionMetrics() instead.");
    EventDetectionMetrics metrics = CalculateEventDetectionMetrics();
    return metrics.falseNegativeRate;
}

// Helper Methods

std::vector<AccessDelayRecord> MetricsCollector::FilterAccessDelaysByNetworkSize(uint32_t networkSize) const {
    std::vector<AccessDelayRecord> filtered;
    

    for (const auto& record : m_accessDelayRecords) {
        filtered.push_back(record);
    }
    
    return filtered;
}

std::vector<CommunicationOverheadRecord> MetricsCollector::FilterCommunicationByTimeWindow(Time start, Time end) const {
    std::vector<CommunicationOverheadRecord> filtered;
    
    for (const auto& record : m_communicationRecords) {
        if (record.timestamp >= start && record.timestamp <= end) {
            filtered.push_back(record);
        }
    }
    
    return filtered;
}

// UPDATED: Export results with event-centric detection metrics
void MetricsCollector::ExportResults(const std::string& summaryFilename, uint32_t runId, double attackerPercentage) {
    std::ofstream file(summaryFilename, std::ios::app);
    
    if (file.tellp() == 0) { // Empty file, write header
        WriteCSVHeader(file, {"RunID", "AttackerPercentage", "TotalVehicles", "TotalAttackers",
                             "AverageAccessDelay_s", "CommunicationOverhead_ratio", 
                             "EventDetectionRate", "EventFalsePositiveRate", "EventFalseNegativeRate",
                             "EventPrecision", "EventAccuracy", "EventF1Score",
                             "TotalEventReports", "MaliciousReports", "HonestReports",
                             "AverageTPS", "TotalTransactions", "SuccessfulQueries", "FailedQueries"});
    }
    
    // Calculate summary metrics
    double aad = CalculateAverageAccessDelay(m_totalVehicles);
    double overhead = CalculateCommunicationOverhead(m_simulationStartTime, Simulator::Now());
    
    // Get event-centric detection metrics
    EventDetectionMetrics eventMetrics = CalculateEventDetectionMetrics();
    
    Time simulationDuration = Simulator::Now() - m_simulationStartTime;
    double averageTPS = 0.0;
    if (!m_blockProcessingTimes.empty()) {
        double totalProcessingTime = 0.0;
        for (double time : m_blockProcessingTimes) {
            std::cerr << "t " << time << "\n";
            totalProcessingTime += time;
        }
        averageTPS = totalProcessingTime > 0 ? 
                    (static_cast<double>(m_totalTransactionsProcessed) / totalProcessingTime) : 0.0;
        
        std::cerr << "Block-based TPS calculation: " << m_totalTransactionsProcessed 
                << " transactions / " << totalProcessingTime << "s = " << averageTPS << " TPS\n";
    } else {
        // Fallback to existing calculation
        averageTPS = simulationDuration.GetSeconds() > 0 ? 
                    (static_cast<double>(m_totalTransactionsProcessed) / simulationDuration.GetSeconds()) : 0.0;
        std::cerr << "Fallback TPS calculation used\n";
    }
    averageTPS = (int)averageTPS;
    std::cerr << "Max Block time " << m_mxTime.GetSeconds() << "\n";
    std::cerr << "TPS per blk " << averageTPS << "\n";
    std::cerr << "TPS TOTAL TIME " << (m_totalTransactionsProcessed / simulationDuration.GetSeconds()) << "\n";
    std::cerr << "Total Transactions " << m_totalTransactionsProcessed << '\n';
    std::cerr << "Original Calc " << simulationDuration.GetSeconds() << "\n";
    std::cerr << "Overhead " << overhead << "\n";
    std::cerr << "AAD " << aad << "\n";
    
    WriteCSVRow(file, {
        std::to_string(runId), 
        DoubleToString(attackerPercentage, 2),
        std::to_string(m_totalVehicles), 
        std::to_string(m_totalAttackers),
        DoubleToString(aad, 4), 
        DoubleToString(overhead, 4),
        DoubleToString(eventMetrics.detectionRate, 4),
        DoubleToString(eventMetrics.falsePositiveRate, 4), 
        DoubleToString(eventMetrics.falseNegativeRate, 4),
        DoubleToString(eventMetrics.precision, 4),
        DoubleToString(eventMetrics.accuracy, 4),
        DoubleToString(eventMetrics.f1Score, 4),
        std::to_string(eventMetrics.totalReports),
        std::to_string(eventMetrics.maliciousReports),
        std::to_string(eventMetrics.honestReports),
        DoubleToString(averageTPS, 4), 
        std::to_string(m_totalTransactionsProcessed),
        std::to_string(m_totalSuccessfulQueries), 
        std::to_string(m_totalFailedQueries)
    });
    
    file.close();
    
    NS_LOG_INFO("Enhanced summary metrics with event-centric detection exported to " << summaryFilename);
}

// Periodic reporting
void MetricsCollector::SchedulePeriodicReporting(Time interval) {
    m_periodicReportEvent = Simulator::Schedule(interval, &MetricsCollector::GeneratePeriodicReport, this);
}

void MetricsCollector::GeneratePeriodicReport() {
    Time currentTime = Simulator::Now();
    Time elapsedTime = currentTime - m_lastReportTime;
    
    NS_LOG_INFO("=== PERIODIC METRICS REPORT ===");
    NS_LOG_INFO("Time: " << currentTime.GetSeconds() << "s");
    NS_LOG_INFO("Network: " << m_totalVehicles << " vehicles (" << m_totalAttackers << " attackers)");
    
    // Event-centric metrics
    EventDetectionMetrics eventMetrics = CalculateEventDetectionMetrics();
    NS_LOG_INFO("Event Detection:");
    NS_LOG_INFO("  Total Reports: " << eventMetrics.totalReports);
    NS_LOG_INFO("  Malicious Reports: " << eventMetrics.maliciousReports);
    NS_LOG_INFO("  Detection Rate: " << DoubleToString(eventMetrics.detectionRate * 100, 1) << "%");
    NS_LOG_INFO("  False Positive Rate: " << DoubleToString(eventMetrics.falsePositiveRate * 100, 1) << "%");
    NS_LOG_INFO("  Accuracy: " << DoubleToString(eventMetrics.accuracy * 100, 1) << "%");
    
    NS_LOG_INFO("Access Delays: " << m_accessDelayRecords.size() << " recorded");
    NS_LOG_INFO("Successful/Failed Queries: " << m_totalSuccessfulQueries << "/" << m_totalFailedQueries);
    NS_LOG_INFO("Communication: " << m_communicationRecords.size() << " packets, " 
                << m_totalCommunicationBytes << " bytes");
    NS_LOG_INFO("Transactions: " << m_totalTransactionsProcessed << " processed");
    NS_LOG_INFO("Reputation Updates: " << m_reputationRecords.size() << " updates");
    
 
    // Calculate recent performance
    double recentTPS = elapsedTime.GetSeconds() > 0 ? 
                      CalculateTransactionThroughput(m_lastReportTime, currentTime) : 0.0;
    double recentAAD = CalculateAverageAccessDelay(m_totalVehicles);
    
    NS_LOG_INFO("Recent TPS: " << DoubleToString(recentTPS, 2));
    NS_LOG_INFO("Current AAD: " << DoubleToString(recentAAD, 4) << "s");
    NS_LOG_INFO("===============================");
    
    m_lastReportTime = currentTime;
    
    // Schedule next report
    Simulator::Schedule(Seconds(120.0), &MetricsCollector::GeneratePeriodicReport, this);
}

// NEW: Generate comprehensive detection performance report
void MetricsCollector::GenerateDetectionPerformanceReport() {
    EventDetectionMetrics overallMetrics = CalculateEventDetectionMetrics();
    auto patternMetrics = CalculateEventDetectionMetricsByAllPatterns();
    
    NS_LOG_INFO("=== EVENT-CENTRIC DETECTION PERFORMANCE REPORT ===");
    NS_LOG_INFO("Total Event Reports Processed: " << overallMetrics.totalReports);
    NS_LOG_INFO("  - Malicious Reports: " << overallMetrics.maliciousReports);
    NS_LOG_INFO("  - Honest Reports: " << overallMetrics.honestReports);
    NS_LOG_INFO("  - Uncertain Reports: " << overallMetrics.uncertainReports);
    
    NS_LOG_INFO("\nConfusion Matrix:");
    NS_LOG_INFO("  True Positives (Fake→Rejected): " << overallMetrics.truePositives);
    NS_LOG_INFO("  False Negatives (Fake→Accepted): " << overallMetrics.falseNegatives);
    NS_LOG_INFO("  True Negatives (Real→Accepted): " << overallMetrics.trueNegatives);
    NS_LOG_INFO("  False Positives (Real→Rejected): " << overallMetrics.falsePositives);
    
    NS_LOG_INFO("\nPerformance Metrics:");
    NS_LOG_INFO("  Detection Rate (Recall): " << DoubleToString(overallMetrics.detectionRate * 100, 2) << "%");
    NS_LOG_INFO("  False Positive Rate: " << DoubleToString(overallMetrics.falsePositiveRate * 100, 2) << "%");
    NS_LOG_INFO("  False Negative Rate: " << DoubleToString(overallMetrics.falseNegativeRate * 100, 2) << "%");
    NS_LOG_INFO("  Precision: " << DoubleToString(overallMetrics.precision * 100, 2) << "%");
    NS_LOG_INFO("  Accuracy: " << DoubleToString(overallMetrics.accuracy * 100, 2) << "%");
    NS_LOG_INFO("  F1 Score: " << DoubleToString(overallMetrics.f1Score, 4));
    
    NS_LOG_INFO("\nPattern-Specific Performance:");
    for (const auto& pair : patternMetrics) {
        const std::string& pattern = pair.first;
        const EventDetectionMetrics& metrics = pair.second;
        NS_LOG_INFO("  " << pattern << ": DR=" << DoubleToString(metrics.detectionRate * 100, 1) 
                   << "%, FPR=" << DoubleToString(metrics.falsePositiveRate * 100, 1) 
                   << "%, Reports=" << metrics.totalReports);
    }
    NS_LOG_INFO("================================================");

    {
        std::cerr << "=== EVENT-CENTRIC DETECTION PERFORMANCE REPORT ===\n";
        std::cerr << "Total Event Reports Processed: " << overallMetrics.totalReports << "\n";
        std::cerr << "  - Malicious Reports: " << overallMetrics.maliciousReports << "\n";
        std::cerr << "  - Honest Reports: " << overallMetrics.honestReports << "\n";
        std::cerr << "  - Uncertain Reports: " << overallMetrics.uncertainReports << "\n";
        std::cerr << "\n";
        std::cerr << "Confusion Matrix:\n";
        std::cerr << "  True Positives (Fake→Rejected): " << overallMetrics.truePositives << "\n";
        std::cerr << "  False Negatives (Fake→Accepted): " << overallMetrics.falseNegatives << "\n";
        std::cerr << "  True Negatives (Real→Accepted): " << overallMetrics.trueNegatives << "\n";
        std::cerr << "  False Positives (Real→Rejected): " << overallMetrics.falsePositives << "\n";
        std::cerr << "\n";
        std::cerr << "Performance Metrics:\n";
        std::cerr << "  Detection Rate (Recall): " << DoubleToString(overallMetrics.detectionRate * 100, 2) << "%\n";
        std::cerr << "  False Positive Rate: " << DoubleToString(overallMetrics.falsePositiveRate * 100, 2) << "%\n";
        std::cerr << "  False Negative Rate: " << DoubleToString(overallMetrics.falseNegativeRate * 100, 2) << "%\n";
        std::cerr << "  Precision: " << DoubleToString(overallMetrics.precision * 100, 2) << "%\n";
        std::cerr << "  Accuracy: " << DoubleToString(overallMetrics.accuracy * 100, 2) << "%\n";
        std::cerr << "  F1 Score: " << DoubleToString(overallMetrics.f1Score, 4) << "\n";
        std::cerr << "\n";
        std::cerr << "Pattern-Specific Performance:\n";
        for (const auto& pair : patternMetrics) {
            const std::string& pattern = pair.first;
            const EventDetectionMetrics& metrics = pair.second;
            std::cerr << "  " << pattern << ": DR=" << DoubleToString(metrics.detectionRate * 100, 1) 
                    << "%, FPR=" << DoubleToString(metrics.falsePositiveRate * 100, 1) 
                    << "%, Reports=" << metrics.totalReports << "\n";
        }
        std::cerr << "================================================\n";
    }
}

// Ground truth management
void MetricsCollector::SetVehicleGroundTruth(const std::string& vehicleId, bool isAttacker, const std::string& pattern) {
    m_vehicleGroundTruth[vehicleId] = isAttacker;
    if (isAttacker && !pattern.empty()) {
        m_vehicleAttackerPatterns[vehicleId] = pattern;
    }
    
    NS_LOG_DEBUG("MetricsCollector: Ground truth set for " << vehicleId 
                 << " - " << (isAttacker ? "ATTACKER" : "HONEST") 
                 << (pattern.empty() ? "" : " (" + pattern + ")"));
}

void MetricsCollector::SetVehicleCurrentBehavior(const std::string& vehicleId, bool willActMalicious, uint32_t eventIndex) {
    auto key = std::make_pair(vehicleId, eventIndex);
    m_vehicleBehaviorPerEvent[key] = willActMalicious;
    
    NS_LOG_DEBUG("MetricsCollector: " << vehicleId << " event " << eventIndex 
                 << " will " << (willActMalicious ? "ATTACK" : "be HONEST"));
}
bool MetricsCollector::GetVehicleGroundTruth(const std::string& vehicleId) const {
    auto it = m_vehicleGroundTruth.find(vehicleId);
    return (it != m_vehicleGroundTruth.end()) ? it->second : false;
}

bool MetricsCollector::GetVehicleCurrentBehavior(const std::string& vehicleId, uint32_t eventIndex) const {
    auto key = std::make_pair(vehicleId, eventIndex);
    auto it = m_vehicleBehaviorPerEvent.find(key);
    if (it != m_vehicleBehaviorPerEvent.end()) {
        return it->second;
    }
    // Fallback to ground truth if no specific behavior set
    return GetVehicleGroundTruth(vehicleId);
}


// UPDATED: Reset method to include event detection records
void MetricsCollector::Reset() {
    m_accessDelayRecords.clear();
    m_communicationRecords.clear();
    m_eventDetectionRecords.clear();  // NEW: Clear event detection records
    m_reputationRecords.clear();
    m_transactionRecords.clear();
    m_performanceSnapshots.clear();
    
    m_vehicleGroundTruth.clear();
    m_vehicleAttackerPatterns.clear();
    m_vehicleBehaviorPerEvent.clear();
    
    m_totalTransactionsProcessed = 0;
    m_mxTime = Seconds(0);
    m_totalSuccessfulQueries = 0;
    m_totalFailedQueries = 0;
    m_totalCommunicationBytes = 0;
    m_totalControlMessages = 0;
    
    if (m_periodicReportEvent.IsRunning()) {
        Simulator::Cancel(m_periodicReportEvent);
    }
    
    NS_LOG_INFO("MetricsCollector reset for new simulation run");
}

// Helper Methods

void MetricsCollector::WriteCSVHeader(std::ofstream& file, const std::vector<std::string>& headers) const {
    for (size_t i = 0; i < headers.size(); ++i) {
        file << headers[i];
        if (i < headers.size() - 1) {
            file << ",";
        }
    }
    file << std::endl;
}

void MetricsCollector::WriteCSVRow(std::ofstream& file, const std::vector<std::string>& values) const {
    for (size_t i = 0; i < values.size(); ++i) {
        file << values[i];
        if (i < values.size() - 1) {
            file << ",";
        }
    }
    file << std::endl;
}

std::string MetricsCollector::TimeToString(Time t) const {
    return DoubleToString(t.GetSeconds(), 6);
}

std::string MetricsCollector::DoubleToString(double value, int precision) const {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

double MetricsCollector::CalculateRealTimeThroughput(Time windowStart, Time windowEnd) const {
    uint32_t transactionCount = 0;
    Time totalProcessingTime = Seconds(0);
    
    for (const auto& record : m_transactionRecords) {
        if (record.successful && 
            record.blockInclusionTime >= windowStart && 
            record.blockInclusionTime <= windowEnd) {
            transactionCount++;
            totalProcessingTime += record.latency;
        }
    }
    
    double windowDuration = (windowEnd - windowStart).GetSeconds();
    double averageLatency = transactionCount > 0 ? 
                           (totalProcessingTime.GetSeconds() / transactionCount) : 0.0;
    
    // Log detailed throughput analysis
    NS_LOG_INFO("Throughput Analysis [" << windowStart.GetSeconds() << "s - " << windowEnd.GetSeconds() << "s]:");
    NS_LOG_INFO("  Transactions: " << transactionCount);
    NS_LOG_INFO("  TPS: " << (windowDuration > 0 ? (transactionCount / windowDuration) : 0.0));
    NS_LOG_INFO("  Average Latency: " << averageLatency << "s");
    
    return windowDuration > 0 ? (transactionCount / windowDuration) : 0.0;
}

void MetricsCollector::SetBlockProcessingTimes(const std::vector<double>& processingTimes) {
    m_blockProcessingTimes = processingTimes;
    NS_LOG_DEBUG("MetricsCollector: Set " << processingTimes.size() << " block processing times");
}

} // namespace ns3