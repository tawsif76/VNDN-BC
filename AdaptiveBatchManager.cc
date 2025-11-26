#include "AdaptiveBatchManager.hpp"
#include "VanetBlockchainApp.hpp"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AdaptiveBatchManager");

AdaptiveBatchManager::AdaptiveBatchManager(VanetBlockchainApp* parent)
    : m_parent(parent),
    //   m_batchMin(10),                    // Minimum batch size
    //   m_batchMax(80),                   // Maximum batch size  
    //   m_batchBase(30),                  // Base batch size
    //   m_latencyMax(3.0),                // Max acceptable latency (seconds)
        m_batchMin(50),                     // Minimum batch size
        m_batchMax(200),                    // Maximum batch size (allow larger batches)
        m_batchBase(100),                   // Base batch size
        m_latencyMax(3.0),                  // Max acceptable latency (seconds)
        m_timeMax(Seconds(1.0)),            // Increase max time trigger significantly to 1 second        // Max time trigger
      m_currentTransactionRate(0.0),
      m_averageTransactionRate(10.0),   // Initial estimate
      m_currentLatency(0.5),            // Initial latency estimate
      m_lastBatchTime(Seconds(0)),
      m_arrivalRateWindow(Seconds(30.0)), // 30-second window for rate calculation
      m_latencyHistorySize(MAX_LATENCY_HISTORY),
      m_currentVehicleCount(0),
      m_networkCongestionFactor(1.0),
      m_processingInProgress(false)
{
    NS_LOG_INFO("[AdaptiveBatchManager] Initialized with DABP algorithm");
    NS_LOG_INFO("  Batch size range: [" << m_batchMin << ", " << m_batchMax << "]");
    NS_LOG_INFO("  Base batch size: " << m_batchBase);
    NS_LOG_INFO("  Max latency threshold: " << m_latencyMax << "s");
    NS_LOG_INFO("  Max time trigger: " << m_timeMax.GetSeconds() << "s");
}

void AdaptiveBatchManager::AddTransaction(const Transaction& tx) {
    if (m_processingInProgress) {
        NS_LOG_WARN("[AdaptiveBatchManager] Cannot add transaction - batch processing in progress");
        return;
    }
    
    // Prevent duplicate transactions
    std::string txId = m_parent->GenerateTransactionId(tx);
    if (m_processedTransactionIds.count(txId)) {
        NS_LOG_DEBUG("[AdaptiveBatchManager] Duplicate transaction ignored: " << txId);
        return;
    }
    
    // Record transaction arrival time for rate calculation
    Time arrivalTime = Simulator::Now();
    m_transactionArrivalTimes.push_back(arrivalTime);
    
    // Maintain arrival time window
    while (!m_transactionArrivalTimes.empty() && 
           (arrivalTime - m_transactionArrivalTimes.front()) > m_arrivalRateWindow) {
        m_transactionArrivalTimes.pop_front();
    }
    
    // Add transaction to buffer
    m_batchBuffer.push_back(tx);
    m_processedTransactionIds.insert(txId);
    
    // Update real-time parameters
    UpdateTransactionArrivalRate();
    UpdateCurrentLatency();
    
    // Calculate current adaptive batch size
    uint32_t adaptiveBatchSize = CalculateAdaptiveBatchSize();
    
    NS_LOG_DEBUG("[AdaptiveBatchManager] TX added. Buffer: " << m_batchBuffer.size() 
                 << ", Adaptive target: " << adaptiveBatchSize 
                 << ", Rate: " << m_currentTransactionRate << " tx/s"
                 << ", Latency: " << m_currentLatency << "s");
    
    // PRIORITY 1: Adaptive size-based trigger
    if (m_batchBuffer.size() >= adaptiveBatchSize) {
        NS_LOG_INFO("[AdaptiveBatchManager] Adaptive batch size reached (" 
                    << adaptiveBatchSize << "). Processing immediately.");
        m_metrics.sizeTriggerActivations++;
        ProcessBatch();
    }
    // PRIORITY 2: Time-based trigger check
    else if (CheckTimeTrigger()) {
        NS_LOG_INFO("[AdaptiveBatchManager] Time trigger activated (" 
                    << m_timeMax.GetSeconds() << "s). Processing batch.");
        m_metrics.timeTriggerActivations++;
        ProcessBatch();
    }
    // PRIORITY 3: Start adaptive timer if not running
    else if (!m_adaptiveTimer.IsRunning() && !m_batchBuffer.empty()) {
        // Dynamic timer based on current conditions
        Time adaptiveInterval = std::min(m_timeMax, 
                                       Seconds(std::max(0.06, m_latencyMax - m_currentLatency))); // was 0.05
        
        NS_LOG_DEBUG("[AdaptiveBatchManager] Starting adaptive timer for " 
                     << adaptiveInterval.GetSeconds() << "s");
        m_adaptiveTimer = Simulator::Schedule(adaptiveInterval, 
                                            &AdaptiveBatchManager::ProcessBatch, this);
    }
}

uint32_t AdaptiveBatchManager::CalculateAdaptiveBatchSize() {
    // DABP Algorithm Implementation:
    // B_adaptive(t) = min(B_max, max(B_min, B_base × (λ(t)/λ_avg) × e^(-L(t)/L_max)))
    
    double rateRatio = (m_averageTransactionRate > 0.0) ? 
                      (m_currentTransactionRate / m_averageTransactionRate) : 1.0;
    
    double latencyFactor = std::exp(-m_currentLatency / m_latencyMax);
    
    // Include network congestion factor
    double congestionAdjustment = 1.0 / m_networkCongestionFactor;
    
    double rawBatchSize = m_batchBase * rateRatio * latencyFactor * congestionAdjustment;
    
    uint32_t adaptiveBatchSize = static_cast<uint32_t>(
        std::min(static_cast<double>(m_batchMax),
                std::max(static_cast<double>(m_batchMin), rawBatchSize))
    );
    
    NS_LOG_DEBUG("[AdaptiveBatchManager] DABP Calculation:");
    NS_LOG_DEBUG("  Rate ratio (λ(t)/λ_avg): " << rateRatio);
    NS_LOG_DEBUG("  Latency factor: " << latencyFactor);
    NS_LOG_DEBUG("  Congestion adjustment: " << congestionAdjustment);
    NS_LOG_DEBUG("  Raw batch size: " << rawBatchSize);
    NS_LOG_DEBUG("  Final adaptive size: " << adaptiveBatchSize);
    
    return adaptiveBatchSize;
}

void AdaptiveBatchManager::UpdateTransactionArrivalRate() {
    if (m_transactionArrivalTimes.size() < 2) {
        return; // Need at least 2 transactions to calculate rate
    }
    
    Time windowStart = m_transactionArrivalTimes.back() - m_arrivalRateWindow;
    Time windowEnd = m_transactionArrivalTimes.back();
    double windowDuration = (windowEnd - windowStart).GetSeconds();
    
    if (windowDuration > 0.0) {
        double instantRate = static_cast<double>(m_transactionArrivalTimes.size()) / windowDuration;
        
        // Apply exponential moving average for smooth rate tracking
        if (m_currentTransactionRate == 0.0) {
            m_currentTransactionRate = instantRate;
        } else {
            m_currentTransactionRate = TRANSACTION_RATE_ALPHA * m_currentTransactionRate + 
                                     (1.0 - TRANSACTION_RATE_ALPHA) * instantRate;
        }
        
        // Update historical average
        if (m_averageTransactionRate == 0.0) {
            m_averageTransactionRate = m_currentTransactionRate;
        } else {
            m_averageTransactionRate = 0.95 * m_averageTransactionRate + 
                                     0.05 * m_currentTransactionRate;
        }
    }
}

void AdaptiveBatchManager::UpdateCurrentLatency() {
    // In a real implementation, this would measure actual network latency
    // For simulation, we estimate based on network conditions
    
    // Base latency estimate
    double baseLatency = 0.5; // seconds
    
    // Adjust for network congestion
    double congestionLatency = baseLatency * m_networkCongestionFactor;
    
    // Adjust for transaction rate (higher rate = higher latency)
    double rateLatency = congestionLatency * (1.0 + (m_currentTransactionRate / 100.0));
    
    // Adjust for vehicle density
    double densityLatency = rateLatency * (1.0 + (m_currentVehicleCount / 1000.0));
    
    m_currentLatency = std::min(m_latencyMax, densityLatency);
    
    // Store in history for analysis
    m_latencyHistory.push_back(m_currentLatency);
    if (m_latencyHistory.size() > m_latencyHistorySize) {
        m_latencyHistory.pop_front();
    }
    
    NS_LOG_DEBUG("[AdaptiveBatchManager] Latency update: " << m_currentLatency 
                 << "s (congestion: " << m_networkCongestionFactor 
                 << ", vehicles: " << m_currentVehicleCount << ")");
}

bool AdaptiveBatchManager::CheckTimeTrigger() {
    if (m_batchBuffer.empty()) {
        return false;
    }
    
    Time currentTime = Simulator::Now();
    Time timeSinceLastBatch = currentTime - m_lastBatchTime;
    
    bool timeTriggerActive = (timeSinceLastBatch >= m_timeMax);
    
    if (timeTriggerActive) {
        NS_LOG_INFO("[AdaptiveBatchManager] Time trigger activated: " 
                    << timeSinceLastBatch.GetSeconds() << "s >= " 
                    << m_timeMax.GetSeconds() << "s");
    }
    
    return timeTriggerActive;
}

void AdaptiveBatchManager::ProcessBatch() {
    if (m_processingInProgress) {
        NS_LOG_WARN("[AdaptiveBatchManager] Batch processing already in progress");
        return;
    }
    
    if (m_batchBuffer.empty()) {
        NS_LOG_DEBUG("[AdaptiveBatchManager] No transactions to process");
        if (m_adaptiveTimer.IsRunning()) m_adaptiveTimer.Cancel();
        return;
    }
    
    m_processingInProgress = true;
    Time processingStartTime = Simulator::Now();
    
    NS_LOG_INFO("[AdaptiveBatchManager] Processing adaptive batch of " 
                << m_batchBuffer.size() << " transactions");
    NS_LOG_INFO("  Current rate: " << m_currentTransactionRate << " tx/s");
    NS_LOG_INFO("  Current latency: " << m_currentLatency << "s");
    NS_LOG_INFO("  Network congestion: " << m_networkCongestionFactor);
    
    if (m_parent->GetNodeName() == "RSU-0") {
        // Create a copy for processing
        std::vector<Transaction> batchToProcess = m_batchBuffer;
        
        // PHASE 1: Send to followers with adaptive batch optimization
        SendAdaptiveBatch(batchToProcess);
        
        // PHASE 2: Add to own transaction pool
        for (const auto& tx : batchToProcess) {
            m_parent->AddTransactionToPool(tx);
        }
        
        // PHASE 3: Schedule block proposal with adaptive delay
        double adaptiveDelay = std::max(0.05, std::min(0.5, m_currentLatency));
        Simulator::Schedule(Seconds(adaptiveDelay), 
                          &VanetBlockchainApp::ProposeBlockForBatch, m_parent);
        
        NS_LOG_INFO("[AdaptiveBatchManager] Block proposal scheduled with adaptive delay: " 
                    << adaptiveDelay << "s");
    }
    
    // Update metrics
    m_metrics.totalBatchesProcessed++;
    m_metrics.averageBatchSize = (m_metrics.averageBatchSize * (m_metrics.totalBatchesProcessed - 1) + 
                                 m_batchBuffer.size()) / m_metrics.totalBatchesProcessed;
    
    // Update timing
    m_lastBatchTime = processingStartTime;
    m_metrics.totalAdaptiveTime += (Simulator::Now() - processingStartTime);
    
    // Clear batch and reset state
    m_batchBuffer.clear();
    if (m_adaptiveTimer.IsRunning()) m_adaptiveTimer.Cancel();
    m_processingInProgress = false;
    
    // Clean up old processed IDs to prevent memory growth
    if (m_processedTransactionIds.size() > 1000) {
        m_processedTransactionIds.clear();
        NS_LOG_INFO("[AdaptiveBatchManager] Cleared processed transaction cache");
    }
    
    // Perform adaptive parameter adjustment
    AdaptiveParameterAdjustment();
}

void AdaptiveBatchManager::SendAdaptiveBatch(const std::vector<Transaction>& batch) {
    if (batch.empty() || !m_parent || m_parent->GetNodeName() != "RSU-0") {
        return;
    }
    
    ndn::Name batchInterestName("/vanet/pbft/adaptive-transaction-batch");
    batchInterestName.append(m_parent->GetNodeName());
    batchInterestName.appendNumber(batch.size());
    batchInterestName.appendNumber(static_cast<uint64_t>(m_currentTransactionRate * 100)); // Rate info
    batchInterestName.appendNumber(static_cast<uint64_t>(m_currentLatency * 1000)); // Latency info
    batchInterestName.appendTimestamp();
    
    auto interest = std::make_shared<ndn::Interest>(batchInterestName);
    
    // Efficient serialization with adaptive metadata
    std::stringstream batchPayload;
    batchPayload << "ADAPTIVE_BATCH:" << batch.size() 
                 << "|RATE:" << m_currentTransactionRate
                 << "|LATENCY:" << m_currentLatency
                 << "|CONGESTION:" << m_networkCongestionFactor;
    
    for (size_t i = 0; i < batch.size(); ++i) {
        batchPayload << "|TX" << i << ":" << batch[i].serialize();
    }
    
    std::string payloadStr = batchPayload.str();
    auto buffer = std::make_shared<::ndn::Buffer>(payloadStr.begin(), payloadStr.end());
    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);
    
    interest->setInterestLifetime(ndn::time::seconds(10));
    interest->setNonce(CreateObject<UniformRandomVariable>()->GetValue(0, UINT32_MAX));
    
    NS_LOG_INFO("[AdaptiveBatchManager] Broadcasting adaptive batch: " << batch.size() 
                << " transactions (" << payloadStr.size() << " bytes)"
                << " [Rate: " << m_currentTransactionRate << " tx/s]"
                << " [Latency: " << m_currentLatency << "s]");
    
    m_parent->SendInterest(interest);
}

void AdaptiveBatchManager::UpdateNetworkParameters(uint32_t vehicleCount, double averageLatency) {
    m_currentVehicleCount = vehicleCount;
    
    // Update network congestion factor based on vehicle density
    double densityFactor = static_cast<double>(vehicleCount) / 100.0; // Normalize to 100 vehicles
    double newCongestionFactor = 1.0 + (densityFactor * 0.5); // Linear increase with density
    
    // Apply exponential moving average for smooth congestion tracking
    m_networkCongestionFactor = CONGESTION_FACTOR_ALPHA * m_networkCongestionFactor + 
                               (1.0 - CONGESTION_FACTOR_ALPHA) * newCongestionFactor;
    
    // Update latency if provided
    if (averageLatency > 0.0) {
        m_currentLatency = averageLatency;
    }
    
    NS_LOG_DEBUG("[AdaptiveBatchManager] Network parameters updated: "
                 << "vehicles=" << vehicleCount 
                 << ", congestion=" << m_networkCongestionFactor
                 << ", latency=" << m_currentLatency << "s");
}

void AdaptiveBatchManager::AdaptiveParameterAdjustment() {
    // Adjust parameters based on recent performance
    
    // If latency consistently high, reduce base batch size
    if (!m_latencyHistory.empty()) {
        double avgLatency = std::accumulate(m_latencyHistory.begin(), m_latencyHistory.end(), 0.0) 
                           / m_latencyHistory.size();
        
        if (avgLatency > (m_latencyMax * LATENCY_UPPER_THRESHOLD_RATIO)) {
            m_batchBase = std::max(m_batchMin, static_cast<uint32_t>(m_batchBase * 0.9));
            NS_LOG_INFO("[AdaptiveBatchManager] High latency detected. Reduced base batch size to: " 
                        << m_batchBase);
        }
        else if (avgLatency < (m_latencyMax * LATENCY_LOWER_THRESHOLD_RATIO)) {
            m_batchBase = std::min(m_batchMax, static_cast<uint32_t>(m_batchBase * 1.1));
            NS_LOG_DEBUG("[AdaptiveBatchManager] Low latency detected. Increased base batch size to: " 
                         << m_batchBase);
        }
    }
    
    const Time maxTimeTrigger = Seconds(1.0); // Match the new value from the constructor
    const Time minTimeTrigger = Seconds(0.05); // Keep a reasonable floor

    if (m_currentTransactionRate > (m_averageTransactionRate * 1.5)) {
        // High transaction rate - reduce time trigger
        m_timeMax = std::max(minTimeTrigger, Seconds(m_timeMax.GetSeconds() * 0.9));
    }
    else if (m_currentTransactionRate < (m_averageTransactionRate * 0.5)) {
        // Low transaction rate - increase time trigger to accumulate more transactions
        m_timeMax = std::min(maxTimeTrigger, Seconds(m_timeMax.GetSeconds() * 1.2));
    }
}

uint32_t AdaptiveBatchManager::GetCurrentAdaptiveBatchSize() const {
    return const_cast<AdaptiveBatchManager*>(this)->CalculateAdaptiveBatchSize();
}

size_t AdaptiveBatchManager::GetBatchBufferSize() const {
    return m_batchBuffer.size();
}

void AdaptiveBatchManager::Reset() {
    m_batchBuffer.clear();
    m_processedTransactionIds.clear();
    m_transactionArrivalTimes.clear();
    m_latencyHistory.clear();
    
    if (m_adaptiveTimer.IsRunning()) {
        m_adaptiveTimer.Cancel();
    }
    
    m_processingInProgress = false;
    m_lastBatchTime = Seconds(0);
    m_currentTransactionRate = 0.0;
    m_currentLatency = 0.5;
    m_networkCongestionFactor = 1.0;
    
    // Reset metrics
    m_metrics = AdaptiveMetrics();
    
    NS_LOG_INFO("[AdaptiveBatchManager] Reset completed - ready for new simulation");
}

} // namespace ns3