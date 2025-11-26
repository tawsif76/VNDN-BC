#ifndef ADAPTIVE_BATCH_MANAGER_HPP
#define ADAPTIVE_BATCH_MANAGER_HPP

#include "ns3/core-module.h"
#include "VanetBlock.hpp"
#include <vector>
#include <set>
#include <deque>

namespace ns3 {

// Forward declaration
class VanetBlockchainApp;

/**
 * @brief Dynamic Adaptive Batch Processing Manager for VNDN
 * 
 * Implements adaptive batch size calculation based on:
 * - Real-time transaction arrival rate
 * - Current network latency
 * - Vehicle density and network congestion
 */
class AdaptiveBatchManager {
public:
    /**
     * @brief Constructor
     * @param parent Pointer to the parent VanetBlockchainApp
     */
    AdaptiveBatchManager(VanetBlockchainApp* parent);
    
    /**
     * @brief Add a transaction to the adaptive batch buffer
     * @param tx Transaction to add
     */
    void AddTransaction(const Transaction& tx);
    
    /**
     * @brief Process the current batch based on adaptive criteria
     */
    void ProcessBatch();
    
    /**
     * @brief Get current batch buffer size
     * @return Number of transactions in buffer
     */
    size_t GetBatchBufferSize() const;
    
    /**
     * @brief Update network parameters for adaptive calculation
     * @param vehicleCount Current number of vehicles in range
     * @param averageLatency Current average network latency
     */
    void UpdateNetworkParameters(uint32_t vehicleCount, double averageLatency);
    
    /**
     * @brief Get current adaptive batch size
     * @return Calculated adaptive batch size
     */
    uint32_t GetCurrentAdaptiveBatchSize() const;
    
    /**
     * @brief Reset the batch manager state
     */
    void Reset();

private:
    // Core adaptive batch processing methods
    
    /**
     * @brief Calculate adaptive batch size using DABP algorithm
     * @return Calculated batch size
     */
    uint32_t CalculateAdaptiveBatchSize();
    
    /**
     * @brief Update transaction arrival rate statistics
     */
    void UpdateTransactionArrivalRate();
    
    /**
     * @brief Monitor current network latency
     */
    void UpdateCurrentLatency();
    
    /**
     * @brief Check if time-based trigger should activate
     * @return True if time trigger should activate batch processing
     */
    bool CheckTimeTrigger();
    
    /**
     * @brief Update adaptive parameters based on network conditions
     */
    void AdaptiveParameterAdjustment();
    
    /**
     * @brief Send the batch to followers and add to transaction pool
     * @param batchToProcess Transactions to process
     */
    void SendAdaptiveBatch(const std::vector<Transaction>& batchToProcess);

    // DABP Algorithm Parameters
    uint32_t m_batchMin;              // Minimum batch size (B_min)
    uint32_t m_batchMax;              // Maximum batch size (B_max)
    uint32_t m_batchBase;             // Base batch size (B_base)
    double m_latencyMax;              // Maximum acceptable latency (L_max)
    Time m_timeMax;                   // Maximum time trigger (T_max)
    
    // Real-time monitoring variables
    double m_currentTransactionRate;   // Current λ(t)
    double m_averageTransactionRate;   // Historical λ_avg
    double m_currentLatency;           // Current L(t)
    Time m_lastBatchTime;             // Time of last batch processing
    
    // Transaction arrival rate tracking
    std::deque<Time> m_transactionArrivalTimes; // For rate calculation
    Time m_arrivalRateWindow;         // Window for calculating arrival rate
    
    // Latency tracking
    std::deque<double> m_latencyHistory;       // Historical latency data
    size_t m_latencyHistorySize;               // Max history size
    
    // Network condition tracking
    uint32_t m_currentVehicleCount;   // Current vehicle density
    double m_networkCongestionFactor; // Network congestion estimate
    
    // Batch processing state
    std::vector<Transaction> m_batchBuffer;
    std::set<std::string> m_processedTransactionIds;
    bool m_processingInProgress;
    EventId m_adaptiveTimer;          // Timer for adaptive processing
    
    // Parent application reference
    VanetBlockchainApp* m_parent;
    
    // Performance monitoring
    struct AdaptiveMetrics {
        uint32_t totalBatchesProcessed = 0;
        uint32_t timeTriggerActivations = 0;
        uint32_t sizeTriggerActivations = 0;
        double averageBatchSize = 0.0;
        double averageProcessingLatency = 0.0;
        Time totalAdaptiveTime = Seconds(0);
    };
    AdaptiveMetrics m_metrics;
    
    // Configuration constants
    static const size_t MAX_ARRIVAL_HISTORY = 100;
    static const size_t MAX_LATENCY_HISTORY = 50;
    static constexpr double LATENCY_UPPER_THRESHOLD_RATIO = 0.75; // 0.8
    static constexpr double LATENCY_LOWER_THRESHOLD_RATIO = 0.25; //0.3
    static constexpr double CONGESTION_FACTOR_ALPHA = 0.7;
    static constexpr double TRANSACTION_RATE_ALPHA = 0.8;
};

} // namespace ns3

#endif // ADAPTIVE_BATCH_MANAGER_HPP