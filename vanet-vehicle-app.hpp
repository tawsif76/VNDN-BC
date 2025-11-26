#ifndef VANET_VEHICLE_APP_HPP
#define VANET_VEHICLE_APP_HPP

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/log.h"
#include "MetricsCollector.hpp"
#include <vector>

namespace ns3 {

// Structure to store pending event reports
struct PendingEventReport {
    std::string eventType;
    std::string eventLocation;
    Time eventTime;
    uint32_t retryCount;
    Time scheduleTime;
    std::string targetRsu;
};

class VanetVehicleApp : public ndn::App {
public:
    static TypeId GetTypeId();

    VanetVehicleApp();
    virtual void StartApplication();
    virtual void StopApplication();
    virtual void OnInterest(std::shared_ptr<const ndn::Interest> interest);
    virtual void OnData(std::shared_ptr<const ndn::Data> data);

    void SetVehicleID(const std::string& id);
    void SetTargetRsuName(const std::string& rsuName);
    void SetAttackerStatus(bool isAttacker);

    // Methods to be called by Simulator::Schedule
    void ScheduleRegistrationRequest();
    void ScheduleEventReport(std::string eventType, std::string eventLocation, Time eventTime);
    void SetMetricsCollector(Ptr<MetricsCollector> collector);
    
    // Add this method declaration to the public section:
    std::string GetAssignedRsu() const;
    void CheckRegistrationStatus(); // Method to check if we should retry
    uint32_t GetTotalEventReportsSent() const; // Get total event reports sent
    void TryNextRsuIfNeeded();
    void SetAvailableRsus(const std::vector<std::string>& rsuList);
    void ReportVehicleMetrics();
    void StartPeriodicMetricsReporting();
    void PeriodicMetricsReport();
    
    // Enhanced getters for superiority analysis
    bool IsCurrentlyAttacker() const { return m_isAttacker; }
    bool IsCurrentlyRegistered() const { return m_isRegistered; }
    uint32_t GetEventSequenceNumber() const { return m_eventSeqNum; }
    Time GetRegistrationRequestTime() const { return m_registrationRequestTime; }
    bool IsRegistrationPending() const { return m_registrationPending; }
    void ScheduleEventReportToSpecificRsu(std::string eventType, std::string eventLocation, 
                                         Time eventTime, std::string targetRsu);

    void SetBehaviorForEvent(uint32_t eventIndex, bool shouldAttack);
    bool ShouldAttackForEvent(uint32_t eventIndex) const;
    void SetCurrentEventIndex(uint32_t eventIndex);  
    void ScheduleLocationQuery(const std::string& queryLocation, const std::string& targetRsu);                                
private:
    void GenerateKeys();
    void SendRegistrationRequest();
    void SendEventReport(const std::string& eventType, const std::string& eventLocation, Time eventTime);
    void ProcessPendingEventReports(); // Process pending event reports when registered
    
    std::string SignData(const std::string& data_content); // Simplified
    // In a real scenario, this would use ndn-cxx KeyChain

    std::string m_vehicleId;
    std::string m_publicKey;  // Store as string for simplicity
    std::string m_privateKey; // Store as string for simplicity (not secure, for simulation only)
    std::string m_targetRsuName; // Name of the RSU to communicate with
    bool m_isAttacker;
    bool m_isRegistered;
    Ptr<MetricsCollector> m_metricsCollector;
    uint32_t m_eventSeqNum;
    ns3::Time m_registrationRequestTime;
    bool m_registrationPending;
    std::vector<std::string> m_availableRsus;    // List of all available RSUs
    uint32_t m_currentRsuIndex;                   // Current RSU being tried
    uint32_t m_maxRsuRetries;                     // Max RSUs to try
    EventId m_rsuFallbackEvent;                   // Event for RSU fallback timing

    
    // New members for multiple event reports and retry logic
    std::vector<PendingEventReport> m_pendingEventReports; // Store pending event reports
    uint32_t m_totalEventReportsSent; // Track total number of event reports sent
    uint32_t m_maxRetries; // Maximum number of retries for pending reports
    
    void QueryRegistrationStatus();
    void OnRegistrationStatusResponse(std::shared_ptr<const ndn::Data> data);
    ns3::EventId m_registrationRetryEvent; 
    void SendEventReportToSpecificRsu(const std::string& eventType, 
                                     const std::string& eventLocation, 
                                     Time eventTime, 
                                     const std::string& targetRsu);
    Time m_lastQueryTime;
    std::map<uint32_t, bool> m_eventSpecificBehavior;  // eventIndex -> shouldAttack
    uint32_t m_currentEventIndex = 0;


    // Location query functionality
    void SendLocationQuery(const std::string& queryLocation, const std::string& targetRsu);
    void OnLocationQueryResponse(std::shared_ptr<const ndn::Data> data);
    
    // Query state tracking
    std::map<std::string, Time> m_pendingLocationQueries;  // queryId -> requestTime
    uint32_t m_locationQueryCount = 0;
    std::map<std::string, Time> m_queryRequestTimes; 

};

} // namespace ns3

#endif // VANET_VEHICLE_APP_HPP