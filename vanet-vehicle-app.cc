#include "vanet-vehicle-app.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp" 
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
#include "ns3/random-variable-stream.h" // For nonce
#include "ns3/core-module.h" // For Simulator::Now()
// #include "block-helpers.hpp"
#include "ndn-cxx/detail/common.hpp"
#include <ndn-cxx/encoding/block-helpers.hpp> 
#include <ndn-cxx/encoding/tlv.hpp>
#include "ndn-cxx/encoding/buffer.hpp"

//send registration and event reports.

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("VanetVehicleApp");
NS_OBJECT_ENSURE_REGISTERED(VanetVehicleApp);

TypeId VanetVehicleApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::VanetVehicleApp")
        .SetParent<ndn::App>()
        .AddAttribute("VehicleID", "The unique ID of this vehicle.",
                      StringValue("V0"),
                      MakeStringAccessor(&VanetVehicleApp::m_vehicleId),
                      MakeStringChecker())
        .AddAttribute("TargetRsuName", "The name of the RSU to send requests to.",
                      StringValue("RSU-0"),
                      MakeStringAccessor(&VanetVehicleApp::m_targetRsuName),
                      MakeStringChecker())
        .AddConstructor<VanetVehicleApp>();
    return tid;
}

VanetVehicleApp::VanetVehicleApp() : m_isAttacker(false), m_isRegistered(false), m_eventSeqNum(0), 
                                     m_totalEventReportsSent(0), m_maxRetries(5) {
}


void VanetVehicleApp::StartApplication() {
    ndn::App::StartApplication();
    GenerateKeys();
    m_registrationPending = false;
    
    // Start periodic metrics reporting
    StartPeriodicMetricsReporting();
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] Started with enhanced metrics collection. Assigned to: " << m_targetRsuName 
                 << ", will register when scheduled");
}



void VanetVehicleApp::StopApplication() {
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] Stopping. Total event reports sent: " << m_totalEventReportsSent);
    ndn::App::StopApplication();
}

void VanetVehicleApp::GenerateKeys() {

    std::hash<std::string> hasher;
    m_publicKey = "PubKey_" + m_vehicleId + "_" + std::to_string(hasher(m_vehicleId + "pub"));
    m_privateKey = "PrivKey_" + m_vehicleId + "_" + std::to_string(hasher(m_vehicleId + "priv"));
}

std::string VanetVehicleApp::SignData(const std::string& data_content) {
 
    std::hash<std::string> hasher;
    return "Sig(" + data_content + ")_by_" + m_vehicleId + "_" + std::to_string(hasher(data_content + m_privateKey)); 
}

void VanetVehicleApp::SetVehicleID(const std::string& id) {
    m_vehicleId = id;
}

void VanetVehicleApp::SetTargetRsuName(const std::string& rsuName) {
    m_targetRsuName = rsuName;
} 

void VanetVehicleApp::SetAttackerStatus(bool isAttacker) {
    m_isAttacker = isAttacker;
}

void VanetVehicleApp::SendRegistrationRequest() {
    if (m_publicKey.empty()) {
        NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Cannot send registration, public key not generated.");
        return;
    }

    ndn::Name regInterestName("/vanet");
    regInterestName.append(m_targetRsuName);
    regInterestName.append("register");
    regInterestName.append(m_vehicleId);
    regInterestName.append(m_publicKey);

    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] *** SENDING REGISTRATION *** to " << m_targetRsuName);

    std::shared_ptr<ndn::Interest> interest = std::make_shared<ndn::Interest>(regInterestName);
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    interest->setNonce(rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
    interest->setInterestLifetime(ndn::time::seconds(30));

   
    m_registrationRequestTime = Simulator::Now();
    m_registrationPending = true;

    if (m_metricsCollector) {
        uint32_t requestSize = 500; // NDN overhead
        requestSize += 2; // Add public key size
        
        m_metricsCollector->RecordCommunicationOverhead("Interest_Registration", requestSize, 
                                                       m_vehicleId, m_targetRsuName, false); 
    }
    
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
    
    // Schedule a check to see if registration completed within reasonable time
    Simulator::Schedule(Seconds(20.0), &VanetVehicleApp::CheckRegistrationStatus, this);
}

// In vanet-vehicle-app.cc - modify ScheduleEventReport
void VanetVehicleApp::ScheduleEventReport(std::string eventType, std::string eventLocation, Time eventTime) {
    if (!m_isRegistered) {
        // Before giving up, query RSU one more time
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Not registered locally. Checking with RSU before reporting " << eventType);
        
        QueryRegistrationStatus();
        
        // Store the event for later sending
        PendingEventReport pendingReport;
        pendingReport.eventType = eventType;
        pendingReport.eventLocation = eventLocation;
        pendingReport.eventTime = eventTime;
        pendingReport.retryCount = 0;
        m_pendingEventReports.push_back(pendingReport);
        
        // Schedule retry after RSU response
        Simulator::Schedule(Seconds(2.0), &VanetVehicleApp::ProcessPendingEventReports, this);
        return;
    }
    
    SendEventReport(eventType, eventLocation, eventTime);
}

void VanetVehicleApp::ProcessPendingEventReports() {
    if (m_pendingEventReports.empty()) {
        return;
    }
    
    auto it = m_pendingEventReports.begin();
    while (it != m_pendingEventReports.end()) {
        if (m_isRegistered) {
            // Vehicle is now registered, send the pending report
            NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                         << "] Now registered! Sending pending event report for " << it->eventType
                         << " to " << (it->targetRsu.empty() ? m_targetRsuName : it->targetRsu));
            
            // *** Use specific RSU if provided, otherwise use default ***
            if (!it->targetRsu.empty()) {
                SendEventReportToSpecificRsu(it->eventType, it->eventLocation, it->eventTime, it->targetRsu);
            } else {
                SendEventReport(it->eventType, it->eventLocation, it->eventTime);
            }
            it = m_pendingEventReports.erase(it);
        } else {
            // Still not registered, check if we should retry
            it->retryCount++;
            if (it->retryCount < m_maxRetries) {
                double retryDelay = std::min(60.0, 5.0 * std::pow(2, it->retryCount));
                
                NFD_LOG_WARN("[" << GetNode()->GetId() << ":" << m_vehicleId 
                             << "] Still not registered. Retry " << it->retryCount 
                             << "/" << m_maxRetries << " for " << it->eventType 
                             << " in " << retryDelay << "s");
                
                Simulator::Schedule(Seconds(retryDelay), 
                                   &VanetVehicleApp::ProcessPendingEventReports, this);
                ++it;
            } else {
                NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                              << "] Giving up on event report for " << it->eventType 
                              << " after " << m_maxRetries << " retries");
                it = m_pendingEventReports.erase(it);
            }
        }
    }
}

void VanetVehicleApp::SendEventReportToSpecificRsu(const std::string& eventType, 
                                                  const std::string& eventLocation, 
                                                  Time eventTime, 
                                                  const std::string& targetRsu) {
    if (!m_isRegistered) {
        NFD_LOG_WARN("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Attempted to send event report but not registered!");
        return;
    }

    Time submissionTime = Simulator::Now();
    bool shouldAttackThisEvent = ShouldAttackForEvent(m_currentEventIndex);
    
    std::string reportContent = eventType; // Default: honest report
    if (shouldAttackThisEvent) {
        // Attack: invert the report
        if (eventType == "Accident") reportContent = "No Accident";
        else if (eventType == "No Accident") reportContent = "Accident";
        else if (eventType == "Jam") reportContent = "No Jam";
        else if (eventType == "No Jam") reportContent = "Jam";
        else if (eventType == "Roadwork") reportContent = "No Roadwork";
        else if (eventType == "No Roadwork") reportContent = "Roadwork";
        else if (eventType == "Construction") reportContent = "No Construction";
        else if (eventType == "No Construction") reportContent = "Construction";
        else if (eventType == "Breakdown") reportContent = "No Breakdown";
        else if (eventType == "No Breakdown") reportContent = "Breakdown";
    }

    std::string dataToSign = m_vehicleId + ";" + reportContent + ";" + eventLocation + ";" +
                             std::to_string(static_cast<long long>(eventTime.GetSeconds())) + ";" +
                             std::to_string(m_eventSeqNum);
    std::string signature = SignData(dataToSign);

        
    std::cerr << "[" << GetNode()->GetId() << ":" << m_vehicleId 
              << (shouldAttackThisEvent ? " (ATTACKING)" : " (HONEST)")
              << "] EVENT #" << m_totalEventReportsSent 
              << " Event: " << reportContent << " vs Truth: " << eventType
              << " (Event Index: " << m_currentEventIndex << ")" << std::endl;

    

                 
    // *** KEY CHANGE: Send to explicitly specified RSU instead of default ***
    ndn::Name eventInterestName("/vanet");
    eventInterestName.append(targetRsu);  // Use explicit target RSU
    eventInterestName.append("eventreport");
    eventInterestName.append(m_vehicleId);
    eventInterestName.appendTimestamp();

    std::shared_ptr<ndn::Interest> interest = std::make_shared<ndn::Interest>(eventInterestName);
    
    std::string payloadStr = m_vehicleId + "|" + reportContent + "|" + eventLocation + "|" +
                             std::to_string(static_cast<long long>(eventTime.GetSeconds())) + "|" +
                             std::to_string(m_eventSeqNum) + "|" + signature + "|" + eventType;

    auto buffer = std::make_shared<::ndn::Buffer>(payloadStr.begin(), payloadStr.end());
    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);

    m_eventSeqNum++;
    m_totalEventReportsSent++;

    if (m_metricsCollector) {
        uint32_t reportSize = 500; // NDN + payload overhead
        
        std::string packetType = shouldAttackThisEvent ? "Interest_EventReport_Malicious" : "Interest_EventReport_Honest";
        m_metricsCollector->RecordCommunicationOverhead(packetType, reportSize, 
                                                       m_vehicleId, m_targetRsuName, false);
    }

    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    interest->setNonce(rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
    interest->setInterestLifetime(ndn::time::seconds(10));

    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId << (m_isAttacker ? " (Attacker)" : "")
                     << "] *** EVENT REPORT #" << m_totalEventReportsSent << " *** "
                     << "EXPLICITLY routed to " << targetRsu << " (instead of " << m_targetRsuName << ")"
                     << " Event: " << reportContent << " vs Truth: " << eventType);
    
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
}

void VanetVehicleApp::SendEventReport(const std::string& eventType, const std::string& eventLocation, Time eventTime) {
    if (!m_isRegistered) {
        NFD_LOG_WARN("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Attempted to send event report but not registered!");
        return;
    }
    
    Time submissionTime = Simulator::Now(); // Record submission time for latency analysis
    
    bool shouldAttackThisEvent = ShouldAttackForEvent(m_currentEventIndex);
    
    std::string reportContent = eventType; // Default: honest report
    if (shouldAttackThisEvent) {
        // Attack: invert the report
        if (eventType == "Accident") reportContent = "No Accident";
        else if (eventType == "No Accident") reportContent = "Accident";
        else if (eventType == "Jam") reportContent = "No Jam";
        else if (eventType == "No Jam") reportContent = "Jam";
        else if (eventType == "Roadwork") reportContent = "No Roadwork";
        else if (eventType == "No Roadwork") reportContent = "Roadwork";
        else if (eventType == "Construction") reportContent = "No Construction";
        else if (eventType == "No Construction") reportContent = "Construction";
        else if (eventType == "Breakdown") reportContent = "No Breakdown";
        else if (eventType == "No Breakdown") reportContent = "Breakdown";
    }

    std::string dataToSign = m_vehicleId + ";" + reportContent + ";" + eventLocation + ";" +
                             std::to_string(static_cast<long long>(eventTime.GetSeconds())) + ";" +
                             std::to_string(m_eventSeqNum);
    std::string signature = SignData(dataToSign);

    std::cerr << "[" << GetNode()->GetId() << ":" << m_vehicleId 
              << (shouldAttackThisEvent ? " (ATTACKING)" : " (HONEST)")
              << "] EVENT #" << m_totalEventReportsSent 
              << " Event: " << reportContent << " vs Truth: " << eventType
              << " (Event Index: " << m_currentEventIndex << ")" << std::endl;
    // Use the assigned target RSU instead of hardcoded RSU-0
    ndn::Name eventInterestName("/vanet");
    eventInterestName.append(m_targetRsuName);
    eventInterestName.append("eventreport");
    eventInterestName.append(m_vehicleId);
    eventInterestName.appendTimestamp();

    std::shared_ptr<ndn::Interest> interest = std::make_shared<ndn::Interest>(eventInterestName);
    
    std::string payloadStr = m_vehicleId + "|" + reportContent + "|" + eventLocation + "|" +
                             std::to_string(static_cast<long long>(eventTime.GetSeconds())) + "|" +
                             std::to_string(m_eventSeqNum) + "|" + signature + "|" + eventType;


    auto buffer = std::make_shared<::ndn::Buffer>(payloadStr.begin(), payloadStr.end());
    ndn::Block appParamsBlock(::ndn::tlv::ApplicationParameters, buffer);
    interest->setApplicationParameters(appParamsBlock);

    m_eventSeqNum++;
    m_totalEventReportsSent++;

    if (m_metricsCollector) {
        uint32_t reportSize = 500; // NDN + payload overhead
        
        std::string packetType = shouldAttackThisEvent ? "Interest_EventReport_Malicious" : "Interest_EventReport_Honest";
        m_metricsCollector->RecordCommunicationOverhead(packetType, reportSize, 
                                                       m_vehicleId, m_targetRsuName, false);
        
    }

    // Enhanced communication metrics recording
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    interest->setNonce(rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
    interest->setInterestLifetime(ndn::time::seconds(10));

    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId << (m_isAttacker ? " (Attacker)" : "")
                     << "] *** EVENT REPORT #" << m_totalEventReportsSent << " *** Sending to " << m_targetRsuName 
                     << ": " << interest->getName()
                     << " Payload (Reported Event): " << reportContent
                     << " Ground Truth: " << eventType);
    
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
}


void VanetVehicleApp::OnInterest(std::shared_ptr<const ndn::Interest> interest) {
    ndn::App::OnInterest(interest); // Call base class
    NFD_LOG_DEBUG("[" << GetNode()->GetId() << ":" << m_vehicleId << "] Received Interest: " << interest->getName());
    // Vehicles typically don't serve data in this scenario, but might get ACKs or config.
}

void VanetVehicleApp::OnData(std::shared_ptr<const ndn::Data> data) {
    Time responseTime = Simulator::Now(); // Record response time immediately
    
    NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] *** VEHICLE RECEIVED SOMETHING *** at " 
                 << responseTime.GetSeconds() << "s");
    NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] Data Name: " << data->getName().toUri());
    
    // Call the base class method FIRST
    ndn::App::OnData(data);

    const std::string dataNameStr = data->getName().toUri();
    std::string contentStr = "N/A";

    if (data->getContent().value_size() > 0) {
        contentStr = std::string(reinterpret_cast<const char*>(data->getContent().value()), 
                                data->getContent().value_size());
    }

    
    if (m_metricsCollector) {
        uint32_t responseSize = 500; // NDN + content overhead
        
        std::string packetType = "Data_Unknown";
        if (dataNameStr.find("/blockchain/keys/") != std::string::npos) {
            packetType = "Data_BlockchainResponse";
        } else if (dataNameStr.find("/register/") != std::string::npos && dataNameStr.find("/ack") != std::string::npos) {
            packetType = "Data_RegistrationAck";
        } else if (dataNameStr.find("registration-confirmed") != std::string::npos) {
            packetType = "Data_RegistrationConfirmation";
        }
        
        m_metricsCollector->RecordCommunicationOverhead(packetType, responseSize, 
                                                       m_targetRsuName, m_vehicleId, false); //changed here from true
    }

    // Check for registration ACKs
    bool isAck = false;
    if (dataNameStr.find("/register/" + m_vehicleId + "/") != std::string::npos && 
        dataNameStr.find("/ack") != std::string::npos) {
        isAck = true;
    }

    std::cerr << "STRING " << dataNameStr << "\n";
    if (isAck && m_registrationPending) {
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId
                    << "] *** REGISTRATION ACK RECEIVED ***");
        
        // Check ACK content to determine actual success
        std::string ackContent = "";
        if (data->getContent().value_size() > 0) {
            ackContent = std::string(reinterpret_cast<const char*>(data->getContent().value()),
                                    data->getContent().value_size());
        }
            bool registrationSuccessful = (ackContent.find("CONFIRMED") != std::string::npos ||
                                ackContent.find("REG_CONFIRMED") != std::string::npos ||
                                ackContent.find("SUCCESS") != std::string::npos) &&
                                (ackContent.find("FAILED") == std::string::npos &&
                                ackContent.find("ERROR") == std::string::npos);

    // Record the actual success/failure
        if (m_metricsCollector) {
            bool res = (ackContent.size() > 0);
            std::cerr << "ALL TIME " << m_registrationRequestTime.GetSeconds() << " " << responseTime.GetSeconds() << "\n";
            m_metricsCollector->RecordAccessDelay(m_vehicleId, m_registrationRequestTime, responseTime,
                                                "registration", true, m_targetRsuName);
        }
        
        if (registrationSuccessful) {
            m_isRegistered = true;
            m_registrationPending = false;
            
            NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId
                        << "] Registration successful with " << m_targetRsuName);
            
            if (!m_pendingEventReports.empty()) {
                ProcessPendingEventReports();
            }
        }
    }
    // Check if it's a registration status response
    if (dataNameStr.find("/blockchain/keys/" + m_vehicleId) != std::string::npos) {
        NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] *** REGISTRATION STATUS RESPONSE *** Processing...");
        if (m_metricsCollector) {
            bool keyFound = (!contentStr.empty() && contentStr != "NOT_FOUND");
            bool res = (contentStr.size() > 0);
            std::cerr << "ALL TIME " << m_lastQueryTime.GetSeconds() << " " << responseTime.GetSeconds() << "\n";
            m_metricsCollector->RecordAccessDelay(m_vehicleId, m_lastQueryTime, responseTime,
                                                "blockchain_query", true, m_targetRsuName);
        }
        OnRegistrationStatusResponse(data);
        return;
    }
    
    if (dataNameStr.find("/location-query/" + m_vehicleId + "/") != std::string::npos) {
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] *** LOCATION QUERY RESPONSE *** Processing...");
        OnLocationQueryResponse(data);
        return;
    }
}


void VanetVehicleApp::SetMetricsCollector(Ptr<MetricsCollector> collector) {
    m_metricsCollector = collector;
}

std::string VanetVehicleApp::GetAssignedRsu() const {
    return m_targetRsuName;
}

void VanetVehicleApp::QueryRegistrationStatus() {
    m_lastQueryTime = Simulator::Now();
    
    ndn::Name queryName("/vanet");
    queryName.append(m_targetRsuName);
    queryName.append("blockchain");
    queryName.append("keys");
    queryName.append(m_vehicleId);
    queryName.appendNumber(Simulator::Now().GetTimeStep());

    auto interest = std::make_shared<ndn::Interest>(queryName);
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    interest->setNonce(rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
    interest->setInterestLifetime(ndn::time::seconds(5)); // Shorter timeout
    interest->setMustBeFresh(true);
    
    if (m_metricsCollector) {
        uint32_t querySize = 500; // Estimate
        m_metricsCollector->RecordCommunicationOverhead("Interest_BlockchainKeyQuery", querySize,
                                                       m_vehicleId, m_targetRsuName, false);
    }
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] One-time registration check to " << m_targetRsuName);
    
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
}

void VanetVehicleApp::ScheduleRegistrationRequest() {
    if (!m_isRegistered && !m_registrationPending) { 
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] *** STARTING REGISTRATION PROCESS ***");
        SendRegistrationRequest();
        
    } else {
        NFD_LOG_DEBUG("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Registration request skipped - already registered or pending");
    }
}

void VanetVehicleApp::ScheduleEventReportToSpecificRsu(std::string eventType, std::string eventLocation, 
                                                      Time eventTime, std::string targetRsu) {
    if (!m_isRegistered) {
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Not registered. Checking with RSU before reporting " << eventType 
                     << " to specific RSU: " << targetRsu);
        
        QueryRegistrationStatus();
        
        // Store the event with specific RSU target
        PendingEventReport pendingReport;
        pendingReport.eventType = eventType;
        pendingReport.eventLocation = eventLocation;
        pendingReport.eventTime = eventTime;
        pendingReport.retryCount = 0;
        pendingReport.targetRsu = targetRsu; // *** Store the specific target RSU ***
        m_pendingEventReports.push_back(pendingReport);
        
        Simulator::Schedule(Seconds(2.0), &VanetVehicleApp::ProcessPendingEventReports, this);
        return;
    }
    
    SendEventReportToSpecificRsu(eventType, eventLocation, eventTime, targetRsu);
}

void VanetVehicleApp::CheckRegistrationStatus() {
    if (m_registrationPending && !m_isRegistered) {
        Time waitTime = Simulator::Now() - m_registrationRequestTime;
        
        if (waitTime < Seconds(5)) { //50
            NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                        << "] Registration check after " << waitTime.GetSeconds() 
                        << "s - querying RSU again");
            
            // Query with exponential backoff to reduce congestion
            QueryRegistrationStatus();
            
            // Schedule next check with increasing delay
            double nextCheckDelay = std::min(30.0, 5.0 + waitTime.GetSeconds() * 0.1);
            m_registrationRetryEvent = Simulator::Schedule(Seconds(nextCheckDelay), 
                                                          &VanetVehicleApp::CheckRegistrationStatus, this);
        } else {
            NFD_LOG_ERROR("[" << GetNode()->GetId() << ":" << m_vehicleId 
                          << "] Registration failed after 300 seconds - giving up");
            m_registrationPending = false;
        }
    }
}

void VanetVehicleApp::OnRegistrationStatusResponse(std::shared_ptr<const ndn::Data> data) {
    std::string response;
    if (data->getContent().value_size() > 0) {
        response = std::string(reinterpret_cast<const char*>(data->getContent().value()),
                             data->getContent().value_size());
    }
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                << "] *** 10-RSU RESPONSE *** from " << m_targetRsuName 
                << " (attempt " << (m_currentRsuIndex + 1) << "/" << m_maxRsuRetries << ")");
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                << "] Response: '" << response << "' (length: " << response.length() << ")");
    
    // Cancel any pending fallback
    if (m_rsuFallbackEvent.IsRunning()) {
        Simulator::Cancel(m_rsuFallbackEvent);
    }
    
    bool isValidKey = (!response.empty() && 
                      response != "NOT_FOUND" && 
                      response.find("ERROR") == std::string::npos &&
                      response.find("FAIL") == std::string::npos &&
                      response.length() > 10);
    
    if (isValidKey) {
        if (!m_isRegistered) {
            NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                        << "] *** 10-RSU SUCCESS *** Registration confirmed via " << m_targetRsuName 
                        << " after " << (m_currentRsuIndex + 1) << " attempts");



            std::cerr << GetNode()->GetId() << ":" << m_vehicleId << "] *** 10-RSU SUCCESS *** Registration confirmed via " << m_targetRsuName 
                        << " after " << (m_currentRsuIndex + 1) << " attempts\n";

                        
            m_isRegistered = true;
            m_registrationPending = false;
            m_currentRsuIndex = 0; // Reset for future queries
            
            // Process pending event reports
            if (!m_pendingEventReports.empty()) {
                NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                            << "] Processing " << m_pendingEventReports.size() 
                            << " pending event reports via " << m_targetRsuName);
                
                for (const auto& report : m_pendingEventReports) {
                    SendEventReport(report.eventType, report.eventLocation, report.eventTime);
                }
                m_pendingEventReports.clear();
            }
        }
    } else {
        NFD_LOG_WARN("[" << GetNode()->GetId() << ":" << m_vehicleId 
                    << "] Invalid response from " << m_targetRsuName << ": '" << response << "'");
        // Let the fallback mechanism try the next RSU
    }
}

uint32_t VanetVehicleApp::GetTotalEventReportsSent() const {
    return m_totalEventReportsSent;
}

void VanetVehicleApp::ReportVehicleMetrics() {
}
void VanetVehicleApp::StartPeriodicMetricsReporting() {
    // Schedule periodic metrics reporting every 2 minutes
    Simulator::Schedule(Seconds(120.0), &VanetVehicleApp::PeriodicMetricsReport, this);
}

void VanetVehicleApp::PeriodicMetricsReport() {
    ReportVehicleMetrics();
    
    // Schedule next report
    Simulator::Schedule(Seconds(120.0), &VanetVehicleApp::PeriodicMetricsReport, this);
}

void VanetVehicleApp::SetBehaviorForEvent(uint32_t eventIndex, bool shouldAttack) {
    m_eventSpecificBehavior[eventIndex] = shouldAttack;
    NFD_LOG_DEBUG("[" << m_vehicleId << "] Event " << eventIndex 
                 << " behavior set: " << (shouldAttack ? "ATTACK" : "HONEST"));
}

bool VanetVehicleApp::ShouldAttackForEvent(uint32_t eventIndex) const {
    auto it = m_eventSpecificBehavior.find(eventIndex);
    if (it != m_eventSpecificBehavior.end()) {
        return it->second;  // Use event-specific behavior
    }
    return m_isAttacker;  // Fallback to base behavior
}
void VanetVehicleApp::SetCurrentEventIndex(uint32_t eventIndex) {
    m_currentEventIndex = eventIndex;
}

void VanetVehicleApp::ScheduleLocationQuery(const std::string& queryLocation, const std::string& targetRsu) {
    if (!m_isRegistered) {
        NFD_LOG_WARN("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Cannot query location - not registered yet");
        return;
    }
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] Scheduling location query for " << queryLocation 
                 << " to " << targetRsu);
    
    SendLocationQuery(queryLocation, targetRsu);
}

void VanetVehicleApp::SendLocationQuery(const std::string& queryLocation, const std::string& targetRsu) {
    Time requestTime = Simulator::Now();
    m_locationQueryCount++;
    
    // Create unique query ID
    std::string queryId = m_vehicleId + "_LOC_" + std::to_string(m_locationQueryCount) + "_" + queryLocation;
    
    ndn::Name queryInterestName("/vanet");
    queryInterestName.append(targetRsu);
    queryInterestName.append("location-query");
    queryInterestName.append(m_vehicleId);
    queryInterestName.append(queryLocation);
    queryInterestName.appendNumber(requestTime.GetTimeStep());

    std::string interestNameStr = queryInterestName.toUri();
    m_queryRequestTimes[interestNameStr] = requestTime;

    auto interest = std::make_shared<ndn::Interest>(queryInterestName);
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    interest->setNonce(rand->GetValue(0, std::numeric_limits<uint32_t>::max()));
    interest->setInterestLifetime(ndn::time::seconds(15));
    interest->setMustBeFresh(true);

    // Store request time for delay calculation
    m_pendingLocationQueries[queryId] = requestTime;

    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] *** LOCATION QUERY *** Requesting info about " << queryLocation 
                 << " from " << targetRsu << " at " << requestTime.GetSeconds() << "s");

    if (m_metricsCollector) {
        uint32_t querySize = 500; // NDN overhead + location string
        m_metricsCollector->RecordCommunicationOverhead("Interest_LocationQuery", querySize, 
                                                       m_vehicleId, targetRsu, false);
    }

    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
}

void VanetVehicleApp::OnLocationQueryResponse(std::shared_ptr<const ndn::Data> data) {
    Time responseTime = Simulator::Now();
    const std::string dataNameStr = data->getName().toUri();
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] *** LOCATION QUERY RESPONSE *** received at " 
                 << responseTime.GetSeconds() << "s");

    // Extract query information from the data name
    std::string queryLocation = "Unknown";
    std::string targetRsu = "Unknown";
    
    // Parse: /vanet/{RSU}/location-query/{vehicleId}/{location}/{timestamp}
    auto name = data->getName();
    if (name.size() >= 5) {
        targetRsu = name.get(1).toUri();
        queryLocation = name.get(4).toUri();
    }

    // Find corresponding request time
    Time requestTime = responseTime; // Default fallback
    std::string queryId = m_vehicleId + "_LOC_" + std::to_string(m_locationQueryCount) + "_" + queryLocation;
    
    auto it = m_pendingLocationQueries.find(queryId);
    if (it != m_pendingLocationQueries.end()) {
        requestTime = it->second;
        m_pendingLocationQueries.erase(it);
    } else {
        // Try to find any pending query for this location
        for (auto& pair : m_pendingLocationQueries) {
            if (pair.first.find(queryLocation) != std::string::npos) {
                requestTime = pair.second;
                m_pendingLocationQueries.erase(pair.first);
                break;
            }
        }
    }

    // Parse response content
    std::string responseContent = "NO_DATA";
    if (data->getContent().value_size() > 0) {
        responseContent = std::string(reinterpret_cast<const char*>(data->getContent().value()),
                                    data->getContent().value_size());
    }

    bool querySuccessful = (responseContent != "NO_DATA" && 
                           responseContent != "NOT_FOUND" && 
                           responseContent.find("ERROR") == std::string::npos &&
                           responseContent.length() > 10);
    
    NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                 << "] Location query result for " << queryLocation 
                 << ": " << (querySuccessful ? "SUCCESS" : "NO_DATA") 
                 << " (Response length: " << responseContent.length() << " chars)");

    // Record access delay in metrics collector
    if (m_metricsCollector) {
        Time delay = responseTime - requestTime;
        bool res = (responseContent.size() > 0);
        std::cerr << "ALL TIME " << requestTime.GetSeconds() << " " << responseTime.GetSeconds() << "\n";
        m_metricsCollector->RecordAccessDelay(
            m_vehicleId,           // vehicleId
            requestTime,           // requestTime
            responseTime,          // responseTime
            "location_query",      // requestType
            true,       // successful
            targetRsu             // targetRsu
        );
        
        // Record communication overhead for response
        uint32_t responseSize = 500 + responseContent.length();
        m_metricsCollector->RecordCommunicationOverhead("Data_LocationQueryResponse", responseSize, 
                                                       targetRsu, m_vehicleId, false);
        
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Location query delay recorded: " << delay.GetMilliSeconds() 
                     << "ms, success: " << (querySuccessful ? "YES" : "NO"));
    }

    // Log detailed response for debugging (truncated for readability)
    if (querySuccessful && responseContent.length() > 100) {
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Response preview: " << responseContent.substr(0, 100) << "...");
    } else {
        NFD_LOG_INFO("[" << GetNode()->GetId() << ":" << m_vehicleId 
                     << "] Full response: " << responseContent);
    }
}


} // namespace ns3