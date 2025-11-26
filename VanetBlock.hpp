#ifndef VANET_BLOCK_HPP
#define VANET_BLOCK_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <sstream>
#include <iomanip>
#include <openssl/sha.h> // Make sure to link against -lcrypto

namespace ns3 {

/**
 * @brief Defines the types of transactions that can be stored on the VANET blockchain.
 */
enum TransactionType {
    REGISTRATION,       // Records a new vehicle's public key and initial reputation.
    EVENT_DECISION,     // Records the consensus decision about a reported event.
    REPUTATION_UPDATE   // Records a change in a vehicle's reputation score.
};

/**
 * @brief Represents a single transaction on the VANET blockchain.
 * It's a unified structure; only fields relevant to its 'type' will be populated.
 */
struct Transaction {
    TransactionType type;
    uint64_t timestamp = 0; // Common to all transaction types

    // --- Fields specific to REGISTRATION ---
    std::string vehicleId_reg;     // The unique ID (e.g., license plate) of the vehicle
    std::string publicKey;         // The public key of the vehicle
    double initialReputation = 0.5; // The starting reputation score

    // --- Fields specific to EVENT_DECISION ---
    std::string eventId_dec;       // A unique identifier for the specific event instance
    std::string eventType;         // E.g., "Accident", "No Accident", "Jam"
    std::string eventLocation;     // GPS coordinates or road segment ID
    uint64_t eventTimestamp = 0;   // Timestamp when the event likely occurred
    // Stores pairs of (VehicleID, ReportContent/Vote)
    std::vector<std::pair<std::string, std::string>> eventReports;
    std::string eventVerdict;      // The consensus result: "True", "False", "Uncertain"
    double eventCredibility = 0.0; // The calculated C_event score

    // --- Fields specific to REPUTATION_UPDATE ---
    std::string vehicleId_rep;     // The ID of the vehicle whose reputation is updated
    std::string eventId_rep;       // The EventID that triggered this update (for traceability)
    double oldReputation = 0.0;    // Reputation before the update
    double newReputation = 0.0;    // Reputation after the update
    std::string serialize() const;
};

/**
 * @brief Represents a single block in the VANET blockchain.
 */
struct VanetBlock {
    uint64_t height = 0;
    uint64_t timestamp = 0;
    std::string previousHash = "0";
    std::string blockHash = "";
    std::string proposerId = ""; // RSU ID that proposed this block
    std::vector<Transaction> transactions; // List of transactions in this block

    // Stores pairs of (RSU_ID, Signature) for PBFT consensus proof
    std::vector<std::pair<std::string, std::string>> consensusSignatures;

    /**
     * @brief Calculates the SHA256 hash of the block's content.
     * @return The SHA256 hash as a hex string.
     */
    std::string calculateHash() const {
        std::stringstream ss;
        ss << height << timestamp << previousHash << proposerId;
        for (const auto& tx : transactions) {
            ss << tx.serialize();
        }

        unsigned char hash[SHA256_DIGEST_LENGTH];
        SHA256((unsigned char*)ss.str().c_str(), ss.str().size(), hash);

        std::stringstream hashStream;
        for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
            hashStream << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
        }
        return hashStream.str();
    }
};

} // namespace ns3
#endif // VANET_BLOCK_HPP