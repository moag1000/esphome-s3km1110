/**
 * @file gateway_buffer_sizes.h
 * @brief Gateway Buffer Size Constants
 *
 * Centralized definitions for buffer sizes used throughout the gateway.
 * These constants replace magic numbers for buffer allocations.
 *
 * Categories:
 * - GW_BUF_SIZE_*        : Generic power-of-2 buffer sizes
 * - GW_BUF_JSON_*        : JSON document buffer sizes
 * - GW_BUF_TOPIC_*       : MQTT topic string buffers
 * - GW_BUF_{PURPOSE}     : Purpose-specific buffer sizes
 * - GW_BUFFER_SIZE_*     : Legacy aliases for backwards compatibility
 *
 * Related headers:
 * - gateway_defaults.h  - Task priorities, unit conversions
 * - gateway_timeouts.h  - Timeout and delay constants
 * - gateway_config.h    - Umbrella header including all
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef GATEWAY_BUFFER_SIZES_H
#define GATEWAY_BUFFER_SIZES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Generic Buffer Sizes (power of 2)
 * ============================================================================
 * Use these for general-purpose buffer allocations where no specific
 * semantic meaning is required.
 * ============================================================================ */

/** @brief 32-byte buffer (very short strings, error codes) */
#define GW_BUF_SIZE_32                      32

/** @brief 64-byte buffer (short identifiers, names) */
#define GW_BUF_SIZE_64                      64

/** @brief 128-byte buffer (medium strings, short topics) */
#define GW_BUF_SIZE_128                     128

/** @brief 256-byte buffer (longer strings, full topics) */
#define GW_BUF_SIZE_256                     256

/** @brief 512-byte buffer (small JSON payloads) */
#define GW_BUF_SIZE_512                     512

/** @brief 1024-byte buffer (1KB, medium JSON payloads) */
#define GW_BUF_SIZE_1024                    1024

/** @brief 2048-byte buffer (2KB, large JSON payloads) */
#define GW_BUF_SIZE_2048                    2048

/** @brief 4096-byte buffer (4KB, very large payloads) */
#define GW_BUF_SIZE_4096                    4096

/* ============================================================================
 * JSON Buffer Sizes
 * ============================================================================
 * Specific sizes for JSON document handling.
 * ============================================================================ */

/** @brief Small JSON buffer (simple state messages) */
#define GW_BUF_JSON_SMALL                   256

/** @brief Medium JSON buffer (standard device state) */
#define GW_BUF_JSON_MEDIUM                  512

/** @brief Large JSON buffer (complex device state, discovery) */
#define GW_BUF_JSON_LARGE                   1024

/** @brief Extra large JSON buffer (HA discovery, config) */
#define GW_BUF_JSON_XLARGE                  2048

/** @brief Maximum JSON buffer (backup data, bulk operations) */
#define GW_BUF_JSON_MAX                     4096

/* ============================================================================
 * String Buffer Sizes
 * ============================================================================
 * Specific sizes for string formatting and storage.
 * ============================================================================ */

/** @brief MAC address string buffer (XX:XX:XX:XX:XX:XX:XX:XX\0) */
#define GW_BUF_MAC_STRING                   18

/** @brief IEEE address string buffer (0x + 16 hex + \0) */
#define GW_BUF_IEEE_STRING                  19

/** @brief Short topic buffer (base topics, simple paths) */
#define GW_BUF_TOPIC_SHORT                  64

/** @brief Standard topic buffer (most MQTT topics) */
#define GW_BUF_TOPIC_STANDARD               128

/** @brief Long topic buffer (full paths with device IDs) */
#define GW_BUF_TOPIC_LONG                   192

/** @brief Maximum topic buffer (complex nested topics) */
#define GW_BUF_TOPIC_MAX                    256

/* ============================================================================
 * Device and Network Buffer Sizes
 * ============================================================================ */

/** @brief Device friendly name buffer */
#define GW_BUF_FRIENDLY_NAME                64

/** @brief Device model identifier buffer */
#define GW_BUF_MODEL_ID                     64

/** @brief Manufacturer name buffer */
#define GW_BUF_MANUFACTURER                 64

/** @brief Endpoint description buffer */
#define GW_BUF_ENDPOINT_DESC                32

/** @brief Cluster name buffer */
#define GW_BUF_CLUSTER_NAME                 48

/** @brief Attribute name buffer */
#define GW_BUF_ATTRIBUTE_NAME               48

/* ============================================================================
 * Command and Response Buffer Sizes
 * ============================================================================ */

/** @brief Command name buffer */
#define GW_BUF_COMMAND_NAME                 32

/** @brief Command payload buffer */
#define GW_BUF_COMMAND_PAYLOAD              512

/** @brief Response message buffer */
#define GW_BUF_RESPONSE_MSG                 256

/** @brief Error message buffer */
#define GW_BUF_ERROR_MSG                    128

/* ============================================================================
 * Log and Debug Buffer Sizes
 * ============================================================================ */

/** @brief Log message buffer */
#define GW_BUF_LOG_MESSAGE                  256

/** @brief Debug output buffer */
#define GW_BUF_DEBUG_OUTPUT                 512

/** @brief Hex dump line buffer */
#define GW_BUF_HEX_DUMP_LINE                80

/* ============================================================================
 * Legacy Aliases (backwards compatibility with gateway_defaults.h)
 * ============================================================================
 * These aliases maintain compatibility with code that used the older
 * GW_BUFFER_SIZE_* naming convention from gateway_defaults.h.
 * For new code, prefer the GW_BUF_* naming convention above.
 * ============================================================================ */

/** @brief Tiny buffer (32 bytes) - legacy alias */
#define GW_BUFFER_SIZE_TINY                 GW_BUF_SIZE_32

/** @brief Small buffer (64 bytes) - legacy alias */
#define GW_BUFFER_SIZE_SMALL                GW_BUF_SIZE_64

/** @brief Medium buffer (128 bytes) - legacy alias */
#define GW_BUFFER_SIZE_MEDIUM               GW_BUF_SIZE_128

/** @brief Large buffer (256 bytes) - legacy alias */
#define GW_BUFFER_SIZE_LARGE                GW_BUF_SIZE_256

/** @brief Extra large buffer (512 bytes) - legacy alias */
#define GW_BUFFER_SIZE_XLARGE               GW_BUF_SIZE_512

/** @brief Huge buffer (1024 bytes) - legacy alias */
#define GW_BUFFER_SIZE_HUGE                 GW_BUF_SIZE_1024

/** @brief Maximum buffer (2048 bytes) - legacy alias */
#define GW_BUFFER_SIZE_MAX                  GW_BUF_SIZE_2048

/** @brief JSON document buffer size - legacy alias */
#define GW_JSON_BUFFER_SIZE                 GW_BUF_JSON_MEDIUM

/** @brief IP address string buffer size (IPv4) - legacy alias */
#define GW_IP_STRING_SIZE                   16

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_BUFFER_SIZES_H */
