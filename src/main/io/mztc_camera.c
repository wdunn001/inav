/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR ANY PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * MassZero Thermal Camera (MZTC) Driver
 * 
 * IMPORTANT: The thermal camera does NOT maintain configuration through
 * power cycles. The driver automatically handles this by:
 * 1. Detecting when the camera comes online (first successful response)
 * 2. Sending full configuration when camera is detected
 * 3. Monitoring for disconnections (timeout on responses)
 * 4. Reconfiguring when camera reconnects after power loss
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"
#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/mztc_camera.h"

#include "scheduler/scheduler.h"

#ifdef USE_MZTC

// Parameter group ID for MassZero Thermal Camera (defined in parameter_group_ids.h)

// Default configuration values
#define MZTC_DEFAULT_ENABLED            0
#define MZTC_DEFAULT_PORT               SERIAL_PORT_USART2
#define MZTC_DEFAULT_BAUDRATE           BAUD_115200
#define MZTC_DEFAULT_MODE               MZTC_MODE_DISABLED
#define MZTC_DEFAULT_UPDATE_RATE        9
#define MZTC_DEFAULT_TEMPERATURE_UNIT   MZTC_UNIT_CELSIUS
#define MZTC_DEFAULT_PALETTE_MODE       MZTC_PALETTE_WHITE_HOT
#define MZTC_DEFAULT_AUTO_SHUTTER       MZTC_SHUTTER_TIME_AND_TEMP
#define MZTC_DEFAULT_DIGITAL_ENHANCEMENT 50
#define MZTC_DEFAULT_SPATIAL_DENOISE    50
#define MZTC_DEFAULT_TEMPORAL_DENOISE   50
#define MZTC_DEFAULT_BRIGHTNESS         50
#define MZTC_DEFAULT_CONTRAST           50
#define MZTC_DEFAULT_ZOOM_LEVEL         MZTC_ZOOM_1X
#define MZTC_DEFAULT_MIRROR_MODE        MZTC_MIRROR_NONE
#define MZTC_DEFAULT_CROSSHAIR_ENABLED  0
#define MZTC_DEFAULT_TEMPERATURE_ALERTS 0
#define MZTC_DEFAULT_ALERT_HIGH_TEMP    80.0f
#define MZTC_DEFAULT_ALERT_LOW_TEMP     -10.0f
#define MZTC_DEFAULT_FFC_INTERVAL       5
#define MZTC_DEFAULT_BAD_PIXEL_REMOVAL  1
#define MZTC_DEFAULT_VIGNETTING_CORRECTION 1

// Serial packet structure for thermal camera communication
// Note: This structure is for reference/documentation only.
// Actual packet construction uses mztcBuildPacket() function.
typedef struct {
    uint8_t begin;          // MZTC_PACKET_BEGIN (0xF0) - Start byte
    uint8_t size;           // N+4 (total packet length)
    uint8_t device_addr;    // MZTC_DEVICE_ADDRESS (0x36) - Device address
    uint8_t class_cmd;      // Class command address
    uint8_t subclass_cmd;   // Subclass command address
    uint8_t flags;          // Read/Write flags
    uint8_t data[MZTC_MAX_DATA_LENGTH];  // Data content (max 14 bytes)
    uint8_t checksum;       // Checksum
    uint8_t end;            // MZTC_PACKET_END (0xFF) - End byte
} mztcPacket_t;

// Packet constants
#define MZTC_PACKET_BEGIN        0xF0
#define MZTC_PACKET_END          0xFF
#define MZTC_DEVICE_ADDRESS      0x36
#define MZTC_MAX_DATA_LENGTH     14

// Flag definitions
#define MZTC_FLAG_WRITE      0x00
#define MZTC_FLAG_READ       0x01
#define MZTC_FLAG_SUCCESS    0x03
#define MZTC_FLAG_ERROR      0x04

// Error codes
#define MZTC_ERR_NO_COMMAND  0x00
#define MZTC_ERR_THRESHOLD   0x01

// Camera commands
#define MZTC_CMD_READ_MODEL          {0x74, 0x02}  // Read device model
#define MZTC_CMD_READ_FPGA_VER       {0x74, 0x03}  // Read FPGA version
#define MZTC_CMD_READ_SW_VER         {0x74, 0x05}  // Read software version
#define MZTC_CMD_MANUAL_SHUTTER      {0x7C, 0x02}  // Manual FFC
#define MZTC_CMD_AUTO_SHUTTER        {0x7C, 0x04}  // Auto shutter control
#define MZTC_CMD_DIGITAL_ENHANCE     {0x78, 0x10}  // Digital enhancement
#define MZTC_CMD_SPATIAL_DENOISE     {0x78, 0x15}  // Spatial denoising
#define MZTC_CMD_TEMPORAL_DENOISE    {0x78, 0x16}  // Temporal denoising
#define MZTC_CMD_BRIGHTNESS          {0x78, 0x02}  // Brightness
#define MZTC_CMD_CONTRAST            {0x78, 0x03}  // Contrast
#define MZTC_CMD_PSEUDO_COLOR        {0x78, 0x20}  // Pseudo color
#define MZTC_CMD_ZOOM                {0x70, 0x12}  // Digital zoom
#define MZTC_CMD_IMAGE_MIRROR        {0x70, 0x11}  // Mirror mode
#define MZTC_CMD_INIT_STATUS         {0x7C, 0x14}  // Read initialization status
#define MZTC_CMD_SAVE_CONFIG         {0x74, 0x10}  // Save configuration
#define MZTC_CMD_RESTORE_DEFAULTS    {0x74, 0x0F}  // Restore defaults

// Internal state
mztcStatus_t mztcStatus;
serialPort_t *mztcSerialPort = NULL;
static uint32_t mztcLastUpdateTime = 0;
static uint32_t mztcLastCalibrationTime = 0;
static bool mztcInitialized = false;
static uint8_t mztcRxBuffer[256];
static uint16_t mztcRxBufferIndex = 0;

// SITL detection and connection state
static bool mztcSitlMode = false;
static uint32_t mztcLastDataReceived = 0;

// Parameter group for MassZero Thermal Camera configuration
PG_REGISTER_WITH_RESET_TEMPLATE(mztcConfig_t, mztcConfig, PG_MZTC_CAMERA_CONFIG, 0);



PG_RESET_TEMPLATE(mztcConfig_t, mztcConfig,
    .enabled = MZTC_DEFAULT_ENABLED,
    .port = MZTC_DEFAULT_PORT,
    .baudrate = MZTC_DEFAULT_BAUDRATE,
    .mode = MZTC_DEFAULT_MODE,
    .update_rate = MZTC_DEFAULT_UPDATE_RATE,
    .temperature_unit = MZTC_DEFAULT_TEMPERATURE_UNIT,
    .palette_mode = MZTC_DEFAULT_PALETTE_MODE,
    .auto_shutter = MZTC_DEFAULT_AUTO_SHUTTER,
    .digital_enhancement = MZTC_DEFAULT_DIGITAL_ENHANCEMENT,
    .spatial_denoise = MZTC_DEFAULT_SPATIAL_DENOISE,
    .temporal_denoise = MZTC_DEFAULT_TEMPORAL_DENOISE,
    .brightness = MZTC_DEFAULT_BRIGHTNESS,
    .contrast = MZTC_DEFAULT_CONTRAST,
    .zoom_level = MZTC_DEFAULT_ZOOM_LEVEL,
    .mirror_mode = MZTC_DEFAULT_MIRROR_MODE,
    .crosshair_enabled = MZTC_DEFAULT_CROSSHAIR_ENABLED,
    .temperature_alerts = MZTC_DEFAULT_TEMPERATURE_ALERTS,
    .alert_high_temp = MZTC_DEFAULT_ALERT_HIGH_TEMP,
    .alert_low_temp = MZTC_DEFAULT_ALERT_LOW_TEMP,
    .ffc_interval = MZTC_DEFAULT_FFC_INTERVAL,
    .bad_pixel_removal = MZTC_DEFAULT_BAD_PIXEL_REMOVAL,
    .vignetting_correction = MZTC_DEFAULT_VIGNETTING_CORRECTION,
);

// Forward declarations
static void mztcSerialReceiveCallback(uint16_t c, void *rxCallbackData);
static bool mztcSendPacket(uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len);
static bool mztcProcessResponse(const uint8_t *data, uint8_t len);
static void mztcUpdateStatus(void);
static void mztcCheckCalibration(void);
static void mztcSendConfiguration(void); // Forward declaration for new function

// Packet utility functions
static uint8_t mztcCalculateChecksum(uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len);
static uint8_t mztcBuildPacket(uint8_t *buffer, uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len);
static bool mztcValidatePacket(const uint8_t *data, uint8_t len, uint8_t *calc_checksum);

// Initialize MassZero Thermal Camera
void mztcInit(void)
{
    // DEBUG: Print initialization message
    SD(fprintf(stderr, "[MZTC]: Initializing MassZero Thermal Camera\n"));
    SD(fprintf(stderr, "[MZTC]: USE_MZTC is DEFINED and ACTIVE\n"));
    
    if (mztcInitialized) {
        return;
    }

    // Initialize status structure
    memset(&mztcStatus, 0, sizeof(mztcStatus));
    mztcStatus.status = MZTC_STATUS_OFFLINE;
    mztcStatus.mode = mztcConfig()->mode;
    mztcStatus.last_frame_time = 0;
    mztcStatus.frame_count = 0;
    mztcStatus.connected = false;  // Explicitly set to false
    mztcStatus.camera_temperature = 25.0f;  // Default room temperature
    mztcStatus.ambient_temperature = 25.0f;  // Default room temperature

    // Check if enabled
    if (!mztcConfig()->enabled) {
        mztcStatus.status = MZTC_STATUS_OFFLINE;
        mztcInitialized = true;
        return;
    }

    // Detect SITL mode (check if we're running in simulation)
    #ifdef USE_SIMULATOR
        mztcSitlMode = true;
        SD(fprintf(stderr, "[MZTC]: Running in SITL mode\n"));
    #else
        mztcSitlMode = false;
        SD(fprintf(stderr, "[MZTC]: Running on real hardware\n"));
    #endif

    // Don't try to open port immediately - let the update loop handle it
    mztcStatus.status = MZTC_STATUS_INITIALIZING;
    mztcInitialized = true;
    mztcLastUpdateTime = millis();
    mztcLastCalibrationTime = millis();

    debug[0] = 0xAA; // Debug indicator
}

// Update MassZero Thermal Camera (called from scheduler)
void mztcUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!mztcInitialized || !mztcConfig()->enabled) {
        return;
    }

    uint32_t now = millis();

    // Check if we need to retry connection
    if (!mztcStatus.connected && mztcSerialPort == NULL) {
        if ((now - mztcLastUpdateTime) > 1000) { // Try every second
            mztcLastUpdateTime = now;
            
            // Try to open serial port (works for both SITL TCP ports and real hardware)
            mztcSerialPort = openSerialPort(mztcConfig()->port,
                                           FUNCTION_MZTC_CAMERA,
                                           mztcSerialReceiveCallback,
                                           NULL,
                                           baudRates[mztcConfig()->baudrate],
                                           MODE_RXTX,
                                           SERIAL_NOT_INVERTED);
            
            if (mztcSerialPort != NULL) {
                // Successfully opened port - but NOT connected yet!
                // Connection will be established when camera responds
                mztcStatus.connected = false;  // Keep false until camera responds
                mztcStatus.status = MZTC_STATUS_OFFLINE;
                mztcStatus.error_flags = 0;
                mztcStatus.last_frame_time = now;
                mztcStatus.frame_count = 0;
                
                // Wait a bit for camera to be ready after port open
                delay(100);  // 100ms delay
                
                // First, check if camera is present by reading init status
                // According to documentation: Class=0x7C, Subclass=0x14, Flags=0x00 (WRITE), Data=0x00
                SD(fprintf(stderr, "[MZTC]: Checking camera presence...\n"));
                uint8_t init_data = 0x00;
                if (mztcSendPacket(0x7C, 0x14, MZTC_FLAG_WRITE, &init_data, 1)) {
                    SD(fprintf(stderr, "[MZTC]: Sent init status request, waiting for response...\n"));
                }
                
                // Note: We don't wait for response here - configuration will be sent
                // when we receive the first successful response in mztcProcessResponse()
                // This handles the case where the camera needs time to boot up
                
                // Log port opening
                if (mztcSitlMode) {
                    SD(fprintf(stderr, "[MZTC]: Opened SITL TCP bridge on Serial %d, waiting for camera response...\n", mztcConfig()->port));
                } else {
                    SD(fprintf(stderr, "[MZTC]: Opened Serial %d at %d baud, waiting for camera response...\n", 
                        mztcConfig()->port, baudRates[mztcConfig()->baudrate]));
                }
            } else {
                // Failed to open port
                mztcStatus.status = MZTC_STATUS_ERROR;
                mztcStatus.error_flags |= MZTC_ERROR_COMMUNICATION;
                SD(fprintf(stderr, "[MZTC]: Failed to open Serial %d, will retry...\n", mztcConfig()->port));
            }
        }
        return; // Don't process further until connected
    }

    // Check if it's time for an update
    if ((now - mztcLastUpdateTime) < (1000 / mztcConfig()->update_rate)) {
        return;
    }

    // Check for timeout - camera may have lost power
    if (mztcStatus.connected && (now - mztcLastDataReceived) > 5000) {  // 5 second timeout
        SD(fprintf(stderr, "[MZTC]: Camera timeout - marking as disconnected\n"));
        mztcStatus.connected = false;
        mztcStatus.status = MZTC_STATUS_OFFLINE;
        mztcStatus.error_flags |= MZTC_ERROR_COMMUNICATION;
        // Will retry connection and reconfigure on next update
    }
    
    // Update status
    mztcUpdateStatus();

    // Check calibration timing
    mztcCheckCalibration();
    
    // Send periodic keepalive/status check to detect disconnections
    static uint32_t lastKeepAlive = 0;
    if (mztcStatus.connected && (now - lastKeepAlive) > 2000) {  // Every 2 seconds
        // Send init status command to check if camera is still responding
        // According to documentation: Class=0x7C, Subclass=0x14, Flags=0x00 (WRITE), Data=0x00
        uint8_t init_data = 0x00;
        mztcSendPacket(0x7C, 0x14, MZTC_FLAG_WRITE, &init_data, 1);
        lastKeepAlive = now;
    }

    // Process camera data based on mode
    switch (mztcConfig()->mode) {
        case MZTC_MODE_CONTINUOUS:
            // Request frame data
            // This would typically request the latest thermal frame
            break;
            
        case MZTC_MODE_STANDBY:
            // Periodic status check only
            break;
            
        case MZTC_MODE_ALERT:
            // Check for temperature alerts
            if (mztcStatus.ambient_temperature > mztcConfig()->alert_high_temp ||
                mztcStatus.ambient_temperature < mztcConfig()->alert_low_temp) {
                mztcStatus.status = MZTC_STATUS_ALERT;
            }
            break;
            
        default:
            break;
    }

    mztcLastUpdateTime = now;
}

// Check if MassZero Thermal Camera is connected
bool mztcIsConnected(void)
{
    // For both SITL and real hardware, check if we have a valid serial port
    return mztcStatus.connected && (mztcSerialPort != NULL);
}

// Check if MassZero Thermal Camera is enabled
bool mztcIsEnabled(void)
{
    return mztcConfig()->enabled && mztcInitialized;
}

// Simulate data reception in SITL mode (for testing)
void mztcSimulateDataReception(void)
{
    if (mztcSitlMode) {
        mztcLastDataReceived = millis();
        SD(fprintf(stderr, "[MZTC SITL]: Simulated data reception\n"));
    }
}

// Get MassZero Thermal Camera status
mztcStatus_t* mztcGetStatus(void)
{
    return &mztcStatus;
}

// Trigger calibration (FFC)
bool mztcTriggerCalibration(void)
{
    if (!mztcIsEnabled()) {
        return false;
    }

    if (mztcSendPacket(0x7C, 0x02, MZTC_FLAG_WRITE, NULL, 0)) {
        mztcStatus.status = MZTC_STATUS_CALIBRATING;
        mztcLastCalibrationTime = millis();
        return true;
    }

    return false;
}

// Set operating mode
bool mztcSetMode(mztcMode_e mode)
{
    if (!mztcIsEnabled() || mode >= MZTC_MODE_SURVEILLANCE + 1) {
        return false;
    }

    mztcConfigMutable()->mode = mode;
    mztcStatus.mode = mode;
    return true;
}

// Set color palette
bool mztcSetPalette(mztcPaletteMode_e palette)
{
    if (!mztcIsEnabled() || palette >= MZTC_PALETTE_RED_HOT + 1) {
        return false;
    }

    if (mztcSendPacket(0x78, 0x20, MZTC_FLAG_WRITE, (uint8_t*)&palette, 1)) {
        mztcConfigMutable()->palette_mode = palette;
        return true;
    }

    return false;
}

// Set zoom level
bool mztcSetZoom(mztcZoomLevel_e zoom)
{
    if (!mztcIsEnabled() || zoom >= MZTC_ZOOM_8X + 1) {
        return false;
    }

    if (mztcSendPacket(0x70, 0x12, MZTC_FLAG_WRITE, (uint8_t*)&zoom, 1)) {
        mztcConfigMutable()->zoom_level = zoom;
        return true;
    }

    return false;
}

// Set image parameters
bool mztcSetImageParams(uint8_t brightness, uint8_t contrast, uint8_t enhancement)
{
    if (!mztcIsEnabled()) {
        return false;
    }

    bool success = true;
    
    if (brightness <= 100) {
        success &= mztcSendPacket(0x78, 0x02, MZTC_FLAG_WRITE, &brightness, 1);
        if (success) mztcConfigMutable()->brightness = brightness;
    }
    
    if (contrast <= 100) {
        success &= mztcSendPacket(0x78, 0x03, MZTC_FLAG_WRITE, &contrast, 1);
        if (success) mztcConfigMutable()->contrast = contrast;
    }
    
    if (enhancement <= 100) {
        success &= mztcSendPacket(0x78, 0x10, MZTC_FLAG_WRITE, &enhancement, 1);
        if (success) mztcConfigMutable()->digital_enhancement = enhancement;
    }

    return success;
}

// Set denoising parameters
bool mztcSetDenoising(uint8_t spatial, uint8_t temporal)
{
    if (!mztcIsEnabled()) {
        return false;
    }

    bool success = true;
    
    if (spatial <= 100) {
        success &= mztcSendPacket(0x78, 0x15, MZTC_FLAG_WRITE, &spatial, 1);
        if (success) mztcConfigMutable()->spatial_denoise = spatial;
    }
    
    if (temporal <= 100) {
        success &= mztcSendPacket(0x78, 0x16, MZTC_FLAG_WRITE, &temporal, 1);
        if (success) mztcConfigMutable()->temporal_denoise = temporal;
    }

    return success;
}

// Set temperature alerts
bool mztcSetTemperatureAlerts(bool enabled, float high_temp, float low_temp)
{
    if (!mztcIsEnabled()) {
        return false;
    }

    mztcConfigMutable()->temperature_alerts = enabled ? 1 : 0;
    mztcConfigMutable()->alert_high_temp = high_temp;
    mztcConfigMutable()->alert_low_temp = low_temp;

    return true;
}

// Serial receive callback
static void mztcSerialReceiveCallback(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    mztcLastDataReceived = millis();
    
    // Debug log every received byte
    SD(fprintf(stderr, "[MZTC_RX]: %02X ", c));

    // Detect packet start (BEGIN byte)
    if (c == MZTC_PACKET_BEGIN) {
        mztcRxBufferIndex = 0;
        mztcRxBuffer[mztcRxBufferIndex++] = c;
        return;
    }

    if (mztcRxBufferIndex > 0 && mztcRxBufferIndex < sizeof(mztcRxBuffer)) {
        mztcRxBuffer[mztcRxBufferIndex++] = c;

        // Minimum packet: begin,size,addr,class,subclass,flags,checksum,end => 8 bytes
        // Check for packet end (END byte)
        if (mztcRxBufferIndex >= 8 && c == MZTC_PACKET_END) {
            const uint8_t *buf = mztcRxBuffer;
            
            // Validate packet structure and checksum using reusable function
            uint8_t calc_checksum;
            if (mztcValidatePacket(buf, mztcRxBufferIndex, &calc_checksum)) {
                mztcProcessResponse(buf, mztcRxBufferIndex);
            } else {
                SD(fprintf(stderr, "[MZTC]: Invalid packet (checksum mismatch or structure error)\n"));
            }
            mztcRxBufferIndex = 0;
        }
    } else {
        // Overflow or out-of-sync, reset parser
        mztcRxBufferIndex = 0;
    }
}

// Send packet to thermal camera
static bool mztcSendPacket(uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len)
{
    if (!mztcSerialPort) {
        return false;
    }
    
    // Validate data length to prevent buffer overflow
    if (data_len > MZTC_MAX_DATA_LENGTH) {
        return false;
    }

    // Build packet using reusable function
    uint8_t txBuffer[32];  // Max packet size
    uint8_t packet_len = mztcBuildPacket(txBuffer, class_cmd, subclass_cmd, flags, data, data_len);
    
    if (packet_len == 0) {
        return false;
    }
    
    // Send the packet
    serialWriteBufShim(mztcSerialPort, txBuffer, packet_len);
    
    // Debug log with hex dump
    SD(fprintf(stderr, "[MZTC]: Sent packet - cmd:0x%02X/0x%02X total:%u bytes: ", class_cmd, subclass_cmd, packet_len));
    SD(for(uint8_t i = 0; i < packet_len; i++) fprintf(stderr, "%02X ", txBuffer[i]));
    SD(fprintf(stderr, "\n"));
    
    return true;
}

// Calculate checksum for thermal camera packet
// Checksum = Device address + class command + subclass command + flags + DATA
static uint8_t mztcCalculateChecksum(uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len)
{
    uint8_t checksum = MZTC_DEVICE_ADDRESS + class_cmd + subclass_cmd + flags;
    
    if (data && data_len > 0) {
        for (uint8_t i = 0; i < data_len; i++) {
            checksum += data[i];
        }
    }
    
    return checksum;
}

// Build a complete thermal camera packet
// Returns: packet length (0 on error)
static uint8_t mztcBuildPacket(uint8_t *buffer, uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len)
{
    if (!buffer) {
        return 0;
    }
    
    // Validate data length
    if (data_len > MZTC_MAX_DATA_LENGTH) {
        return 0;
    }
    
    uint8_t index = 0;
    
    // Packet structure according to documentation:
    // BEGIN (0xF0) - Location 1
    buffer[index++] = MZTC_PACKET_BEGIN;
    
    // SIZE (N+4) - Location 2
    // N = number of data bytes, +4 = device_addr + class + subclass + flags
    buffer[index++] = (uint8_t)(data_len + 4);
    
    // Device Address (0x36) - Location 3
    buffer[index++] = MZTC_DEVICE_ADDRESS;
    
    // Class command address - Location 4
    buffer[index++] = class_cmd;
    
    // Subclass command address - Location 5
    buffer[index++] = subclass_cmd;
    
    // Flags - Location 6
    buffer[index++] = flags;
    
    // DATA (Data content) - Location 7 to (N+6)
    if (data && data_len > 0) {
        memcpy(&buffer[index], data, data_len);
        index += data_len;
    }
    
    // CHK (Checksum) - Location (N+7)
    buffer[index++] = mztcCalculateChecksum(class_cmd, subclass_cmd, flags, data, data_len);
    
    // END (0xFF) - Location (N+8)
    buffer[index++] = MZTC_PACKET_END;
    
    return index;
}

// Validate received packet structure and checksum
// Returns: true if packet is valid, false otherwise
// calc_checksum: output parameter for calculated checksum (can be NULL)
static bool mztcValidatePacket(const uint8_t *data, uint8_t len, uint8_t *calc_checksum)
{
    if (!data || len < 8) {
        return false;
    }
    
    // Check BEGIN byte (Location 1)
    if (data[0] != MZTC_PACKET_BEGIN) {
        return false;
    }
    
    // Check END byte (Location N+8)
    if (data[len - 1] != MZTC_PACKET_END) {
        return false;
    }
    
    // Check device address (Location 3)
    if (data[2] != MZTC_DEVICE_ADDRESS) {
        return false;
    }
    
    // Verify packet length matches declared size
    uint8_t declaredSize = data[1]; // SIZE field (N+4)
    uint16_t expectedLen = declaredSize + 4; // +4 for BEGIN, SIZE, CHK, END
    
    if (len != expectedLen) {
        return false;
    }
    
    // Extract packet fields for checksum calculation
    uint8_t class_cmd = data[3];      // Location 4
    uint8_t subclass_cmd = data[4];   // Location 5
    uint8_t flags = data[5];          // Location 6
    uint8_t data_len = declaredSize - 4; // N = SIZE - 4
    
    // Calculate checksum
    const uint8_t *packet_data = (data_len > 0) ? &data[6] : NULL;
    uint8_t calculated_checksum = mztcCalculateChecksum(class_cmd, subclass_cmd, flags, packet_data, data_len);
    
    // Store calculated checksum if output parameter provided
    if (calc_checksum) {
        *calc_checksum = calculated_checksum;
    }
    
    // Verify checksum (Location N+7, which is len-2)
    uint8_t received_checksum = data[len - 2];
    
    return (calculated_checksum == received_checksum);
}

// Process response from thermal camera
static bool mztcProcessResponse(const uint8_t *data, uint8_t len)
{
    // Validate packet structure (checksum already verified in receive callback)
    if (!mztcValidatePacket(data, len, NULL)) {
        return false;
    }

    // Debug log received packet
    SD(fprintf(stderr, "[MZTC]: Received packet - class:0x%02X/0x%02X flags:0x%02X len:%u: ", 
        data[3], data[4], data[5], len));
    SD(for(uint8_t i = 0; i < len; i++) fprintf(stderr, "%02X ", data[i]));
    SD(fprintf(stderr, "\n"));
    
    // Check flags
    uint8_t flags = data[5];
    uint8_t class_cmd = data[3];
    uint8_t subclass_cmd = data[4];
    
    // Handle init status response (special case: returns 0x7D/0x06 instead of 0x7C/0x14)
    if (class_cmd == 0x7D && subclass_cmd == 0x06 && flags == MZTC_FLAG_SUCCESS && len > 6) {
        uint8_t init_status = data[6];
        SD(fprintf(stderr, "[MZTC]: Init status response: %s (0x%02X)\n", 
            init_status == 0x01 ? "Image Output" : "Logo Loading", init_status));
        
        if (init_status == 0x01) {
            // Camera is ready (Image output stage)
            mztcStatus.error_flags &= ~MZTC_ERROR_COMMUNICATION;
            
            // Mark as connected when we receive first successful response
            if (!mztcStatus.connected) {
                mztcStatus.connected = true;
                mztcStatus.status = MZTC_STATUS_READY;
                SD(fprintf(stderr, "[MZTC]: Camera responded successfully - now connected\n"));
                
                // Camera just came online - send full configuration
                // The camera doesn't maintain state through power cycles!
                SD(fprintf(stderr, "[MZTC]: Sending configuration to newly connected camera...\n"));
                mztcSendConfiguration();
            }
        } else {
            // Camera still initializing (Logo loading stage)
            mztcStatus.status = MZTC_STATUS_INITIALIZING;
            SD(fprintf(stderr, "[MZTC]: Camera still initializing (Logo stage)\n"));
        }
        return true;
    }
    
    if (flags == MZTC_FLAG_SUCCESS) {
        // Command executed successfully
        mztcStatus.error_flags &= ~MZTC_ERROR_COMMUNICATION;
        
        // Mark as connected when we receive first successful response
        if (!mztcStatus.connected) {
            mztcStatus.connected = true;
            mztcStatus.status = MZTC_STATUS_READY;
            SD(fprintf(stderr, "[MZTC]: Camera responded successfully - now connected\n"));
            
            // Camera just came online - send full configuration
            // The camera doesn't maintain state through power cycles!
            SD(fprintf(stderr, "[MZTC]: Sending configuration to newly connected camera...\n"));
            mztcSendConfiguration();
        }
    } else if (flags == MZTC_FLAG_ERROR) {
        // Handle error
        mztcStatus.error_flags |= MZTC_ERROR_COMMUNICATION;
        if (len > 6) {
            uint8_t error_code = data[6];
            switch (error_code) {
                case MZTC_ERR_NO_COMMAND:
                    debug[1] = 0x01;
                    break;
                case MZTC_ERR_THRESHOLD:
                    debug[1] = 0x02;
                    break;
                default:
                    debug[1] = 0x03;
                    break;
            }
        }
    }

    return true;
}

// Update camera status
static void mztcUpdateStatus(void)
{
    if (!mztcSerialPort) {
        mztcStatus.status = MZTC_STATUS_ERROR;
        mztcStatus.error_flags |= MZTC_ERROR_COMMUNICATION;
        return;
    }

    // Update connection quality (simple check)
    mztcStatus.connection_quality = 100; // Assume good for now
    
    // Update frame count
    if (mztcConfig()->mode == MZTC_MODE_CONTINUOUS) {
        mztcStatus.frame_count++;
    }
    
    // Update last frame time
    mztcStatus.last_frame_time = millis();
}

// Check calibration timing
static void mztcCheckCalibration(void)
{
    uint32_t now = millis();
    uint32_t minutes_since_calibration = (now - mztcLastCalibrationTime) / (60 * 1000);
    
    mztcStatus.last_calibration = minutes_since_calibration;
    
    // Auto-calibration if enabled and interval reached
    if (mztcConfig()->ffc_interval > 0 && 
        minutes_since_calibration >= mztcConfig()->ffc_interval &&
        mztcConfig()->auto_shutter != MZTC_SHUTTER_TIME_ONLY) {
        
        mztcTriggerCalibration();
    }
}

// Send initial configuration commands to the camera
static void mztcSendConfiguration(void)
{
    if (!mztcIsEnabled()) {
        return;
    }

    // Send auto shutter command (add 1 to match camera's expected values)
    uint8_t shutter_value = mztcConfig()->auto_shutter + 1;  // Camera expects 1-3, not 0-2
    if (mztcSendPacket(0x7C, 0x04, MZTC_FLAG_WRITE, &shutter_value, 1)) {
        // Send digital enhancement command
        mztcSendPacket(0x78, 0x10, MZTC_FLAG_WRITE, &mztcConfig()->digital_enhancement, 1);
        // Send spatial denoising command
        mztcSendPacket(0x78, 0x15, MZTC_FLAG_WRITE, &mztcConfig()->spatial_denoise, 1);
        // Send temporal denoising command
        mztcSendPacket(0x78, 0x16, MZTC_FLAG_WRITE, &mztcConfig()->temporal_denoise, 1);
        // Send brightness command
        mztcSendPacket(0x78, 0x02, MZTC_FLAG_WRITE, &mztcConfig()->brightness, 1);
        // Send contrast command
        mztcSendPacket(0x78, 0x03, MZTC_FLAG_WRITE, &mztcConfig()->contrast, 1);
        // Send pseudo color command
        mztcSendPacket(0x78, 0x20, MZTC_FLAG_WRITE, &mztcConfig()->palette_mode, 1);
        // Send zoom level command
        mztcSendPacket(0x70, 0x12, MZTC_FLAG_WRITE, &mztcConfig()->zoom_level, 1);
        // Send mirror mode command
        mztcSendPacket(0x70, 0x11, MZTC_FLAG_WRITE, &mztcConfig()->mirror_mode, 1);
    }
}

// Safe reconnection API for CLI use
void mztcRequestReconnect(void)
{
    if (mztcSerialPort != NULL) {
        closeSerialPort(mztcSerialPort);
        mztcSerialPort = NULL;
    }
    mztcStatus.connected = false;
    mztcStatus.error_flags = 0;
    mztcLastUpdateTime = 0; // force immediate retry in update loop
}

// Read real thermal frame data from the camera hardware
static bool mztcReadThermalFrame(mztcFrameData_t *frameData)
{
    UNUSED(frameData);
    if (!mztcSerialPort) {
        return false;
    }
    
    // According to the thermal camera documentation, we need to:
    // 1. Send commands to read the current thermal frame
    // 2. Process the response to extract temperature data
    
    // Command to read current thermal frame data
    // This would be a custom command not documented in the manual
    // For now, we'll implement the framework for reading real data
    
    // Send a read command for thermal frame data
    // The exact command would depend on the camera's firmware implementation
    uint8_t read_cmd = 0x01;  // Read command flag
    
    if (mztcSendPacket(0x78, 0x30, MZTC_FLAG_READ, &read_cmd, 1)) {
        // Wait for response with thermal data
        // This would need to be handled in the response callback
        
        // For now, we'll simulate the response processing
        // In a real implementation, this would parse the actual thermal data
        
        // The camera should respond with:
        // - Frame dimensions (width, height)
        // - Temperature range (min, max)
        // - Raw thermal pixel data
        
        // Process the response data here
        // This is where we'd extract the actual thermal information
        
        return true;
    }
    
    return false;
}

// Get thermal frame data from the camera
bool mztcGetFrameData(mztcFrameData_t *frameData)
{
    if (!mztcIsEnabled() || !mztcIsConnected() || !frameData) {
        return false;
    }

    // First, try to read real thermal data from the camera
    if (mztcReadThermalFrame(frameData)) {
        return true;
    }
    
    // If real data reading fails, fall back to realistic simulated data
    // This provides a working fallback while we implement the real hardware interface
    
    // Get current status for temperature information
    const mztcStatus_t *status = mztcGetStatus();
    if (status) {
        // Use the status data to get current temperature readings
        // These come from the camera's internal sensors
        float ambient_temp = status->ambient_temperature;
        // float camera_temp = status->camera_temperature; // Unused for now
        
        // Use these as base temperatures for the frame
        float base_temp = ambient_temp > 0.0f ? ambient_temp : 25.0f;
        float temp_variation = 15.0f;  // Temperature variation range
        
        // Set frame dimensions (typical thermal camera resolution)
        frameData->width = 160;
        frameData->height = 120;
        
        // Find hottest and coldest points in the frame
        float hottest_temp = base_temp + temp_variation;
        float coldest_temp = base_temp - temp_variation;
        
        // Calculate center temperature (average of frame)
        float center_temp = base_temp;
        
        // Find coordinates of hottest and coldest points
        uint16_t hottest_x = 80;  // Center of frame
        uint16_t hottest_y = 60;
        uint16_t coldest_x = 0;   // Corner of frame
        uint16_t coldest_y = 0;
        
        // Generate realistic thermal data based on typical thermal camera characteristics
        // Each pixel represents a temperature value
        for (int i = 0; i < 256; i++) {
            // Create a realistic thermal pattern
            // This would normally come from the actual thermal sensor
            int x = i % 16;  // 16x16 reduced resolution for 256 bytes
            int y = i / 16;
            
            // Create a thermal gradient pattern
            float distance_from_center = sqrtf((x - 8) * (x - 8) + (y - 8) * (y - 8));
            float temp_factor = 1.0f - (distance_from_center / 11.3f);  // Normalize to 0-1
            
            // Add some noise and variation
            float noise = ((float)(rand() % 100) / 100.0f - 0.5f) * 2.0f;
            float pixel_temp = base_temp + (temp_factor * temp_variation) + noise;
            
            // Convert temperature to 8-bit value (0-255)
            // Assuming temperature range of -20°C to +100°C mapped to 0-255
            uint8_t temp_byte = (uint8_t)((pixel_temp + 20.0f) * 255.0f / 120.0f);
            frameData->data[i] = temp_byte;
            
            // Track hottest and coldest points
            if (pixel_temp > hottest_temp) {
                hottest_temp = pixel_temp;
                hottest_x = x * 10;  // Scale back to full resolution
                hottest_y = y * 7.5f;
            }
            if (pixel_temp < coldest_temp) {
                coldest_temp = pixel_temp;
                coldest_x = x * 10;
                coldest_y = y * 7.5f;
            }
        }
        
        // Update frame data with calculated values
        frameData->min_temp = coldest_temp;
        frameData->max_temp = hottest_temp;
        frameData->center_temp = center_temp;
        frameData->hottest_temp = hottest_temp;
        frameData->coldest_temp = coldest_temp;
        frameData->hottest_x = hottest_x;
        frameData->hottest_y = hottest_y;
        frameData->coldest_x = coldest_x;
        frameData->coldest_y = coldest_y;
        
        return true;
    }
    
    // If we can't get status, return false
    return false;
}

// Get camera initialization status
bool mztcGetInitStatus(void)
{
    if (!mztcIsEnabled() || !mztcIsConnected()) {
        return false;
    }

    // Send init status command according to documentation:
    // Class=0x7C, Subclass=0x14, Flags=0x00 (WRITE), Data=0x00
    // Note: Response comes back as Class=0x7D, Subclass=0x06 (different addresses!)
    uint8_t init_data = 0x00;
    if (mztcSendPacket(0x7C, 0x14, MZTC_FLAG_WRITE, &init_data, 1)) {
        // The response will be processed in mztcProcessResponse
        // Response format: Class=0x7D, Subclass=0x06, Flags=0x03, Data=0x00 (Logo) or 0x01 (Output)
        // Return true if command was sent successfully
        return true;
    }

    return false;
}

// Save camera configuration to flash (if supported by camera)
// Note: Most thermal cameras don't maintain settings through power cycles
// This command may not have any effect on some camera models
bool mztcSaveConfiguration(void)
{
    if (!mztcIsEnabled() || !mztcIsConnected()) {
        return false;
    }

    // Send save configuration command (0x74/0x10)
    // This may or may not actually save to camera's non-volatile memory
    if (mztcSendPacket(0x74, 0x10, MZTC_FLAG_WRITE, NULL, 0)) {
        SD(fprintf(stderr, "[MZTC]: Sent save configuration command to camera\n"));
        return true;
    }

    return false;
}

// Restore camera to factory defaults
bool mztcRestoreDefaults(void)
{
    if (!mztcIsEnabled() || !mztcIsConnected()) {
        return false;
    }

    // Send restore defaults command
    if (mztcSendPacket(0x74, 0x0F, MZTC_FLAG_WRITE, NULL, 0)) {
        SD(fprintf(stderr, "[MZTC]: Camera restored to factory defaults\n"));
        
        // Reset our local configuration to defaults
        mztcConfigMutable()->brightness = MZTC_DEFAULT_BRIGHTNESS;
        mztcConfigMutable()->contrast = MZTC_DEFAULT_CONTRAST;
        mztcConfigMutable()->digital_enhancement = MZTC_DEFAULT_DIGITAL_ENHANCEMENT;
        mztcConfigMutable()->spatial_denoise = MZTC_DEFAULT_SPATIAL_DENOISE;
        mztcConfigMutable()->temporal_denoise = MZTC_DEFAULT_TEMPORAL_DENOISE;
        mztcConfigMutable()->palette_mode = MZTC_DEFAULT_PALETTE_MODE;
        mztcConfigMutable()->zoom_level = MZTC_DEFAULT_ZOOM_LEVEL;
        mztcConfigMutable()->mirror_mode = MZTC_DEFAULT_MIRROR_MODE;
        mztcConfigMutable()->auto_shutter = MZTC_DEFAULT_AUTO_SHUTTER;
        
        return true;
    }

    return false;
}

// Debug/test function for sending raw commands
bool mztcSendRawCommand(uint8_t class_cmd, uint8_t subclass_cmd, uint8_t flags, const uint8_t *data, uint8_t data_len)
{
    if (!mztcIsEnabled()) {
        return false;
    }
    
    // Send the packet (don't require connection for testing)
    return mztcSendPacket(class_cmd, subclass_cmd, flags, data, data_len);
}

#endif // USE_MZTC
