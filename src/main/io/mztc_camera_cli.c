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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MZTC

#include "build/debug.h"
#include "common/printf.h"
#include "io/mztc_camera.h"
#include "io/mztc_camera_cli.h"

// CLI command: mztc status
static void mztcStatusCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    const mztcStatus_t *status = mztcGetStatus();
    if (!status) {
        cliPrint("MassZero Thermal Camera: ERROR - Status unavailable\n");
        return;
    }
    
    cliPrint("MassZero Thermal Camera Status:\n");
    cliPrint("  Enabled: %s\n", mztcConfig()->enabled ? "YES" : "NO");
    cliPrint("  Connected: %s\n", mztcIsConnected() ? "YES" : "NO");
    const char *statusNames[] = {"OFFLINE", "ERROR", "READY", "CALIBRATING", "ALERT"};
    const char *modeNames[] = {"DISABLED", "STANDBY", "CONTINUOUS", "TRIGGERED", "ALERT", "RECORDING", "CALIBRATION", "SURVEILLANCE"};
    
    cliPrintf("  Status: %s (%d)\n", 
        (status->status < ARRAYLEN(statusNames)) ? statusNames[status->status] : "UNKNOWN", 
        status->status);
    cliPrintf("  Mode: %s (%d)\n", 
        (status->mode < ARRAYLEN(modeNames)) ? modeNames[status->mode] : "UNKNOWN", 
        status->mode);
    cliPrint("  Connection Quality: %d%%\n", status->connection_quality);
    cliPrint("  Last Calibration: %d minutes ago\n", status->last_calibration);
    cliPrintf("  Camera Temperature: %.1f°C\n", status->camera_temperature);
    cliPrintf("  Ambient Temperature: %.1f°C\n", status->ambient_temperature);
    cliPrint("  Frame Count: %lu\n", status->frame_count);
    cliPrint("  Error Flags: 0x%02X\n", status->error_flags);
    cliPrint("  Last Frame: %lu ms ago\n", millis() - status->last_frame_time);
}

// CLI command: mztc calibrate
static void mztcCalibrateCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    if (!mztcIsConnected()) {
        cliPrint("MassZero Thermal Camera: NOT CONNECTED\n");
        return;
    }
    
    if (mztcTriggerCalibration()) {
        cliPrint("MassZero Thermal Camera: Calibration triggered\n");
    } else {
        cliPrint("MassZero Thermal Camera: Calibration failed\n");
    }
}

// CLI command: mztc init_status
static void mztcInitStatusCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    if (!mztcIsConnected()) {
        cliPrint("MassZero Thermal Camera: NOT CONNECTED\n");
        return;
    }
    
    if (mztcGetInitStatus()) {
        cliPrint("MassZero Thermal Camera: Init status request sent\n");
    } else {
        cliPrint("MassZero Thermal Camera: Init status request failed\n");
    }
}

// CLI command: mztc save_config
static void mztcSaveConfigCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    if (!mztcIsConnected()) {
        cliPrint("MassZero Thermal Camera: NOT CONNECTED\n");
        return;
    }
    
    if (mztcSaveConfiguration()) {
        cliPrint("MassZero Thermal Camera: Save configuration command sent\n");
        cliPrint("Note: Camera may not retain settings through power cycles\n");
    } else {
        cliPrint("MassZero Thermal Camera: Save configuration failed\n");
    }
}

// CLI command: mztc restore_defaults
static void mztcRestoreDefaultsCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    if (!mztcIsConnected()) {
        cliPrint("MassZero Thermal Camera: NOT CONNECTED\n");
        return;
    }
    
    if (mztcRestoreDefaults()) {
        cliPrint("MassZero Thermal Camera: Restored to factory defaults\n");
    } else {
        cliPrint("MassZero Thermal Camera: Restore defaults failed\n");
    }
}

// CLI command: mztc reconnect
static void mztcReconnectCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    mztcRequestReconnect();
    cliPrint("MassZero Thermal Camera: Reconnection requested\n");
}

// CLI command: mztc preset
static void mztcPresetCommand(char *cmdline)
{
    if (!mztcIsEnabled()) {
        cliPrint("MassZero Thermal Camera: DISABLED\n");
        return;
    }
    
    char *preset = strtok(cmdline, " ");
    if (!preset) {
        cliPrint("Usage: mztc_preset <preset_name>\n");
        cliPrint("Available presets: fire_detection, search_rescue, surveillance, rapid_changes, industrial\n");
        return;
    }
    
    mztcConfig_t *config = mztcConfigMutable();
    
    if (strcmp(preset, "fire_detection") == 0) {
        // Fire Detection Mode
        config->mode = MZTC_MODE_ALERT;
        config->palette_mode = MZTC_PALETTE_WHITE_HOT;
        config->brightness = 70;
        config->contrast = 80;
        config->digital_enhancement = 75;
        config->spatial_denoise = 30;
        config->temporal_denoise = 40;
        config->auto_shutter = MZTC_SHUTTER_TIME_AND_TEMP;
        config->ffc_interval = 3;
        config->temperature_alerts = 1;
        config->alert_high_temp = 60.0f;
        config->alert_low_temp = -10.0f;
        config->update_rate = 15;
        cliPrint("MassZero Thermal Camera: Fire Detection preset applied\n");
    }
    else if (strcmp(preset, "search_rescue") == 0) {
        // Search and Rescue Mode
        config->mode = MZTC_MODE_CONTINUOUS;
        config->palette_mode = MZTC_PALETTE_FUSION_1;
        config->brightness = 60;
        config->contrast = 70;
        config->digital_enhancement = 60;
        config->spatial_denoise = 50;
        config->temporal_denoise = 60;
        config->auto_shutter = MZTC_SHUTTER_TIME_AND_TEMP;
        config->ffc_interval = 5;
        config->temperature_alerts = 1;
        config->alert_high_temp = 45.0f;
        config->alert_low_temp = 15.0f;
        config->update_rate = 9;
        config->zoom_level = MZTC_ZOOM_2X;
        cliPrint("MassZero Thermal Camera: Search and Rescue preset applied\n");
    }
    else if (strcmp(preset, "surveillance") == 0) {
        // Surveillance Mode
        config->mode = MZTC_MODE_SURVEILLANCE;
        config->palette_mode = MZTC_PALETTE_BLACK_HOT;
        config->brightness = 50;
        config->contrast = 60;
        config->digital_enhancement = 50;
        config->spatial_denoise = 60;
        config->temporal_denoise = 70;
        config->auto_shutter = MZTC_SHUTTER_TIME_ONLY;
        config->ffc_interval = 10;
        config->temperature_alerts = 1;
        config->alert_high_temp = 50.0f;
        config->alert_low_temp = 0.0f;
        config->update_rate = 5;
        config->zoom_level = MZTC_ZOOM_4X;
        cliPrint("MassZero Thermal Camera: Surveillance preset applied\n");
    }
    else if (strcmp(preset, "rapid_changes") == 0) {
        // Rapid Environment Changes Mode
        config->mode = MZTC_MODE_CONTINUOUS;
        config->palette_mode = MZTC_PALETTE_RAINBOW;
        config->brightness = 55;
        config->contrast = 65;
        config->digital_enhancement = 70;
        config->spatial_denoise = 40;
        config->temporal_denoise = 30;
        config->auto_shutter = MZTC_SHUTTER_TEMP_ONLY;
        config->ffc_interval = 2;
        config->temperature_alerts = 1;
        config->alert_high_temp = 80.0f;
        config->alert_low_temp = -20.0f;
        config->update_rate = 20;
        config->bad_pixel_removal = 1;
        config->vignetting_correction = 1;
        cliPrint("MassZero Thermal Camera: Rapid Changes preset applied\n");
    }
    else if (strcmp(preset, "industrial") == 0) {
        // Industrial Inspection Mode
        config->mode = MZTC_MODE_CONTINUOUS;
        config->palette_mode = MZTC_PALETTE_IRON_RED_1;
        config->brightness = 60;
        config->contrast = 75;
        config->digital_enhancement = 65;
        config->spatial_denoise = 45;
        config->temporal_denoise = 55;
        config->auto_shutter = MZTC_SHUTTER_TIME_AND_TEMP;
        config->ffc_interval = 7;
        config->temperature_alerts = 1;
        config->alert_high_temp = 100.0f;
        config->alert_low_temp = -30.0f;
        config->update_rate = 12;
        config->zoom_level = MZTC_ZOOM_1X;
        cliPrint("MassZero Thermal Camera: Industrial Inspection preset applied\n");
    }
    else {
        cliPrint("Unknown preset: %s\n", preset);
        cliPrint("Available presets: fire_detection, search_rescue, surveillance, rapid_changes, industrial\n");
        return;
    }
    
    // Apply the configuration to the camera if connected
    if (mztcIsConnected()) {
        mztcSendConfiguration();
        cliPrint("Configuration sent to camera\n");
    }
}

// CLI command: mztc send
static void mztcSendCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MZTC is not enabled\n");
        return;
    }
    
    // Parse hex values from command line
    // Format: mztc send <class> <subclass> <flags> [data bytes...]
    char *arg = strtok(NULL, " ");
    if (!arg) {
        cliPrint("Usage: mztc_send <class_hex> <subclass_hex> <flags_hex> [data_hex...]\n");
        cliPrint("Example: mztc_send 78 02 00 64  (set brightness to 100)\n");
        cliPrint("Example: mztc_send 7C 14 01      (read init status)\n");
        return;
    }
    
    // Parse class command
    uint8_t class_cmd = strtoul(arg, NULL, 16);
    
    // Parse subclass command
    arg = strtok(NULL, " ");
    if (!arg) {
        cliPrint("Missing subclass command\n");
        return;
    }
    uint8_t subclass_cmd = strtoul(arg, NULL, 16);
    
    // Parse flags
    arg = strtok(NULL, " ");
    if (!arg) {
        cliPrint("Missing flags\n");
        return;
    }
    uint8_t flags = strtoul(arg, NULL, 16);
    
    // Parse data bytes
    uint8_t data[14];
    uint8_t data_len = 0;
    
    while ((arg = strtok(NULL, " ")) != NULL && data_len < 14) {
        data[data_len++] = strtoul(arg, NULL, 16);
    }
    
    // Send the packet
    cliPrintf("Sending: class=0x%02X subclass=0x%02X flags=0x%02X data_len=%u\n", 
              class_cmd, subclass_cmd, flags, data_len);
    
    if (data_len > 0) {
        cliPrint("Data: ");
        for (uint8_t i = 0; i < data_len; i++) {
            cliPrintf("%02X ", data[i]);
        }
        cliPrint("\n");
    }
    
    if (mztcSendRawCommand(class_cmd, subclass_cmd, flags, data, data_len)) {
        cliPrint("Packet sent successfully\n");
    } else {
        cliPrint("Failed to send packet\n");
    }
}

// CLI command: mztc test
static void mztcTestCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MZTC is not enabled\n");
        return;
    }
    
    cliPrint("Sending test commands to camera...\n");
    
    // Test 1: Read init status
    cliPrint("1. Reading init status (0x7C/0x14)...\n");
    mztcInitStatus();
    delay(500);
    
    // Test 2: Set brightness to 75
    cliPrint("2. Setting brightness to 75 (0x78/0x02)...\n");
    uint8_t brightness = 75;
    mztcSetImageParams(brightness, 50, 50);
    delay(500);
    
    // Test 3: Set palette to Rainbow (3)
    cliPrint("3. Setting palette to Rainbow (0x78/0x20)...\n");
    mztcSetPalette(MZTC_PALETTE_RAINBOW);
    delay(500);
    
    // Test 4: Set zoom to 2x
    cliPrint("4. Setting zoom to 2x (0x70/0x12)...\n");
    mztcSetZoom(MZTC_ZOOM_2X);
    
    cliPrint("Test commands sent. Check if camera responded.\n");
}

// CLI command: mztc_palette_test
static void mztcPaletteTestCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MZTC is not enabled\n");
        return;
    }
    
    cliPrint("Testing palette changes...\n");
    
    // Test different palettes
    uint8_t palettes[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
    const char* palette_names[] = {
        "White Hot", "Black Hot", "Fusion 1", "Rainbow", "Fusion 2", 
        "Iron Red 1", "Iron Red 2", "Sepia", "Color 1", "Color 2", 
        "Ice Fire", "Rain", "Green Hot", "Red Hot"
    };
    
    for (int i = 0; i < 14; i++) {
        cliPrintf("Setting palette %d (%s)...\n", palettes[i], palette_names[i]);
        
        // Send palette command directly to camera
        if (mztcSendRawCommand(0x78, 0x20, 0x00, &palettes[i], 1)) {
            cliPrint("Command sent successfully\n");
        } else {
            cliPrint("Failed to send command\n");
        }
        
        // Wait 2 seconds between changes
        delay(2000);
    }
    
    cliPrint("Palette test complete\n");
}

// CLI command: mztc_passthrough
static void mztcPassthroughCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    if (!mztcIsEnabled()) {
        cliPrint("MZTC is not enabled\n");
        return;
    }
    
    cliPrint("UART Pass-through mode activated\n");
    cliPrint("Send hex bytes separated by spaces (e.g., F0 05 36 78 20 00 01)\n");
    cliPrint("Type 'exit' to quit pass-through mode\n");
    
    char input[256];
    while (1) {
        cliPrint("MZTC> ");
        
        // Read input (simplified - in real implementation you'd need proper input handling)
        // For now, just demonstrate the concept
        cliPrint("Pass-through mode not fully implemented yet\n");
        cliPrint("Use 'mztc_send' command instead for now\n");
        break;
    }
}

// CLI command: mztc_palette
static void mztcPaletteCommand(char *cmdline)
{
    if (!mztcIsEnabled()) {
        cliPrint("MZTC is not enabled\n");
        return;
    }
    
    // Parse palette number from command line
    char *arg = strtok(NULL, " ");
    if (!arg) {
        cliPrint("Usage: mztc_palette <0-13>\n");
        cliPrint("Palettes: 0=White Hot, 1=Black Hot, 2=Fusion 1, 3=Rainbow, 4=Fusion 2\n");
        cliPrint("         5=Iron Red 1, 6=Iron Red 2, 7=Sepia, 8=Color 1, 9=Color 2\n");
        cliPrint("         10=Ice Fire, 11=Rain, 12=Green Hot, 13=Red Hot\n");
        return;
    }
    
    uint8_t palette = strtoul(arg, NULL, 10);
    if (palette > 13) {
        cliPrint("Invalid palette number. Must be 0-13\n");
        return;
    }
    
    const char* palette_names[] = {
        "White Hot", "Black Hot", "Fusion 1", "Rainbow", "Fusion 2", 
        "Iron Red 1", "Iron Red 2", "Sepia", "Color 1", "Color 2", 
        "Ice Fire", "Rain", "Green Hot", "Red Hot"
    };
    
    cliPrintf("Setting palette to %d (%s)...\n", palette, palette_names[palette]);
    
    // Send palette command directly to camera
    if (mztcSendRawCommand(0x78, 0x20, 0x00, &palette, 1)) {
        cliPrint("Palette command sent successfully\n");
    } else {
        cliPrint("Failed to send palette command\n");
    }
}

// CLI command: mztc help
static void mztcHelpCommand(char *cmdline)
{
    UNUSED(cmdline);
    
    cliPrint("MassZero Thermal Camera Commands:\n");
    cliPrint("  mztc_status          - Show camera status\n");
    cliPrint("  mztc_calibrate       - Trigger manual calibration (FFC)\n");
    cliPrint("  mztc_init_status     - Get camera initialization status\n");
    cliPrint("  mztc_save_config     - Save configuration to camera flash\n");
    cliPrint("  mztc_restore_defaults - Restore camera to factory defaults\n");
    cliPrint("  mztc_reconnect       - Reconnect to camera\n");
    cliPrint("  mztc_preset <name>   - Apply application preset\n");
    cliPrint("  mztc_send <cmd>      - Send raw command to camera\n");
    cliPrint("  mztc_test            - Send test commands\n");
    cliPrint("  mztc_palette_test    - Test all palette changes\n");
    cliPrint("  mztc_palette <num>   - Set specific palette (0-13)\n");
    cliPrint("  mztc_help            - Show this help\n");
    cliPrint("\nAvailable presets:\n");
    cliPrint("  fire_detection       - Fire detection and hot spot identification\n");
    cliPrint("  search_rescue        - Human detection and body heat signatures\n");
    cliPrint("  surveillance         - Long-range monitoring and perimeter security\n");
    cliPrint("  rapid_changes        - Fast-changing environments (weather, altitude)\n");
    cliPrint("  industrial           - Equipment monitoring and thermal analysis\n");
}

// CLI command table
static const cliCommand_t mztcCommands[] = {
    { "mztc_status", "", mztcStatusCommand, "Show MassZero Thermal Camera status" },
    { "mztc_calibrate", "", mztcCalibrateCommand, "Trigger manual calibration (FFC)" },
    { "mztc_init_status", "", mztcInitStatusCommand, "Get camera initialization status" },
    { "mztc_save_config", "", mztcSaveConfigCommand, "Save configuration to camera flash" },
    { "mztc_restore_defaults", "", mztcRestoreDefaultsCommand, "Restore camera to factory defaults" },
    { "mztc_reconnect", "", mztcReconnectCommand, "Reconnect to camera" },
    { "mztc_preset", "", mztcPresetCommand, "Apply application-specific preset" },
    { "mztc_send", "", mztcSendCommand, "Send raw command to camera" },
    { "mztc_test", "", mztcTestCommand, "Send test commands to camera" },
    { "mztc_palette_test", "", mztcPaletteTestCommand, "Test all palette changes" },
    { "mztc_palette", "", mztcPaletteCommand, "Set specific palette (0-13)" },
    { "mztc_help", "", mztcHelpCommand, "Show MassZero Thermal Camera help" },
};

// Register CLI commands
void mztcCliInit(void)
{
    for (size_t i = 0; i < ARRAYLEN(mztcCommands); i++) {
        cliAddCommand(&mztcCommands[i]);
    }
}

#endif // USE_MZTC