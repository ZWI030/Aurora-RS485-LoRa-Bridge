#include "Aurora.h"

// =================================================================
//  Configuration for Aurora Inverter (RS485)
// =================================================================

// !!! Confirm the address
#define INVERTER_ADDRESS 2

// Define the flow control pins
#define RS485_CONTROL_PIN 21  

// The Serial2 pin for RS485 communication (Heltec Wireless Stick Lite V3)
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17

Aurora inverter(INVERTER_ADDRESS, &Serial2, RS485_CONTROL_PIN);      


void setup() {
    Serial.begin(115200);
    delay(2000); 

    Serial.println("\n--- Aurora Inverter Reader  ---");
    Serial.println("Initializing...");

    Serial2.begin(19200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    inverter.begin();

    Serial.printf("Setup complete. Attempting to communicate with inverter at address %d.\n", INVERTER_ADDRESS);
    Serial.println("Starting periodic reads in 3 seconds...");
    delay(3000);
}

void loop() {
    readAndPrintInverterData();

    Serial.println("\n-----------------------------------\n");

    delay(5000);
}

// =================================================================
//  Data Acquisition and Printing Core Function
// =================================================================

void readAndPrintInverterData() {
    Serial.println("--- Starting New Read Cycle ---");

    // 1. Read State Information (Command 50)
    Aurora::DataState state = inverter.readState();
    if (state.state.readState) {
        Serial.printf("  [OK] Global State: %d (%s)\n", state.state.globalState, state.state.getGlobalState().c_str());
        Serial.printf("  [OK] Inverter State: %d (%s)\n", state.inverterState, state.getInverterState().c_str());
        Serial.printf("  [OK] Alarm State: %d (%s)\n", state.alarmState, state.getAlarmState().c_str());
    } else {
        Serial.println("  [FAIL] Failed to read State.");
    }

    // 2. Read DSP Real-Time Measurements (Command 59)
    Aurora::DataDSP dsp;
    dsp = inverter.readDSP(3, 0); // Grid Power
    if (dsp.state.readState) { Serial.printf("  [OK] Grid Power: %.2f W\n", dsp.value); } else { Serial.println("  [FAIL] Failed to read Grid Power."); }
    
    dsp = inverter.readDSP(1, 0); // Grid Voltage
    if (dsp.state.readState) { Serial.printf("  [OK] Grid Voltage: %.2f V\n", dsp.value); } else { Serial.println("  [FAIL] Failed to read Grid Voltage."); }
    
    dsp = inverter.readDSP(23, 0); // PV Voltage
    if (dsp.state.readState) { Serial.printf("  [OK] PV Voltage (Input 1): %.2f V\n", dsp.value); } else { Serial.println("  [FAIL] Failed to read PV Voltage."); }
    
    dsp = inverter.readDSP(21, 0); // Inverter Temperature
    if (dsp.state.readState) { Serial.printf("  [OK] Inverter Temperature: %.2f C\n", dsp.value); } else { Serial.println("  [FAIL] Failed to read Inverter Temp."); }

    // 3. Read Cumulative Energy (Command 78)
    Aurora::DataCumulatedEnergy energy = inverter.readCumulatedEnergy(0); // Daily Energy
    if (energy.state.readState) { Serial.printf("  [OK] Daily Energy: %lu Wh\n", energy.energy); } else { Serial.println("  [FAIL] Failed to read Daily Energy."); }

    // 4. Read Last Four Alarms (Command 86)
    Aurora::DataLastFourAlarms alarms = inverter.readLastFourAlarms();
    if (alarms.state.readState) {
        Serial.printf("  [OK] Last 4 Alarms (newest to oldest): %d, %d, %d, %d\n", alarms.alarm4, alarms.alarm3, alarms.alarm2, alarms.alarm1);
    } else {
        Serial.println("  [FAIL] Failed to read Last Four Alarms.");
    }

    // 5. Read Part Number (PN - Command 52)
    Aurora::DataSystemPN pn = inverter.readSystemPN();
    if (pn.readState) { Serial.printf("  [OK] Part Number: %s\n", pn.PN.c_str()); } else { Serial.println("  [FAIL] Failed to read Part Number."); }

    // 6. Read Serial Number (SN - Command 63)
    Aurora::DataSystemSerialNumber sn = inverter.readSystemSerialNumber();
    if (sn.readState) { Serial.printf("  [OK] Serial Number: %s\n", sn.SerialNumber.c_str()); } else { Serial.println("  [FAIL] Failed to read Serial Number."); }
}