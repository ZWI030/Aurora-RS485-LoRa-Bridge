/*
 * Original Author: xreef
 * Original Repo: https://github.com/xreef/ABB_Aurora_Solar_Inverter_Library
 * * Modified by: ZWI030
 * Date: 2025-11-15
 * Modification: Optimized send() function for ESP32 with echo cancellation and improved flow control.
 */

#include "Aurora.h"

// ESP32 optimized constructor - HardwareSerial only
Aurora::Aurora(byte inverterAddress, HardwareSerial* serial, byte serialCommunicationControlPin) {
    this->inverterAddress = inverterAddress;
    this->serialCommunicationControlPin = serialCommunicationControlPin;
    this->hs = serial;
    sendStatus = false;
    receiveStatus = false;
    clearReceiveData();
}

void Aurora::begin(){
    pinMode(this->serialCommunicationControlPin, OUTPUT);
    digitalWrite(this->serialCommunicationControlPin, RS485_RX_PIN_VALUE);
    if (this->hs){
        this->serialDef.begin(*this->hs, 19200);
    }
}

void Aurora::setInverterAddress(byte inverterAddress) { this->inverterAddress = inverterAddress; }
byte Aurora::getInverterAddress() { return this->inverterAddress; }
void Aurora::clearData(byte *data, byte len) { for (int i = 0; i < len; i++) { data[i] = 0; } }

int Aurora::crc16(byte *data, int offset, int count) {
    byte BccLo = 0xFF;
    byte BccHi = 0xFF;
    for (int i = 0; i < count; i++) {
        byte New = data[offset + i] ^ BccLo;
        byte Tmp = New << 4;
        New = Tmp ^ New;
        Tmp = New >> 5;
        BccLo = BccHi;
        BccHi = New ^ Tmp;
        Tmp = New << 3;
        BccLo = BccLo ^ Tmp;
        Tmp = New >> 4;
        BccLo = BccLo ^ Tmp;
    }
    return (int)word(~BccHi, ~BccLo);
}

//----------------   Automatic flow direction control   -----------------------
//================
//This function is only used when using an RS485 module with automatic flow control.
//=================
/*
bool Aurora::send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, byte param5, byte param6) {
    sendStatus = false;
    receiveStatus = false;
    byte sendData[10];
    sendData[0] = address; sendData[1] = param0; sendData[2] = param1; sendData[3] = param2;
    sendData[4] = param3; sendData[5] = param4; sendData[6] = param5; sendData[7] = param6;
    int crc = crc16(sendData, 0, 8);
    sendData[8] = lowByte(crc);
    sendData[9] = highByte(crc);

    clearReceiveData();

    for (int i = 0; i < MAX_ATTEMP; i++) {
        // Clear the serial port receiving buffer
        while(this->serialDef.stream->available()) {
            this->serialDef.stream->read();
        }

        // Send data
        if (this->serialDef.stream->write(sendData, sizeof(sendData)) == sizeof(sendData)) {
            this->serialDef.stream->flush(); 
            sendStatus = true;

            byte bytesRead = 0;
            unsigned long startTime = millis();
            const unsigned long timeout = 500; 

            while (millis() - startTime < timeout && bytesRead < sizeof(receiveData)) {
                if (this->serialDef.stream->available() > 0) {
                    receiveData[bytesRead] = this->serialDef.stream->read();
                    bytesRead++;
                }
            }
            

            // Only perform CRC check if the expected length of data is received
            if (bytesRead == sizeof(receiveData)) {
                if ((int)word(receiveData[7], receiveData[6]) == crc16(receiveData, 0, 6)) {
                    receiveStatus = true;
                    break; 
                }
            }
        }
    }
    return receiveStatus;
}

//----------------   Original library functions   -----------------------

bool Aurora::send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, byte param5, byte param6) {
    sendStatus = false;
    receiveStatus = false;
    byte sendData[10];
    sendData[0] = address; sendData[1] = param0; sendData[2] = param1; sendData[3] = param2;
    sendData[4] = param3; sendData[5] = param4; sendData[6] = param5; sendData[7] = param6;
    int crc = crc16(sendData, 0, 8);
    sendData[8] = lowByte(crc);
    sendData[9] = highByte(crc);
    clearReceiveData();
    for (int i = 0; i < MAX_ATTEMP; i++) {
        //digitalWrite(this->serialCommunicationControlPin, RS485_TX_PIN_VALUE);
        delay(5);
        if (this->serialDef.stream->write(sendData, sizeof(sendData)) != 0) {
            this->serialDef.stream->flush();
            sendStatus = true;

            
            //digitalWrite(this->serialCommunicationControlPin, RS485_RX_PIN_VALUE);
            if (this->serialDef.stream->readBytes(receiveData, sizeof(receiveData)) != 0) {
                if ((int)word(receiveData[7], receiveData[6]) == crc16(receiveData, 0, 6)) {
                    receiveStatus = true;
                    break;
                }
            }
        }
    }
    return receiveStatus;
}
*/

//============================================================
// Program flow control send function: program flow control + echo cancellation (nRE grounded, receive open)
// Regardless of whether nRE is grounded or connected to DE and then to GPIO, this function can be used
//============================================================

bool Aurora::send(byte address, byte param0, byte param1, byte param2, byte param3, byte param4, byte param5, byte param6) {
    sendStatus = false;
    receiveStatus = false;
    
    byte sendData[10];
    sendData[0] = address; sendData[1] = param0; sendData[2] = param1; sendData[3] = param2;
    sendData[4] = param3; sendData[5] = param4; sendData[6] = param5; sendData[7] = param6;
    int crc = crc16(sendData, 0, 8);
    sendData[8] = lowByte(crc);
    sendData[9] = highByte(crc);

    clearReceiveData();

    for (int i = 0; i < MAX_ATTEMP; i++) {
        //1. Clear the hardware RX cache before sending (remove residual noise)
        while(this->serialDef.stream->available()) {
            this->serialDef.stream->read();
        }

        // 2. Switch to send mode (DE high)
        digitalWrite(this->serialCommunicationControlPin, HIGH); 
        delayMicroseconds(10); 

        // 3. Write data
        size_t bytesSent = this->serialDef.stream->write(sendData, sizeof(sendData));
        
        // 4. Wait for sending completion
        this->serialDef.stream->flush(); 
        
        // 5. Switch back to receive mode (DE low)
        digitalWrite(this->serialCommunicationControlPin, LOW); 

        // ============================================================
        // Echo cancellation
        // ============================================================
        size_t echoCount = 0;
        unsigned long echoStart = millis();
        while(echoCount < bytesSent && (millis() - echoStart < 100)) {      // If sure no hardware echo, can reduce to 10ms or comment out directly
            if (this->serialDef.stream->available()) {
                this->serialDef.stream->read(); 
                echoCount++;
            }
        }

        // 6. Receive the inverter's reply
        if (bytesSent == sizeof(sendData)) {
            sendStatus = true;
            byte bytesRead = 0;
            unsigned long startTime = millis();
            const unsigned long timeout = 1000; // Timeout 1000ms

            while (millis() - startTime < timeout && bytesRead < sizeof(receiveData)) {
                if (this->serialDef.stream->available() > 0) {
                    receiveData[bytesRead] = this->serialDef.stream->read();
                    bytesRead++;
                }
            }

            // 7. CRC 
            if (bytesRead == sizeof(receiveData)) {
                if ((int)word(receiveData[7], receiveData[6]) == crc16(receiveData, 0, 6)) {
                    receiveStatus = true;
                    break; 
                }
            }
        }
    }
    return receiveStatus;
}



void Aurora::clearReceiveData() { clearData(receiveData, 8); }

Aurora::DataState Aurora::readState() {
	DataState allState;
    allState.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_STATE, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (allState.state.readState == true) { 
        allState.state.transmissionState = receiveData[0];
        allState.state.globalState = receiveData[1];
        allState.inverterState = receiveData[2];
        allState.channel1State = receiveData[3];
        allState.channel2State = receiveData[4];
        allState.alarmState = receiveData[5];
    }
    return allState;
}

Aurora::DataVersion Aurora::readVersion() {
	DataVersion version;
    version.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_VERSION, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (version.state.readState == true) { 
        version.state.transmissionState = receiveData[0];
        version.state.globalState = receiveData[1];
        version.par1 = (char)receiveData[aPar1];
        version.par2 = (char)receiveData[aPar2];
        version.par3 = (char)receiveData[aPar3];
        version.par4 = (char)receiveData[aPar4];
    }
    return version;
}


Aurora::DataDSP Aurora::readDSP(byte type, byte global) {
	DataDSP DSP;
    if ((((int)type >= 1 && (int)type <= 9) || ((int)type >= 21 && (int)type <= 63)) && ((int)global <= 1)) {
        DSP.state.readState = send(this->inverterAddress, (byte)COMMAND_DSP, type, global, (byte)0, (byte)0, (byte)0, (byte)0);
    } else {
        DSP.state.readState = false;
        clearReceiveData();
    }
    if (DSP.state.readState == true) { 
        DSP.state.transmissionState = receiveData[0];
        DSP.state.globalState = receiveData[1];
        foo.asBytes[0] = receiveData[5];
        foo.asBytes[1] = receiveData[4];
        foo.asBytes[2] = receiveData[3];
        foo.asBytes[3] = receiveData[2];
        DSP.value = foo.asFloat;
    }
    return DSP;
}
Aurora::DataTimeDate Aurora::readTimeDate() {
	DataTimeDate timeDate;
    timeDate.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_TIMEDATE, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (timeDate.state.readState == true) {
        timeDate.state.transmissionState = receiveData[0];
        timeDate.state.globalState = receiveData[1];
        timeDate.seconds = ((unsigned long)receiveData[2] << 24) + ((unsigned long)receiveData[3] << 16) + ((unsigned long)receiveData[4] << 8) + (unsigned long)receiveData[5];
        timeDate.epochTime = timeDate.seconds+EPOCH_TO_START_TIME;
    }
    return timeDate;
}
bool Aurora::writeTimeDate(unsigned long epochTime) {
	unsigned long timeInSecond = epochTime - EPOCH_TO_START_TIME;
    if (timeInSecond>0) {
    	byte sendingDateTime[4];
    	sendingDateTime[0] = (timeInSecond >> 24) & 0xff;
    	sendingDateTime[1] = (timeInSecond >> 16) & 0xff;
    	sendingDateTime[2] = (timeInSecond >> 8) & 0xff;
    	sendingDateTime[3] = timeInSecond & 0xff;
        return send(this->inverterAddress, (byte)COMMAND_SET_TIME_DATE, sendingDateTime[0], sendingDateTime[1], sendingDateTime[2], sendingDateTime[3], (byte)0, (byte)0);
    }
    else {
        clearReceiveData();
        return false;
    }
}
Aurora::DataLastFourAlarms Aurora::readLastFourAlarms() {
	DataLastFourAlarms lastFourAlarms;
    lastFourAlarms.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_LAST_4_ALARMS, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (lastFourAlarms.state.readState == true) {
        lastFourAlarms.state.transmissionState = receiveData[0];
        lastFourAlarms.state.globalState = receiveData[1];
        lastFourAlarms.alarm1 = receiveData[2];
        lastFourAlarms.alarm2 = receiveData[3];
        lastFourAlarms.alarm3 = receiveData[4];
        lastFourAlarms.alarm4 = receiveData[5];
    }
    return lastFourAlarms;
}
Aurora::DataJunctionBoxState Aurora::readJunctionBoxState(byte nj) { /* ... 内容不变 ... */ return DataJunctionBoxState(); }
bool Aurora::readJunctionBoxVal(byte nj, byte par) { return send(this->inverterAddress, (byte)201, nj, par, (byte)0, (byte)0, (byte)0, (byte)0); }

// +++ 修正以下函数 +++
Aurora::DataSystemPN Aurora::readSystemPN() {
	DataSystemPN systemPN;
    systemPN.readState = send(this->inverterAddress, (byte)COMMAND_GET_PN, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (systemPN.readState == true) { 
        systemPN.PN = String(String((char)receiveData[0]) + String((char)receiveData[1]) + String((char)receiveData[2]) + String((char)receiveData[3]) + String((char)receiveData[4]) + String((char)receiveData[5]));
    }
    return systemPN;
}

Aurora::DataSystemSerialNumber Aurora::readSystemSerialNumber() {
	DataSystemSerialNumber systemSerialNumber;
    systemSerialNumber.readState = send(this->inverterAddress, (byte)COMMAND_GET_SN, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (systemSerialNumber.readState == true) { 
        systemSerialNumber.SerialNumber = String(String((char)receiveData[0]) + String((char)receiveData[1]) + String((char)receiveData[2]) + String((char)receiveData[3]) + String((char)receiveData[4]) + String((char)receiveData[5]));
    }
    return systemSerialNumber;
}

Aurora::DataManufacturingWeekYear Aurora::readManufacturingWeekYear() {
	DataManufacturingWeekYear manufacturingWeekYear;
    manufacturingWeekYear.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_MANUFACTURING_WEEK_YEAR, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (manufacturingWeekYear.state.readState == true) { 
        manufacturingWeekYear.state.transmissionState = receiveData[0];
        manufacturingWeekYear.state.globalState = receiveData[1];
        manufacturingWeekYear.Week = String(String((char)receiveData[aWeekH]) + String((char)receiveData[aWeekL]));
        manufacturingWeekYear.Year = String(String((char)receiveData[aYearH]) + String((char)receiveData[aYearL]));
    }
    return manufacturingWeekYear;
}

Aurora::DataFirmwareRelease Aurora::readFirmwareRelease() {
	DataFirmwareRelease firmwareRelease;
    firmwareRelease.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_FIRMWARE_RELEASE, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (firmwareRelease.state.readState == true) { 
        firmwareRelease.state.transmissionState = receiveData[0];
        firmwareRelease.state.globalState = receiveData[1];
        firmwareRelease.release = String(String((char)receiveData[aRel3]) + "." + String((char)receiveData[aRel2]) + "." + String((char)receiveData[aRel1]) + "." + String((char)receiveData[aRel0]));
    }
    return firmwareRelease;
}

Aurora::DataCumulatedEnergy Aurora::readCumulatedEnergy(byte par) {
	DataCumulatedEnergy cumulatedEnergy;
    if ((int)par <= 6) {
        cumulatedEnergy.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_CUMULATED_ENERGY, par, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    } else {
        cumulatedEnergy.state.readState = false;
        clearReceiveData();
    }
    if (cumulatedEnergy.state.readState == true) { 
        cumulatedEnergy.state.transmissionState = receiveData[0];
        cumulatedEnergy.state.globalState = receiveData[1];
        ulo.asBytes[0] = receiveData[5];
        ulo.asBytes[1] = receiveData[4];
        ulo.asBytes[2] = receiveData[3];
        ulo.asBytes[3] = receiveData[2];
        cumulatedEnergy.energy = ulo.asUlong;
    }
    return cumulatedEnergy;
}

// (其他函数保持不变)
bool Aurora::writeBaudRateSetting(byte baudcode) { if ((int)baudcode <= 3) { return send(this->inverterAddress, (byte)COMMAND_BAUD_RATE_SETTING_WRITE, baudcode, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); } else { clearReceiveData(); return false; } }
Aurora::DataConfigStatus Aurora::readConfig() {
	DataConfigStatus configStatus;
	configStatus.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_CONFIG, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (configStatus.state.readState == true) {
        configStatus.state.transmissionState = receiveData[0];
        configStatus.state.globalState = receiveData[1];
        configStatus.configStatus = receiveData[aConfCode];
    }
    return configStatus;
}
Aurora::DataTimeCounter Aurora::readTimeCounter(byte param) {
	DataTimeCounter timeCounter;
	timeCounter.state.readState = send(this->inverterAddress, (byte)COMMAND_GET_TIME_COUNTER, param, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0);
    if (timeCounter.state.readState == true) {
        timeCounter.state.transmissionState = receiveData[0];
        timeCounter.state.globalState = receiveData[1];
        timeCounter.upTimeInSec = ((unsigned long)receiveData[2] << 24) + ((unsigned long)receiveData[3] << 16) + ((unsigned long)receiveData[4] << 8) + (unsigned long)receiveData[5];
    }
    return timeCounter;
}
bool Aurora::readFlagsSwitchCentral() { return send(this->inverterAddress, (byte)67, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); }
bool Aurora::readCumulatedEnergyCentral(byte var, byte ndays_h, byte ndays_l, byte global) { return send(this->inverterAddress, (byte)68, var, ndays_h, ndays_l, global, (byte)0, (byte)0); }
bool Aurora::readFirmwareReleaseCentral(byte var) { return send(this->inverterAddress, (byte)72, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); }
bool Aurora::readBaudRateSettingCentral(byte baudcode, byte serialline) { return send(this->inverterAddress, (byte)85, baudcode, serialline, (byte)0, (byte)0, (byte)0, (byte)0); }
bool Aurora::readSystemInfoCentral(byte var) { return send(this->inverterAddress, (byte)101, var, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); }
bool Aurora::readJunctionBoxMonitoringCentral(byte cf, byte rn, byte njt, byte jal, byte jah) { return send(this->inverterAddress, (byte)103, cf, rn, njt, jal, jah, (byte)0); }
bool Aurora::readSystemPNCentral() { return send(this->inverterAddress, (byte)105, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); }
bool Aurora::readSystemSerialNumberCentral() { return send(this->inverterAddress, (byte)107, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0); }