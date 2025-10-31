/**
 * @file walter-energy-harvesting-spl-sensor.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 30 Oct 2025
 * @copyright DPTechnics bv
 * @brief Cellular sound level sensor
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 * 
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 * 
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 * 
 * This sketch uploads sound level data to the walter demo platform every 30 minutes.
 */

#include <esp_mac.h>
#include <inttypes.h>
#include <WalterModem.h>
#include <DecibelMeter.hpp>

/**
 * @brief The cellular APN, leave blank for automatic APN
 */
#define CELLULAR_APN "soracom.io"

/**
 * @brief The address of the server to upload the data to. 
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The size in bytes of the SPL sensor packet.
 */
#define PACKET_SIZE 20

/**
 * @brief The serial interface to talk to the modem.
 */
#define ModemSerial Serial2

/**
 * @brief The radio access technology to use - LTEM or NBIOT.
 */
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

/**
 * @brief I/O pin used to power the SPL meter.
 */
#define DB_METER_POWER 18

/**
 * @brief I/O pin used for I2C SDA with the SPL meter.
 */
#define DB_METER_SDA 10

/**
 * @brief I/O pin used for I2C SCL with the SPL meter.
 */
#define DB_METER_SCL 9

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The decibel meter instance.
 */
DecibelMeter dbmeter(DB_METER_SDA, DB_METER_SCL, 10000);

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[PACKET_SIZE] = { 0 };

/**
 * Request 2s active time and 1 hour sleep time.
 */
const char *psmActive = "00000001";
const char *psmTAU = "00100001";

/**
 * Set to 1 to enable logging, set to 0 disable logging.
 */
#define ENABLE_LOGGING 0

/**
 * @brief Log a line of text followed by a newline if enabled.
 */
#if ENABLE_LOGGING
  #define logln(x) Serial.println(x)
#else
  #define logln(x) do {} while (0)
#endif

/**
 * @brief Log a line of formatted text if enabled.
 */
#if ENABLE_LOGGING
  #define logf(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define logf(fmt, ...) do {} while (0)
#endif

/**
 * @brief Check if the modem has network connection.
 * 
 * This function checks if the modem has network connection.
 *
 * @return True when the modem has a network connection, false if not.
 */
bool lteConnected() {
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief Connect to the LTE network.
 *
 * This function will try to connect to the LTE network.
 * 
 * @return True when the connection attempt succeeded, false if not.
 */
bool lteConnect() {
  // Set the functionality level of the modem to minimum
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    logln("Could not set the modem to minimum functionality level");
    return false;
  }
  delay(500);

  // Create a PDP context with specified APN
  if (!modem.definePDPContext(1, CELLULAR_APN)) {
    logln("Could not create PDP context");
    return false;
  }
  logln("Attempting to connect to the network...");

  // Set the functionality level of the modem to full
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    logln("Error: Could not set the modem to full functionality level");
    return false;
  }

  delay(1000);

  // Set the network operator selection to automatic
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
      logln("Network selection mode to was set to automatic");
  } else {
      logln("Error: Could not set the network selection mode to automatic");
      return false;
  }

  // Wait (maximum 5 minutes) until successfully registered to the network
  unsigned short timeout = 300;
  unsigned short i = 0;
  while (!lteConnected() && i < timeout) {
    i++;
    delay(1000);
  }
  if (i >= timeout) {
    return false;
  }

  // Show cellular connection information
  WalterModemRsp rsp = {};
  if (modem.getRAT(&rsp)) {
    logf("Connected to %s ",
                  rsp.data.rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M");
    dataBuf[29] = (uint8_t) rsp.data.rat;
  }
  if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,
                               &rsp)) {
    logf("on band %u using operator %s (%u%02u)\r\n",
                  rsp.data.cellInformation.band,
                  rsp.data.cellInformation.netName, rsp.data.cellInformation.cc,
                  rsp.data.cellInformation.nc);
    logf("Signal strength: RSRP: %.2f, RSRQ: %.2f\r\n",
                  rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
  }

  return true;
}

/**
 * @brief Connect to an UDP socket.
 * 
 * This function will set-up the modem and connect an UDP socket. The LTE 
 * connection must be active before this function can be called.
 * 
 * @param ip The IP address of the server to connect to.
 * @param port The port to connect to.
 * 
 * @return True on success, false on error.
 */
bool socketConnect(const char *ip, uint16_t port)
{
  /* Construct a socket */
  if(modem.socketConfig()) {
    logln("Created a new socket");
  } else {
    logln("Could not create a new socket");
    return false;
  }

  /* disable socket tls as the demo server does not use it */
  if(modem.socketConfigSecure(false)) {
    logln("Disabled TLS");
  } else {
    logln("Could not configure TLS");
    return false;
  }

  /* Connect to the UDP test server */
  if(modem.socketDial(ip, port)) {
    logln("Connected to UDP server");
  } else {
    logln("Could not connect UDP socket");
    return false;
  }

  return true;
}

/**
 * @brief Set up the system.
 * 
 * This function will set up the system and initialize the modem.
 * 
 * @return None.
 */
void setup()
{
#if ENABLE_LOGGING
  Serial.begin(115200);
  delay(1000);
#endif

  logln("Energy Harvesting SPL sensor V0.0.1");

  /* Enable 3.3V power to switch on the decibel meter */
  pinMode(DB_METER_POWER, OUTPUT);
  digitalWrite(DB_METER_POWER, HIGH);
  delay(50);

  dbmeter.begin();
  //logf("Decibel meter version: 0x%02X\n", dbmeter.getVersion());

  /* Configure 1000ms averaging and reset min/max */
  dbmeter.setAveragingInterval(1000);
  dbmeter.resetMinMax();

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  logf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  /* Modem initialization */
  if(modem.begin(&ModemSerial)) {
    logln("Modem initialization OK");
  } else {
    logln("Modem initialization ERROR");
    esp_sleep_enable_timer_wakeup(1000000);
    esp_light_sleep_start();
    ESP.restart();
    return;
  }

  modem.configPSM(WALTER_MODEM_PSM_ENABLE, psmTAU, psmActive);

  /* Connect to cellular network */
  if (!lteConnected() && !lteConnect()) {
    logln("Error: Unable to connect to cellular network, restarting Walter in 10 seconds");
    esp_sleep_enable_timer_wakeup(10000000);
    esp_light_sleep_start();
    ESP.restart();
  }

  static WalterModemRsp rsp = {};

  /* Read the decibel meter's values */
  uint8_t db = dbmeter.readDecibel();
  uint8_t dbmin = dbmeter.readMinDecibel();
  uint8_t dbmax = dbmeter.readMaxDecibel();
  logf("dB = %03d [MIN: %03d, MAX: %03d]\n", db, dbmin, dbmax);

  /* Reset the min/max registers */
  dbmeter.resetMinMax();

  /* Construct the decibel meter sensor packet */
  dataBuf[6] = db;
  dataBuf[7] = dbmin;
  dataBuf[8] = dbmax;

  if(!socketConnect(SERV_ADDR, SERV_PORT)) {
    logln("Could not connect to UDP server socket");
    esp_sleep_enable_timer_wakeup(1000000);
    esp_light_sleep_start();
    ESP.restart();
    return;
  }

  /* Workaround to wake-up the modem from PSM */
  if(!modem.socketSend(dataBuf, 1)) {
    logln("Could not transmit data");
    delay(1000);
    ESP.restart();
    return;
  }

  esp_sleep_enable_timer_wakeup(1000000);
  esp_light_sleep_start();

  /* Try up to five times to read signal strength */
  for(int i = 0; i < 5; ++i) { 
    if(!modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
      logln("Could not request cell information");
    } else {
      logf("Connected on band %u using operator %s (%u%02u)",
        rsp.data.cellInformation.band, rsp.data.cellInformation.netName,
        rsp.data.cellInformation.cc, rsp.data.cellInformation.nc);
      logf(" and cell ID %u.\r\n",
        rsp.data.cellInformation.cid);
      logf("Signal strength: RSRP: %.2f, RSRQ: %.2f.\r\n",
        rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
    }

    if(rsp.data.cellInformation.rsrp < -20) { 
      break;
    }

    esp_sleep_enable_timer_wakeup(1000000);
    esp_light_sleep_start();
  }

  /* Add monitor data to packet */
  dataBuf[9] = rsp.data.cellInformation.cc >> 8;
  dataBuf[10] = rsp.data.cellInformation.cc & 0xFF;
  dataBuf[11] = rsp.data.cellInformation.nc >> 8;
  dataBuf[12] = rsp.data.cellInformation.nc & 0xFF;
  dataBuf[13] = rsp.data.cellInformation.tac >> 8;
  dataBuf[14] = rsp.data.cellInformation.tac & 0xFF;
  dataBuf[15] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
  dataBuf[16] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
  dataBuf[17] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
  dataBuf[18] = rsp.data.cellInformation.cid & 0xFF;
  dataBuf[19] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);
  
  if(!modem.socketSend(dataBuf, PACKET_SIZE)) {
    logln("Could not transmit data");
    esp_sleep_enable_timer_wakeup(1000000);
    esp_light_sleep_start();
    ESP.restart();
    return;
  }

  esp_sleep_enable_timer_wakeup(5000000);
  esp_light_sleep_start();

  if(!modem.socketClose()) {
    logln("Could not close the socket");
    esp_sleep_enable_timer_wakeup(1000000);
    esp_light_sleep_start();
    ESP.restart();
    return;
  }

  // Go to deep sleep for 30 minutes - 11 seconds 
  modem.sleep(30 * 60 - 11);
}

void loop()
{
  delay(10000);
  ESP.restart();
}
