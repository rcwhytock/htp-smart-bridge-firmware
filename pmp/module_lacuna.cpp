#ifndef REGION
#define REGION R_EU868
#endif

#include "module_lacuna.h"

#include "debug.h"
#include "secrets.h"

#ifdef MODULE_LACUNA_DEBUG
#define NAME "lacuna"
#define serial_debug Serial
#endif

static char payload[255];

#define MIN_RSSI 98

scheduler_result_t MODULE_LACUNA::scheduler(void) {
    if (global_activate_pira != INACTIVE) {
        return DONT_RUN;
    }

#ifdef serial_debug
    serial_debug.print(NAME);
    serial_debug.print(": scheduler(");
    serial_debug.println("active)");
#endif
    return {5000, false};
}

void MODULE_LACUNA::initialize(void) {}

void MODULE_LACUNA::running(void) {
    // Turn on power for lacuna modem
    pinMode(MODULE_LACUNA_5V, OUTPUT);
    digitalWrite(MODULE_LACUNA_5V, HIGH);

    setup_lacuna();
    receive_lacuna();

    // Turn off power for lacuna modem
    digitalWrite(MODULE_LACUNA_5V, LOW);
}

void MODULE_LACUNA::setup_lacuna(void) {
    // SX1262 configuration for lacuna LS200 board
    lsSX126xConfig cfg;
    lsCreateDefaultSX126xConfig(&cfg);

    cfg.nssPin = PB12;
    cfg.resetPin = -1;  // Not needed
    cfg.antennaSwitchPin = PB2;
    cfg.busyPin = PH1;
    cfg.dio1Pin = PB7;
    cfg.osc = lsSX126xOscTCXO;      // for Xtal: lsSX126xOscXtal
    cfg.type = lsSX126xTypeSX1262;  // for SX1261: lsSX126xTypeSX1261

    // Initialize SX1262
    int result = lsInitSX126x(&cfg);

    if (!result) {
#ifdef serial_debug
        serial_debug.print("E22/SX1262: ");
        serial_debug.println(lsErrorToString(result));
#endif
    }

    uint8_t relay_networkKey[] = RELAY_NETWORKKEY;
    uint8_t relay_appKey[] = RELAY_APPKEY;
    uint8_t relay_deviceAddress[] = RELAY_DEVICEADDRESS;

    // LoRaWAN parameters for relay
    lsCreateDefaultLoraWANParams(&relay_loraWANParams, relay_networkKey, relay_appKey, relay_deviceAddress);

    // transmission parameters for Lacuna satellites
    lsCreateDefaultLoraSatTxParams(&SattxParams);

    // Override default Lacuna satellite parameters
    SattxParams.frequency = 862750000;
    SattxParams.power = 21;
    SattxParams.grid = lsLoraEGrid_3_9_khz;
    SattxParams.codingRate = lsLoraECodingRate_1_3;
    SattxParams.bandwidth = lsLoraEBandwidth_335_9_khz;
    ;
    SattxParams.nbSync = 4;
    SattxParams.hopping = 1;

    // transmission parameters for terrestrial LoRa
    lsCreateDefaultLoraTxParams(&txParams);

    // Override defult LoRa parameters
    txParams.spreadingFactor = lsLoraSpreadingFactor_11;
    txParams.power = 21;
    txParams.codingRate = lsLoraCodingRate_4_5;
    txParams.invertIq = false;
    txParams.frequency = 867300000;
    txParams.bandwidth = lsLoraBandwidth_125_khz;
    txParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;
    txParams.preambleLength = 8;

    // LoRa parameters for relay (receive)
    lsCreateDefaultLoraTxParams(&relayParams);

    // Make sure that SF and bandwith match with device that we will relay.
    relayParams.spreadingFactor = lsLoraSpreadingFactor_9;
    relayParams.invertIq = false;
    relayParams.frequency = 868500000;
    relayParams.bandwidth = lsLoraBandwidth_125_khz;
    relayParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;
}

/*!
 * @brief
 * @param[in]
 * @param[in]
 * @return
 */
void MODULE_LACUNA::receive_lacuna(void) {
    int32_t rxlength = lsReceiveLora(&relay_loraWANParams, &relayParams, global_relay_payload);

    if (rxlength > 0) {
        /* valid relay data received */
#ifdef serial_debug
        serial_debug.print(NAME);
        serial_debug.print(": message(");
        serial_debug.println("Valid uplink received!)");
        serial_debug.print("        SNR: ");
        serial_debug.println(relayParams.snr);
        serial_debug.print("        RSSI: ");
        serial_debug.println(relayParams.rssi);
        serial_debug.print("        SignalRSSI: ");
        serial_debug.println(relayParams.signalrssi);
        serial_debug.print("        Payload in hex: ");
        printHex(global_relay_payload, rxlength, serial_debug);
        serial_debug.println();
        serial_debug.print("        Device ID in hex: ");
        printHex(getDeviceId(), 4, serial_debug);
        serial_debug.println();
#endif

        global_relay_payload_size = rxlength;

        if (MIN_RSSI == 0 || relayParams.rssi > -MIN_RSSI) {
            global_activate_pira = LORA;
        }
    }
}
/*** end of file ***/
