
// 20161218 - Migrated to production
// 20170613 - Updated to use Dallas sensor and CayenneLPP
// 20171119 - Alex Nijmeijer for AiD: Removed Dallas and Caynenne functionality. Added Sharp DustSensor readout, OTAA activation
// 20171126 - Added use of LowPower Library. Payload format now defined in this file.


/**
 *
 * To decode the binary payload, you can use the following
 * javascript decoder function. It should work with the TTN console.
 *
 function Decoder(bytes, port) {
   if (bytes[0]==0xA1) { // Apeldoorn 1n Data?
     if (bytes[1]==0xD0) { // Dust Sensor ?
       var bits = bytes[2]<<24 | bytes[3]<<16 | bytes[4]<<8 | bytes[5];
       var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
       var exponent = bits>>>23 & 0xff;
       var mantisse = (exponent === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
       var floating_point = sign * mantisse * Math.pow(2, exponent - 127) / 0x800000;
       return {
         dust_density_ug_m3: floating_point
       } ;
      }
    } 
  }
 */

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * 
 * To use this sketch, first register your application and device with
   the things network. Enter the APPEUI, DEVEUI and APPKEY below.
   Each device should have their own unique values for these
   fields.

   To use OTAA: uncomment DISABLE_JOIN in the lmic/config.h file (usually found in
   C:\Users\[Your_UserName]\Documents\Arduino\libraries\arduino-lmic\src\lmic)

 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>

#define USE_OTAA 1
#define USE_SENSOR_FAN 0            /* 1: enable an air-fan just before the measurements */
#define SENSOR_CONTINUOUS_READOUT 0 /* 1: for standalone sensor testing */

#ifdef USE_OTAA == 1
  // Alex OTAA DUST SENSOR
  // This EUI must be in little-endian format, so least-significant-byte
  // first (LSB..MSB). When copying an EUI from ttnctl output, this means to reverse
  // the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
  // The same for all Dust-sensor-nodes
  static const u1_t PROGMEM APPEUI[8] = { 0x17, 0x8B, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; 

  // This should also be in little endian format, see above (LSB..MSB)
  // incremental number for TTN-APLD-Dust sensors
  static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 

  // This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does not really apply). In
  // practice, a key taken from ttnctl can be copied as-is (MSB..LSB).
  static const u1_t PROGMEM APPKEY[16] = { 0x16, 0xA3, 0x4B, 0x11, 0xD3, 0x21, 0x66, 0x65, 0x0C, 0x61, 0xC4, 0x30, 0x37, 0xF3, 0x01, 0x45 }; 

  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }
#else
// ABP code here
#endif


/******* Setup LoRaWAN stack LMIC 1.5 ********/

// Verifications on LMIC configuration
#ifndef DISABLE_JOIN
 // #error "DISABLE_JOIN not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_PING
  #error "DISABLE_PING not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_BEACONS
  #error "DISABLE_BEACONS not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_MCMD_PING_SET
  #error "DISABLE_MCMD_PING_SET not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_MCMD_BCNI_ANS
  #error "DISABLE_MCMD_BCNI_ANS not defined in LMIC, add define!"
#endif 

#ifdef DISABLE_INVERT_IQ_ON_RX
  #error "DISABLE_INVERT_IQ_ON_RX defined in LMIC, remove define!"
#endif 



static uint8_t mydata[] = "Hello World";
static uint8_t mydata_size = 0;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60; // Schedule TX every this many seconds (might become longer due to duty cycle limitations).
bool isSending = false; /// Variable to keep track of transmission status of LMIC

// Hardware configuration for RFM95 TTN Apeldoorn node
// DIO0 is required, DIO1 is required for LoRa, DIO2 for FSK
#define LMIC_NSS    10
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    LMIC_UNUSED_PIN
#define LMIC_DIO0   4
#define LMIC_DIO1   5
#define LMIC_DIO2   7

const lmic_pinmap lmic_pins = {
    .nss  =  LMIC_NSS,
    .rxtx =  LMIC_RXTX,   
    .rst  =  LMIC_RST,
    .dio  = {LMIC_DIO0, 
             LMIC_DIO1, 
             LMIC_DIO2},  
};

#define TTNAPELDOORN_BOARD 1
#if TTNAPELDOORN_BOARD == 0
// Hardware config for Alex Dust sensor setup
const unsigned dust_pins_enable         = A5;
const unsigned dust_pins_analog_voltage = A3;
const unsigned dust_pins_timing_check   = A4;  // To validate the sample moment with respect to the output voltage of the sensor
const unsigned enable_fan_sensor        = PD2; //  
#else
// Hardware config for TTNApeldoorn Dust sensor setup
const unsigned dust_pins_enable         = 8;   // D8;
const unsigned dust_pins_analog_voltage = A0;
const unsigned dust_pins_timing_check   = 9 ;  // D9; // To validate the sample moment with respect to the output voltage of the sensor
const unsigned enable_fan_sensor        = A1;  //  
#endif

/**********************************/

byte LMIC_transmitted = 0;
byte LMIC_event_Timeout = 0;


void setup() 
{
    Serial.begin(115200);
    Serial.println(F("Starting"));
    sharpInit();
    
#if SENSOR_CONTINUOUS_READOUT != 0
    while (1) {
      sharpGetDustDensity();
      delay (3000); // 1 sec between measurements
    }
#endif
  
    sharpGetDustDensity(); // single dummy read on startup
  
    // LMIC init
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
    //LMIC_setDrTxpow(DR_SF12,14);
    //LMIC_setDrTxpow(DR_SF11,14);

    // Disable ADR
    LMIC_setAdrMode(1);
    // Disable link check validation (shall be enabled when ADR is active
    LMIC_setLinkCheckMode(1);
    
    // Correction to optimize recepetion of downlink traffic
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job
    do_send(&sendjob);
}

void loop() {
  os_runloop_once();            // Wait for response of the queued message (check if message is send correctly)

  if(!isSending)
  {
    Serial.println(F("ZZ"));  /// Send to the world tha we will go to sleep
    delay(10);            /// Delay before going to sleep to allow serial to send.
      
    int i = 0;
    while(i < TX_INTERVAL)
    {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
      i += 8; // increment time with sleep time of 8 seconds.
    }

    Serial.println(F("HI"));
      
    // Start transmission now
    os_setCallback( &sendjob, do_send );
  }
}

/// \brief on-event function for LMIC
/// \param Event struct
void onEvent (ev_t ev) {
    switch(ev) {
//      case EV_SCAN_TIMEOUT:   Serial.println(F("EV_SCAN_TIMEOUT"));   break;
//      case EV_BEACON_FOUND:   Serial.println(F("EV_BEACON_FOUND"));   break;
//      case EV_BEACON_MISSED:  Serial.println(F("EV_BEACON_MISSED"));  break;
//      case EV_BEACON_TRACKED: Serial.println(F("EV_BEACON_TRACKED")); break;
        case EV_JOINING:        Serial.println(F("EV_JOINING"));        break;
        case EV_JOINED:         Serial.println(F("EV_JOINED"));         break;
//      case EV_RFU1:           Serial.println(F("EV_RFU1"));           break;
        case EV_JOIN_FAILED:    Serial.println(F("EV_JOIN_FAILED"));    break;
        case EV_REJOIN_FAILED:  Serial.println(F("EV_REJOIN_FAILED"));  break;
        case EV_TXCOMPLETE:     Serial.println(F("RXComplete"));
            if (LMIC.txrxFlags & TXRX_ACK)
            {
              Serial.println(F("RXAck"));
            }
            if (LMIC.dataLen) 
            {
               Serial.print("RXPayload ");
               Serial.print(LMIC.dataLen);
               Serial.print(" bytes\n");
               Serial.print("Payload: ");
               for (int j = 0; j < LMIC.dataLen; j++)
               {
                  Serial.print(LMIC.frame[LMIC.dataBeg + j], HEX);
               }
               Serial.print("\n");
               //received = true;
      }
            // Signal that transmission is completed
            isSending = false;
            break;
//      case EV_LOST_TSYNC:   Serial.println(F("EV_LOST_TSYNC"));       break;
//      case EV_RESET:        Serial.println(F("EV_RESET"));            break;
//      case EV_RXCOMPLETE:   Serial.println(F("EV_RXCOMPLETE"));       break;
//      case EV_LINK_DEAD:    Serial.println(F("EV_LINK_DEAD"));        break;
//      case EV_LINK_ALIVE:   Serial.println(F("EV_LINK_ALIVE"));       break;
//      default:              Serial.println(F("Unknown event"));       break;
        default:              Serial.println(F("Other event!"));        break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
      Serial.println(F("TXRXPEND"));
    } 
    else
    {

      AiD_add_data_concentration_ug_m3(sharpGetDustDensity());
      isSending = true;
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1,mydata, mydata_size, 0); // port 1
      
   
      Serial.println(F("TX"));
    }
}


float sharpGetDustDensity() {
  // if we have a sensor FAN, power it for a few seconds before we enable the sensor, and take the measurement sample
  #if USE_SENSOR_FAN != 0
     digitalWrite(enable_fan_sensor, HIGH);
     delay(3000); //3 sec
  #endif   
    
    // Enable the LED in the sensor. 
    digitalWrite (dust_pins_enable, LOW) ; // drive pin low
    pinMode (dust_pins_enable, OUTPUT) ;
    
    // wait until we can take the sample (the analog voltage takes some time to settle)
    delayMicroseconds(150); //  0.28ms Wait until we can measure the sensor output. tuned such that the timing-check pulse is around the peak of the analog output
    
    // take the sample (use the timing-check pin to validate the sample moment on a scope)
    digitalWrite(dust_pins_timing_check, HIGH);
    float voltage =  analogRead (dust_pins_analog_voltage);
    digitalWrite(dust_pins_timing_check, LOW);
    
    delayMicroseconds(17); //  wait a bit before we disable the sensor (either by floating the output, or pulling it HIGH)
    #if 1
      digitalWrite (dust_pins_enable, HIGH) ; // drive pin high before making it input -> much smaller rise-time
      // delayMicroseconds(100);  
    #else
      pinMode (dust_pins_enable, INPUT) ; // => HIGH , pulled up by sensor. takes a while >10ms before it reaches 3V, but forcing/driving the pin high before making int INPUT does noet help
    #endif
    
    // Do the conversion to ug/m3  and print results to the terminal
    Serial.print(voltage); Serial.print("\n");
 
    voltage = voltage * 3.3 / 1024; // DAC scaling
    voltage = voltage / 33 * (33 + 10); // External Resistor divider (= 33k / (33k + 10k))
    Serial.print("Sensor voltage:"); Serial.print(voltage ); Serial.print("\n");
    float DustDensity_ugm3 = (voltage - 0.58) / 5.9 * 1000 + 50; // add 60 to avoid negative numbers. Does not make sense, but neither do negative concentrations
    Serial.print("density:"); Serial.print(DustDensity_ugm3 ); Serial.print("\n");

  // Disable the sensor, when we have one
  #if USE_SENSOR_FAN != 0
    digitalWrite(enable_fan_sensor, LOW);
  #endif  
    
    return DustDensity_ugm3;
}

void sharpInit() {
  digitalWrite (dust_pins_enable, HIGH) ; // drive pin high (disabled)
  pinMode (dust_pins_enable, OUTPUT) ;
  pinMode (dust_pins_enable, INPUT) ; // => HIGH , pulled up by sensor

  pinMode (dust_pins_timing_check, OUTPUT) ;
  digitalWrite(dust_pins_timing_check, LOW);
 
  pinMode(dust_pins_analog_voltage, INPUT);
  analogRead (dust_pins_analog_voltage) ; // do a dummy read to get the pin in the right state?


  pinMode (enable_fan_sensor, OUTPUT) ;
  digitalWrite(enable_fan_sensor, LOW);
  
}

int AiD_add_data_concentration_ug_m3 (float value) {
  union {
    uint32_t a_int;
    float a_float;
  } convert;
  
  convert.a_float=value;
  mydata[0] = 0xA1; // Apeldoorn In data
  mydata[1] = 0xD0; // Dust sensor ug_m3
  mydata[2] = (convert.a_int>>24) & 0xFF;
  mydata[3] = (convert.a_int>>16) & 0xFF;
  mydata[4] = (convert.a_int>>8) & 0xFF;
  mydata[5] = (convert.a_int>>0) & 0xFF;
  mydata_size = 6;
  return mydata_size;
}
