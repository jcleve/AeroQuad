//MAVLINK_SERIAL -> AQv3.2 Serial2 TX


// MavLink 1.0 DKP
#include "C:\Users\john.cleve\Documents\GitHub\AeroQuad\Libraries\mavlink\include\mavlink\v1.0\common\mavlink.h"

#define MAVLINK_SERIAL   Serial1
#define DEBUG_SERIAL     Serial

uint8_t     MavLink_Connected;
uint16_t  hb_count;

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;

int LED = 13;

// ******************************************
// Message #0  HEARTHBEAT 
uint8_t      aq_type = 0;
uint8_t      aq_autopilot = 0;
uint8_t      aq_base_mode = 0;
int32_t      aq_hdop = 0;
uint32_t     aq_custom_mode = 0;
uint8_t      aq_system_status = 0;
uint8_t      aq_mavlink_version = 0;

// Message # 1  SYS_STATUS 
uint16_t     aq_voltage_battery = 0;    // 1000 = 1V
int16_t      aq_current_battery = 0;    //  10 = 1A

// Message #24  GPS_RAW_INT 
uint8_t    aq_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    aq_sat_visible = 0;           // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    aq_latitude = 0;              // 585522540;
int32_t    aq_longitude = 0;            // 162344467;
int32_t    aq_gps_altitude = 0;        // 1000 = 1m
int32_t    aq_gps_speed = 0;
uint16_t  aq_gps_heading = 0;
// ******************************************
// These are special for FrSky
int32_t   adc2 = 0;               // 100 = 1.0V
int32_t     vfas = 0;                // 100 = 1,0V
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype
// ex. 83 = 8 sattelites visible, 3D lock

// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    ap_latitude = 0;              // 585522540;
int32_t    ap_longitude = 0;            // 162344467;
int32_t    ap_gps_altitude = 0;        // 1000 = 1m
int32_t    ap_gps_speed = 0;

uint8_t c;
long system_dropped_packets = 0;

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;

void setup()
{
  MAVLINK_SERIAL.begin(115200); //correct speed?
  DEBUG_SERIAL.begin(115200);
}

void loop()
{
  if(!MavLink_Connected)
  {
    digitalWrite(LED, LOW);
  }
  readMAVLink();
}
void readMAVLink() {
  while(MAVLINK_SERIAL.available() > 0) {

    c = MAVLINK_SERIAL.read();
    //try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      switch(msg.msgid) {

        // Message Types:  

      case MAVLINK_MSG_ID_HEARTBEAT: //0
        //aq_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg)& 0x80) > 7;
        //aq_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
        MavLink_Connected_timer=millis(); 
        if(!MavLink_Connected); 
        {
          hb_count++;   
          if((hb_count++) > 10) {        // If  received > 10 heartbeats from MavLink then we are connected
            MavLink_Connected=1;
            hb_count=0;
            digitalWrite(LED,HIGH);      // LED will be ON when connected to MavLink, else it will slowly blink
          }
        }     
        DEBUG_SERIAL.println("Heartbeat"); 
        //DEBUG_SERIAL.println(aq_base_mode);
        //DEBUG_SERIAL.println(aq_custom_mode);        
        break;

      case MAVLINK_MSG_ID_SYS_STATUS :   // 1
        aq_voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1 = 1mV
        aq_current_battery = mavlink_msg_sys_status_get_current_battery(&msg);     // 1=10mA

        //storeVoltageReading(ap_voltage_battery);
        //storeCurrentReading(ap_current_battery);
        //        uint8_t temp_cell_count;
        //        if(aq_voltage_battery > 21000) temp_cell_count = 6;
        //        else if (ap_voltage_battery > 17500) temp_cell_count = 5;
        //        else if(ap_voltage_battery > 12750) temp_cell_count = 4;
        //        else if(ap_voltage_battery > 8500) temp_cell_count = 3;
        //        else if(ap_voltage_battery > 4250) temp_cell_count = 2;
        //        else temp_cell_count = 0;
        //        if(temp_cell_count > ap_cell_count)
        //          ap_cell_count = temp_cell_count;
        DEBUG_SERIAL.println("System Status");
        DEBUG_SERIAL.println(aq_voltage_battery);
        DEBUG_SERIAL.println(aq_current_battery);
        break;

      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:   // 24
        //aq_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
        //aq_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
        //aq_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);                // GPS H.DOP 
        //gps_status = aq_sat_visible;
        //if(aq_fixtype == 3)  {
          aq_latitude = mavlink_msg_global_position_int_get_lat(&msg);
          aq_longitude = mavlink_msg_global_position_int_get_lon(&msg);
          aq_gps_altitude = mavlink_msg_global_position_int_get_relative_alt(&msg);    // 1m =1000
          //aq_gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg);         // 100 = 1m/s
          aq_gps_heading = mavlink_msg_global_position_int_get_hdg(&msg);
        //}
        //else
        //{
          ap_gps_speed = 0;  
        //}
        DEBUG_SERIAL.println("GPS RAW");
        
        DEBUG_SERIAL.println(aq_latitude);
        DEBUG_SERIAL.println(aq_longitude);
        DEBUG_SERIAL.println(aq_gps_altitude);
        DEBUG_SERIAL.println(aq_gps_heading);
        break;

      default:
        {
          DEBUG_SERIAL.println(c);
        }
        break;
      }
    }
  }
  system_dropped_packets += status.packet_rx_drop_count;
}







