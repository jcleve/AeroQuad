
#include "../mavlink/include/mavlink/v1.0/common/mavlink.h"

 
// Example variable, by declaring them static they're persistent
// and will thus track the system state
static int packet_drops = 0;
static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */
 
/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
static void communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status;
 
	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
 
	while(uart0_char_available())
	{
		uint8_t c = uart0_get_char();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
 
			switch(msg.msgid)
			{
			        case MAVLINK_MSG_ID_HEARTBEAT:
			        {
				  // E.g. read GCS heartbeat and go into
                                  // comm lost mode if timer times out
			        }
			        break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				// EXECUTE ACTION
				break;
			default:
				//Do nothing
				break;
			}
		}
 
		// And get the next one
	}
 
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
 
	// COMMUNICATION THROUGH SECOND UART
 
	while(uart1_char_available())
	{
		uint8_t c = uart1_get_char();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
		{
			// Handle message the same way like in for UART0
                        // you can also consider to write a handle function like
                        // handle_mavlink(mavlink_channel_t chan, mavlink_message_t* msg)
                        // Which handles the messages for both or more UARTS
		}
 
		// And get the next one
	}
 
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
}
