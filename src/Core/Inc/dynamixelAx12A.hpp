#ifndef __DYNAMIXEL_AX_12A_HPP__
#define __DYNAMIXEL_AX_12A_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "string.h"
#include "string.h"
#include "stdio.h"

#ifdef __cplusplus
}
#endif

/*	The structure of the instruction packet:
 *
 *  [0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMETERS][CHECKSUM]
 *
 *  The structure of the status packet (Return packet):
 *
 *  [0xFF][0xFF][ID][LENGTH][ERROR][PARAMETERS][CHECKSUM]
 *
 */

/*
 * 	READ DATA
 *
 *  Function 	: Read data from the control table of a Dynmixel actuar
 *  Length   	: 0x04
 *	Instruction : 0x02
 *	Parameter1	: Starting address of the location where the data is to be read
 *	Parameter2	: Length of the data to be read
 *
 */

//////////////////////////////////////////////////
// Header Macro
#define HEADER_FIRST							0xFF
#define HEADER_SECOND							0xFF

// Header Macro
///////////////////////////////////////////////////

///////////////////////////////////////////////
// EEPROM Address Macro

#define MODEL_NUMBER_L_ADR 							0x00 // Access : RD , -  Initial Value: 12  (0x0C)
#define MODEL_NUMBER_H _ADR							0x01 // Access : RD , -  Initial Value: 0   (0x00)
#define VERSION_FIRMWARE_ADR						0x02 // Access : RD , -  Initial Value:	?
#define ID_ADR										0x03 // Access : RD , WR Initial Value: 1   (0x01)
#define BAUDRATE_ADR								0x04 // Access : RD , WR Initial Value: 1   (0x01)
#define DELAY_TIME_ADR								0x05 // Access : RD , WR Initial Value: 250 (0xFA)
#define CW_ANGLE_LIMIT_L_ADR						0x06 // Access : RD , WR Initial Value: 0   (0x00)
#define CW_ANGLE_LIMIT_H_ADR						0x07 // Access : RD , WR Initial Value: 0   (0x00)
#define CCW_ANGLE_LIMIT_L_ADR						0x08 // Access : RD , WR Initial Value: 255 (0xFF)
#define CCW_ANGLE_LIMIT_H_ADR						0x09 // Access : RD , WR Initial Value: 3   (0x03)
#define HIGH_TEMP_LIMIT_ADR							0x0B // Access : RD , WR Initial Value: 85  (0x55)
#define LOW_VOLT_LIMIT_ADR							0x0C // Access : RD , WR Initial Value: 60  (0x3C)
#define HIGH_VOLT_LIMIT_ADR							0x0D // Access : RD , WR Initial Value: 190 (0xBE)
#define MAX_TORQUE_LIMIT_L_ADR						0x0E // Access : RD , WR Initial Value: 255 (0xFF)
#define MAX_TORQUE_LIMIT_H_ADR						0x0F // Access : RD , WR Initial Value: 3   (0x03)
#define STATUS_RETURN_LEVEL_ADR						0x10 // Access : RD , WR Initial Value: 2   (0x02)
#define ALARM_LED_ADR								0x11 // Access : RD , WR Initial Value: 4   (0x04)
#define ALARM_SHUTDOWN_ADR							0x12 // Access : RD , WR Initial Value: 4   (0x04)
#define DOWN_CALIBRATION_L_ADR						0x14 // Access : RD , -  Initial Value: ?
#define DOWN_CALIBRATION_H_ADR						0x15 // Access : RD , -  Initial Value: ?
#define UP_CALIBRATION_L_ADR						0x16 // Access : RD , -  Initial Value: ?

// EEPROM Address Macro
///////////////////////////////////////////////

///////////////////////////////////////////////
// RAM Address Macro

#define UP_CALIBRATION_H_ADR						0x17 // Access : RD , -  Initial Value: ?
#define TORQUE_ENABLE_ADR							0x18 // Access : RD , WR Initial Value: 0 (0x00)
#define LED_ADR										0x19 // Access : RD , WR Initial Value: 0 (0x00)
#define CW_COMPLIANCE_MARGIN_ADR					0x1A // Access : RD , WR Initial Value: 0 (0x00)
#define CCW_COMPLIANCE_MARGIN_ADR					0x1B // Access : RD , WR Initial Value: 0 (0x00)
#define CW_COMPLIANCE_SLOPE_ADR						0x1C // Access : RD , WR Initial Value: 0 (0x00)
#define CCW_COMPLIANCE_SLOPE_ADR					0x1D // Access : RD , WR Initial Value: 0 (0x00)
#define GOAL_POSITION_L_ADR							0x1E // Access : RD , WR Initial Value: [ADDR36]value
#define GOAL_POSITION_H_ADR							0x1F // Access : RD , WR Initial Value: [ADDR37]value
#define MOVING_SPEED_L_ADR							0x20 // Access : RD , WR Initial Value: 0 (0x00)
#define MOVING_SPEED_H_ADR							0x21 // Access : RD , WR Initial Value: 0 (0x00)
#define TORQUE_LIMIT_L_ADR							0x22 // Access : RD , WR Initial Value: [ADDR14]value
#define TORQUE_LIMIT_H_ADR							0x23 // Access : RD , WR Initial Value: [ADDR15]value
#define PRESENT_POSITION_L_ADR						0x24 // Access : RD , -  Initial Value: ?
#define PRESENT_POSITION_H_ADR						0x25 // Access : RD , -  Initial Value: ?
#define PRESENT_SPEED_L_ADR							0x26 // Access : RD , -  Initial Value: ?
#define PRESENT_SPEED_H_ADR							0x27 // Access : RD , -  Initial Value: ?
#define PRESENT_LOAD_L_ADR							0x28 // Access : RD , -  Initial Value: ?
#define PRESENT_LOAD_H_ADR							0x29 // Access : RD , -  Initial Value: ?
#define PRESENT_VOLTAGE_ADR							0x2A // Access : RD , -  Initial Value: ?
#define PRESENT_TEMP_ADR							0x2B // Access : RD , -  Initial Value: ?
#define REGISTERED_INSTRUCTION_ADR					0x2C // Access : RD , WR Initial Value: 0  (0x00)
#define MOVING_ADR									0x2E // Access : RD , -  Initial Value: 0  (0x00)
#define LOCK_ADR									0x2F // Access : RD , WR Initial Value: 0  (0x00)
#define PUNCH_L_ADR									0x30 // Access : RD , WR Initial Value: 0  (0x00)
#define PUNCH_H_ADR									0x31 // Access : RD , WR Initial Value: 0  (0x00)

// RAM Address Macro
///////////////////////////////////////////////

//////////////////////////////////////////////////
// Other collaborator macro

#define BUFFER_SIZE									100 // size of max msg size
#define COMMAND_RESPONSE_MSG_LENGTH					6   // length of response message to configuration messages

// Other collaborator macro
//////////////////////////////////////////////////

class dynamixel_ax_12A_parameters {

private:


	enum INSTRUCTION : uint8_t {

		PING = 0x01, 		 //  No action. Used for obtaining a Status Packet
		READ_DATA = 0x02,    //  Reading values in the Control Table
		WRITE_DATA = 0x03,   //  Writing values to the Control Table
		REG_WRITE = 0x04,	 //  Similar to W RITE_DATA, but stays in standby mode until the ACION instruction is given
		ACTION = 0x05,	     //  Triggers the action registered by the REG_WRITE instruction
		RESET = 0x06, 		 //  Changes the control table values of the Dynamixel actuator to the Factory Default Value settings
		SYNC_WRITE = 0x83 	 //  Used for controlling many Dynamixel actuators at the same time

	};

	UART_HandleTypeDef *uartType;

	uint8_t calculated_checksum_u8;


public:


	typedef enum{

		RIGHT = 0x01,
		LEFT = 0x02

	}Turn_direction_st;

	dynamixel_ax_12A_parameters(UART_HandleTypeDef *__uartType) {

		this->uartType = __uartType;
		this->calculated_checksum_u8 = 0;

	}

	////////////////////////////////////////////////////
	// Send Command Actuator

	void ping_actuator(uint8_t ID) {

		// create array
		uint8_t ping_packet_u8[6];

		// assign syncron bytes
		ping_packet_u8[0] = HEADER_FIRST;
		ping_packet_u8[1] = HEADER_SECOND;

		ping_packet_u8[2] = ID;   // Assign acuator id
		ping_packet_u8[3] = 0x02; // Assign packet length

		ping_packet_u8[4] = PING; // Assign instruction

		this->calculate_checksum(ping_packet_u8, 5); // calculate checksum

		ping_packet_u8[5] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(ping_packet_u8, 6); // send msg and get response


	}

	void set_ID(uint8_t ID, uint8_t new_ID) {

		// create array
		uint8_t set_id_msg_u8[8];

		// assign syncron bytes
		set_id_msg_u8[0] = HEADER_FIRST;
		set_id_msg_u8[1] = HEADER_SECOND;

		set_id_msg_u8[2] = ID;   // Assign acuator id
		set_id_msg_u8[3] = 0x04; // Assign packet length

		set_id_msg_u8[4] = WRITE_DATA; // Assign instruction
		set_id_msg_u8[5] = ID_ADR; // Assign ID addr
		set_id_msg_u8[6] = new_ID; // Assign parameter : NEW ID

		this->calculate_checksum(set_id_msg_u8, 7); // calculate checksum

		set_id_msg_u8[7] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_id_msg_u8, 8); // send msg and get response


	}

	void set_baudrate(uint8_t ID, uint32_t new_baudrate) {

		// create array
		uint8_t set_baudrate_msg[8];

		// assign syncron bytes
		set_baudrate_msg[0] = HEADER_FIRST;
		set_baudrate_msg[1] = HEADER_SECOND;

		set_baudrate_msg[2] = ID;   // Assign acuator id
		set_baudrate_msg[3] = 0x04; // Assign packet length

		set_baudrate_msg[4] = WRITE_DATA; // Assign instruction
		set_baudrate_msg[5] = BAUDRATE_ADR; // Assign baudrate addr

		set_baudrate_msg[6] = this->baudrate_to_value(new_baudrate); // Assign parameter : baudrate

		this->calculate_checksum(set_baudrate_msg, 7); // calculate checksum

		set_baudrate_msg[7] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_baudrate_msg, 8); // send msg and get response


	}

	void set_operation_range_ccw(uint8_t ID , float position_range){

		uint16_t pos = this->calculate_pos(position_range); // position limit calculate

		// create array
		uint8_t set_ccw_position_limit[9];

		// assign syncron bytes
		set_ccw_position_limit[0] = HEADER_FIRST;
		set_ccw_position_limit[1] = HEADER_SECOND;

		set_ccw_position_limit[2] = ID;    // Assign actuator id
		set_ccw_position_limit[3] = 0x05;  // Assign packet length

		set_ccw_position_limit[4] = WRITE_DATA; // Assign instruction
		set_ccw_position_limit[5] = CCW_ANGLE_LIMIT_L_ADR; // Assign limit ccw position first adr

		set_ccw_position_limit[6] = pos;		// Assign parameter position limit
		set_ccw_position_limit[7] = pos >> 8;	// Assign parameter position limit

		this->calculate_checksum(set_ccw_position_limit, 8); // calculate checksum

		set_ccw_position_limit[8] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_ccw_position_limit, 9); // send msg and get response



	}

	void set_operation_range_cw(uint8_t ID , float position_range){

		uint16_t pos = this->calculate_pos(position_range); // position limit calculate

		// create array
		uint8_t set_cw_position_limit[9];

		// assign syncron bytes
		set_cw_position_limit[0] = HEADER_FIRST;
		set_cw_position_limit[1] = HEADER_SECOND;

		set_cw_position_limit[2] = ID;    // Assign actuator id
		set_cw_position_limit[3] = 0x05;  // Assign packet length

		set_cw_position_limit[4] = WRITE_DATA; // Assign instruction
		set_cw_position_limit[5] = CW_ANGLE_LIMIT_L_ADR; // Assign limit ccw position first adr

		set_cw_position_limit[6] = pos;		// Assign parameter position limit
		set_cw_position_limit[7] = pos >> 8;	// Assign parameter position limit

		this->calculate_checksum(set_cw_position_limit, 8); // calculate checksum

		set_cw_position_limit[8] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_cw_position_limit, 9); // send msg and get response


	}

	void set_endless_turn(uint8_t ID, float velocity_percent,Turn_direction_st direction){

		this->set_operation_range_ccw(ID, 0);
		this->set_operation_range_cw(ID, 0);

		uint16_t velocity = this->calculate_endless_turn_vel(velocity_percent); // velocity percent calculate

		// Set direction via 10th bit
		if(direction == RIGHT){
			velocity |= (1UL<<10);
		}

		else if(direction == LEFT){
			velocity &= ~(0UL<<10);
		}
		else{
			velocity = 0b0000011111111111;
		}

		// create array
		uint8_t set_endless_turn[9];

		// assign syncron bytes
		set_endless_turn[0] = HEADER_FIRST;
		set_endless_turn[1] = HEADER_SECOND;

		set_endless_turn[2] = ID;    // Assign actuator id
		set_endless_turn[3] = 0x05;  // Assign packet length

		set_endless_turn[4] = WRITE_DATA; // Assign instruction
		set_endless_turn[5] = MOVING_SPEED_L_ADR; // Assign goal speed first adr

		set_endless_turn[6] = velocity;		// Assign parameter velocity limit
		set_endless_turn[7] = velocity >> 8;	// Assign parameter velocity limit

		this->calculate_checksum(set_endless_turn, 8); // calculate checksum

		set_endless_turn[8] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_endless_turn, 9); // send msg and get response


	}


	void set_position_and_velocity(uint8_t ID, float position_deg,
			float velocity_deg_per_s) {

		uint16_t pos = this->calculate_pos(position_deg); // goal position calculate
		uint16_t velocity = this->calculate_vel(velocity_deg_per_s); // goal position calculate
		/*
		 * When set to 0, the velocity is the largest possible for the
		 supplied voltage, e.g. no velocity control is applied
		 */

		// create array
		uint8_t set_pos_vel_u8[11];

		// assign syncron bytes
		set_pos_vel_u8[0] = HEADER_FIRST;
		set_pos_vel_u8[1] = HEADER_SECOND;

		set_pos_vel_u8[2] = ID;   // Assign acuator id
		set_pos_vel_u8[3] = 0x07; // Assign packet length

		set_pos_vel_u8[4] = WRITE_DATA; // Assign instruction
		set_pos_vel_u8[5] = GOAL_POSITION_L_ADR; // Assign position first addr

		set_pos_vel_u8[6] = pos;	  // Assign parameter : position
		set_pos_vel_u8[7] = pos >> 8; // Assign parameter : position

		set_pos_vel_u8[8] = velocity; // Assign parameter : velocity
		set_pos_vel_u8[9] = velocity >> 8; // Assign parameter : velocity

		this->calculate_checksum(set_pos_vel_u8, 10); // calculate checksum

		set_pos_vel_u8[10] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_pos_vel_u8, 11); // send msg and get response


	}

	void set_max_torque(uint8_t ID) {

		uint16_t max_torque_u16 = 0x3FF;

		// create array
		uint8_t set_max_torque_u8[9];

		// assign syncron bytes
		set_max_torque_u8[0] = HEADER_FIRST;
		set_max_torque_u8[1] = HEADER_SECOND;

		set_max_torque_u8[2] = ID;   // Assign acuator id
		set_max_torque_u8[3] = 0x05; // Assign packet length

		set_max_torque_u8[4] = WRITE_DATA; // Assign instruction
		set_max_torque_u8[5] = MAX_TORQUE_LIMIT_L_ADR; // Assign max torque first addr

		set_max_torque_u8[6] = max_torque_u16;	  // Assign parameter : position
		set_max_torque_u8[7] = max_torque_u16 >> 8; // Assign parameter : position

		this->calculate_checksum(set_max_torque_u8, 8); // calculate checksum

		set_max_torque_u8[8] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_max_torque_u8, 9); // send msg and get response


	}

	void set_led(uint8_t ID, uint8_t led_status) {

		/* led_status = 1 -> led on
		 * led_status = 0 -> led off
		 */

		// create array
		uint8_t set_led_array[8];

		// assign syncron bytes
		set_led_array[0] = HEADER_FIRST;
		set_led_array[1] = HEADER_SECOND;

		set_led_array[2] = ID;   // Assign acuator id
		set_led_array[3] = 0x04; // Assign packet length

		set_led_array[4] = WRITE_DATA;  // Assign instruction
		set_led_array[5] = LED_ADR; 	// Assign led addr

		set_led_array[6] = led_status;	  // Assign parameter : position

		this->calculate_checksum(set_led_array, 7); // calculate checksum

		set_led_array[7] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(set_led_array, 8); // send msg and get response



	}

	void reset_actuator(uint8_t ID) {

		// create array
		uint8_t reset_packet_u8[6];

		// assign syncron bytes
		reset_packet_u8[0] = HEADER_FIRST;
		reset_packet_u8[1] = HEADER_SECOND;

		reset_packet_u8[2] = ID;   // Assign acuator id
		reset_packet_u8[3] = 0x02; // Assign packet length

		reset_packet_u8[4] = RESET; // Assign instruction

		this->calculate_checksum(reset_packet_u8, 5); // calculate checksum

		reset_packet_u8[5] = this->calculated_checksum_u8; // assign checksum

		this->polling_msg(reset_packet_u8, 6); // send msg and get response


	}



	// Send Command Actuator
	////////////////////////////////////



	////////////////////////////////////////////////////////
	// Other collaborator function

	void polling_msg(uint8_t *tx_buffer_ptr, uint16_t transmit_size_u16) {

		HAL_UART_Transmit_IT(this->uartType, tx_buffer_ptr, transmit_size_u16);

		while (this->uartType->gState != HAL_UART_STATE_READY)
			;

	}


	uint32_t value_to_baudrate(uint8_t baudrate_adr_value) {

		return ((uint32_t)(2000000 / (baudrate_adr_value + 1)));
	}

	uint8_t baudrate_to_value(uint32_t baudrate_in_BPS) {

		return ((uint8_t)((2000000 / baudrate_in_BPS) - 1));
	}

	uint16_t calculate_pos(float position_in_degree) {


		// per unit degree 0.293255
		return ((uint16_t)(position_in_degree/0.293255));

	}

	uint16_t calculate_vel(float velocity_deg_per_s) {

		float rpm = 0.16667 * velocity_deg_per_s;

		return ((uint16_t)(rpm/0.111));

	}

	uint16_t calculate_endless_turn_vel(float velocity_percent){

		if(velocity_percent > 100){

			velocity_percent = 100;

		}

		else if(velocity_percent <0){

			velocity_percent = 0;
		}

		return ((uint16_t)(velocity_percent/0.1));

	}

	void calculate_checksum(uint8_t *data_array, uint8_t size) {

		this->calculated_checksum_u8 = 0;

		for (int i = 2; i < size; i++) {

			this->calculated_checksum_u8 += data_array[i];

		}

		this->calculated_checksum_u8 = (~(this->calculated_checksum_u8)) & 0xFF;

	}

	// Other collaborator function
	////////////////////////////////////////////////////////

};

typedef dynamixel_ax_12A_parameters *dynamixel_ax_12A_parameters_ptr;

#endif
