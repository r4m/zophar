/*
 * Copyright (c) 2010, Department of Information Engineering, University of Padova.
 * All rights reserved.
 *
 * This file is part of Zophar.
 *
 * Zophar is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * Zophar is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Zophar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ===================================================================================
 */

/**
 *
 * Header file.
 *
 * @date 26/09/2010 16:11
 * @author Filippo Zanella <filippo.zanella@dei.unipd.it>
 */

#ifndef TMOTE_COMM_H
#define TMOTE_COMM_H

typedef nx_struct coordinates_msg
{
	nx_int16_t x;											// [mm] x coordinate
	nx_int16_t y;											// [mm] y coordinate
	nx_int16_t z;											// [mm] z coordinate
} coordinates_msg;

typedef nx_struct matrices_args_msg
{
	nx_int16_t Xi11;										
	nx_int16_t Xi12;										
	nx_int16_t Xi21;										
	nx_int16_t Xi22;										
	nx_int16_t Psi1;										
	nx_int16_t Psi2;										
	//nx_uint8_t timeStamp;
	nx_uint8_t nodeID;										
} matrices_args_msg;

typedef nx_struct consensus_msg
{
	nx_uint8_t nodeID;										
	nx_int16_t random;
} consensus_msg;

typedef enum
{
	NONE = 4,
	COLLECT_RSS_MEASURES,
	COMPUTE_MATRICES_ARGS,
	CHOOSE_INSTANT_CONSENSUS_ATTEMPT,
	IDLE,

	SEND_CONSENSUS,
	WAIT_REPLY_CONSENSUS,

	SEND_CONFIRM_CONSENSUS,
	WAIT_CONFIRM_CONSENSUS,

	SEND_SYM_GOSSIP,
	WAIT_REPLY_SYM_GOSSIP,
	SEND_CONFIRM_SYM_GOSSIP,

	UPDATE_TX_MATRICES,
	UPDATE_RX_MATRICES,
	ESTIMATE_CHANNEL,
} state_a_t;

#define TIMER_START_DLSPI 0xEA60							// [ms] Period in witch the DLSPI takes place (60 sec)
#define TIMER_COLLECT_RSS_MEASURE 2500								
#define TIMER_WAIT_CONFIRM_CONSENSUS 40						

enum
{
	CHANNEL_RADIO = 11,										// Channel radio
	POWER_RADIO = 29,										// Power of the radio CC2420

	AM_COORDINATES_MSG = 83,									// ID of coordinates_msg
	AM_CONSENSUS_MSG = 84,										// ID of cons_matrices_args_msg
	AM_MATRICES_ARGS_MSG = 93,
	
	RSSI_OFFSET = -35, //-45								// [dBm] RSSI offset
	RSS_BOUND = -75,										// [dBm] RSS lower bound
	LQI_BOUND = 90,											// [dBm] LQI upper bound

	MAX_COORDINATES_MSG = 10,								// Max number of coordinates_msg to send // PAY ATTENTION TO OVERFLOWS!

	C_BEACONS = 11, //12									// Total number of nodes in the network
	C_SLOT = 200,											// [ms] Time related with the matrixUpdate
	GAP = 20, //10											// [ms] Empty range
	C_PROBABILITY = 2,										// [%] Probability that two nodes are extracted at the same time
	PERCENTAGE = 100,
	//C_TIME = ((uint32_t)(C_BEACONS-1)*(C_SLOT+GAP)*PERCENTAGE)/(C_PROBABILITY);

	QUEUE_SIZE = 12, //15

	D0 = 10,												// [dm]

	AVOID_ROUND = 10,

	ZERO = 0,
};
#endif
