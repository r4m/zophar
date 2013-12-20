/*
 * copyright (c) 2010, Department of Information Engineering, University of Padova.
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
 * Application for the least square estimation of the communication channel parameters
 * using a (symmetric gossip) consensus algorithm.
 *
 * @date 26/09/2010 16:11
 * @author Filippo Zanella <filippo.zanella@dei.unipd.it>
 */

module SensorP
{
	uses
	{
		interface Boot;
		interface Leds;

		interface Timer < TMilli > as TimerStartDLSPI;
		interface Timer < TMilli > as TimerCollectRSSMeasure;
		interface Timer < TMilli > as TimerConsensus;
		interface Timer < TMilli > as TimerWaitConfirmConsensus;
		interface Timer < TMilli > as TimerUpdateMatricesArgs;

		interface CC2420Config;
		interface CC2420Packet as InfoRadio;

		interface SplitControl as AMCtrlRadio;
		interface PacketAcknowledgements as AMPktAck;
		interface AMSend as AMSendCoordinates;
		interface Receive as ReceiveCoordinates;
		interface AMSend as AMSendConsensus;
		interface Receive as ReceiveConsensus;

		interface SplitControl as AMCtrlSerial;
		interface AMSend as SAMSendMatricesArgs;

		interface Queue < uint16_t > as QueueDistance;
		interface Queue < int8_t > as QueueRss;

		interface Random;
	}

	provides command void chooseInstantConsensusAttempt();
	provides command void updateMatrices(matrices_args_msg * yourMam);
}


implementation
{
	/* Global variables */
	message_t coordConfigPkt;
	message_t matrixArgsPkt;
	message_t csgPkt;

	matrices_args_msg mam;									// Save the arguments computed by the task computeMatricesArgs()
	matrices_args_msg yourMamCons;							// Save the matrices_args_msg of the symmetric node (tx)
	matrices_args_msg yourMamIdle;							// Save the matrices_args_msg of the symmetric node (rx)

	uint32_t C_TIME;

	bool lockedRadio;										// Flag to know when the radio transmits
	bool lockedSerial;										// Flag to know when the serial port transmits
	uint8_t curState;										// Current state of the node

	uint8_t coordConfigPktId;								// Number of coordinates_msg send in broadcast

	uint8_t size;
	
	int16_t random;

	/* Task declaration */
	task void sendCoordinate();
	task void computeMatricesArgs();
	task void sendConsensus();
	task void updateMatricesArgs();
	task void sendMatricesArgs();
	//task void sendData();

	/*************************************** INIT **************************************/

	event void Boot.booted() {
		call Leds.set(ZERO);
		lockedRadio = FALSE;
		curState = NONE;
		size = ZERO;
		C_TIME =
			((uint32_t) (C_BEACONS - 1) * (C_SLOT + GAP) * PERCENTAGE) / (C_PROBABILITY);

		call CC2420Config.setChannel(CHANNEL_RADIO);
		call CC2420Config.sync();
	}

	event void CC2420Config.syncDone(error_t error) {
		if (error == SUCCESS) {
			call AMCtrlRadio.start();
		}
		else {
			call CC2420Config.sync();
		}
	}

	event void AMCtrlRadio.startDone(error_t error) {
		if (error == SUCCESS) {
			call AMCtrlSerial.start();
		}
		else {
			call AMCtrlRadio.start();
		}
	}

	event void AMCtrlSerial.startDone(error_t error) {
		if (error == SUCCESS) {
			call Leds.led1On();
			call InfoRadio.setPower(&coordConfigPkt, POWER_RADIO);
			call InfoRadio.setPower(&matrixArgsPkt, POWER_RADIO);
			call InfoRadio.setPower(&csgPkt, POWER_RADIO);
			//call InfoRadio.setPower(&dataPkt,POWER_RADIO);

			call TimerStartDLSPI.startOneShot(TIMER_START_DLSPI);
			//call TimerStartDLSPI.startPeriodic(TIMER_START_DLSPI);
		}
		else {
			call AMCtrlSerial.start();
		}
	}

	event void AMCtrlRadio.stopDone(error_t error) {
	}

	event void AMCtrlSerial.stopDone(error_t error) {
	}

	/***********************************************************************************/
	/********************************** DLSPI ************************************/
	/***********************************************************************************/

	event void TimerStartDLSPI.fired() {
		call Leds.set(ZERO);
		while (!(call QueueDistance.empty())) {
			call QueueDistance.dequeue();
		}
		while (!(call QueueRss.empty())) {
			call QueueRss.dequeue();
		}
		coordConfigPktId = 1;
		call TimerCollectRSSMeasure.startPeriodic(TIMER_COLLECT_RSS_MEASURE);
		#ifdef DEBUG
		printf(">> Consensus Period = %ld [ms]\n", C_TIME);
		printfflush();
		#endif
	}

	/*********************************** COLLECT_RSS_MEASURES ***********************************/
	/***********************************************************************************/

	event void TimerCollectRSSMeasure.fired() {
		curState = COLLECT_RSS_MEASURES;
		if (coordConfigPktId <= MAX_COORDINATES_MSG) {
			post sendCoordinate();
		}
		else {
			curState = COMPUTE_MATRICES_ARGS;
			lockedRadio = FALSE;
			call TimerCollectRSSMeasure.stop();
			post computeMatricesArgs();
		}
	}

	/****************************** RADIO: AMSendCoordinates *********************************/

	task void sendCoordinate() {
		coordConfigPktId++;

		if (!lockedRadio) {
			coordinates_msg *rccm =
				(coordinates_msg *) (call
				AMSendCoordinates.getPayload
				(&coordConfigPkt,
				sizeof(coordinates_msg)));
			rccm->x = X_COORD_CONFIG;
			rccm->y = Y_COORD_CONFIG;
			rccm->z = Z_COORD_CONFIG;
			if (call
				AMSendCoordinates.send(AM_BROADCAST_ADDR, &coordConfigPkt,
			sizeof(coordinates_msg)) == SUCCESS) {
				lockedRadio = TRUE;
			}
		}
	}

	event void AMSendCoordinates.sendDone(message_t * msg, error_t error) {
		if (&coordConfigPkt == msg) {
			lockedRadio = FALSE;
		}

		call Leds.led2Toggle();
	}

	/****************************** RADIO: ReceiveCoordinates ********************************/

	event message_t *ReceiveCoordinates.receive(message_t * msg,
	void *payload, uint8_t len) {
		coordinates_msg coord_config;
		uint16_t distance;
		int8_t rss;
		uint8_t lqi;

		if (curState == COLLECT_RSS_MEASURES) {
			if (len == sizeof(coordinates_msg)) {
				coordinates_msg *receivedPkt = (coordinates_msg *) payload;

				atomic
				{
					rss = call InfoRadio.getRssi(msg) + RSSI_OFFSET;
					lqi = call InfoRadio.getLqi(msg);
				}

				if (rss > RSS_BOUND && lqi > LQI_BOUND) {
					atomic
					{
						coord_config.x = receivedPkt->x;
						coord_config.y = receivedPkt->y;
						coord_config.z = receivedPkt->z;
					}
					distance = (uint16_t)
						sqrtf(powf((X_COORD_CONFIG - coord_config.x), 2)
						+ powf((Y_COORD_CONFIG - coord_config.y),
						2) + powf((Z_COORD_CONFIG -
						coord_config.z), 2));

					if ((call QueueDistance.size() < QUEUE_SIZE)) {
						call QueueDistance.enqueue(distance);
						size++;
					}

					if ((call QueueRss.size() < QUEUE_SIZE)) {
						call QueueRss.enqueue(rss);
					}

					#ifdef DEBUG
						call Leds.led0Toggle();
					#endif
				}
			}
		}

		return msg;
	}

	/******************************** COMPUTE_MATRICES_ARGS *********************************/
	/**********************************************************************************/

	task void computeMatricesArgs() {
		uint16_t distance[size];
		int8_t rss[size];
		uint8_t indx;
		float tmpSum;
		uint8_t dSize = size;
		uint8_t rSize = size;

		if (curState == COMPUTE_MATRICES_ARGS) {

			for (indx = 0; indx < dSize; indx++) {
				distance[indx] = (uint16_t) (call QueueDistance.dequeue());
				#ifdef DEBUG
				//printf("distance[%u] = %d [mm]\n", indx, distance[indx]);
				//printfflush();
				#endif
			}

			for (indx = 0; indx < rSize; indx++) {
				rss[indx] = call QueueRss.dequeue();
				#ifdef DEBUG
				//printf("rss[%u] = %d [dBm]\n", indx, rss[indx]);
				//printfflush();
				#endif
			}

			mam.Xi11 = AVOID_ROUND * dSize;				// = rSize

			tmpSum = 0;
			for (indx = 0; indx < dSize; indx++) {
				tmpSum += (log10f(distance[indx]) - log10f(D0));
			}
			mam.Xi12 = (int16_t)(-10 * tmpSum);
			mam.Xi21 = mam.Xi12;

			tmpSum = 0;
			for (indx = 0; indx < dSize; indx++) {
				tmpSum += powf((log10f(distance[indx]) - log10f(D0)), 2);
			}
			mam.Xi22 = (int16_t)(100 * tmpSum);

			tmpSum = 0;
			for (indx = 0; indx < rSize; indx++) {
				tmpSum += rss[indx];
			}
			mam.Psi1 = (int16_t)tmpSum;

			tmpSum = 0;
			for (indx = 0; indx < rSize; indx++) {
				tmpSum += (float)rss[indx] * (log10f(distance[indx]) - log10f(D0));
			}
			mam.Psi2 = (int16_t)(-10 * tmpSum);

			#ifdef DEBUG
				call Leds.set(ZERO);
			printf(" xi[1][1] = %ld\n", mam.Xi11);
			printf(" xi[1][2] = %ld\n", mam.Xi12);
			printf(" xi[2][1] = %ld\n", mam.Xi21);
			printf(" xi[2][2] = %ld\n", mam.Xi22);
			printf(" psi[1] = %ld\n", mam.Psi1);
			printf(" psi[2]= %ld\n", mam.Psi2);
			printfflush();
			#endif	
			curState = IDLE;
			#ifdef DEBUG
			printf(" State: IDLE [matrixCreation()]\n");
			printfflush();
			#endif
			post sendMatricesArgs();

			call chooseInstantConsensusAttempt();
		}
	}

	/************************ COMMAND: chooseInstantConsensusAttempt **************************/

	command void chooseInstantConsensusAttempt() {
		uint32_t c_extraction;

		curState = CHOOSE_INSTANT_CONSENSUS_ATTEMPT;

		c_extraction =
			((float)(call Random.rand16()) / (float)(~(uint16_t) 1)) *
			(C_TIME - (C_SLOT + GAP));
		//c_extraction = ((float)(call Random.rand16())/(float)(~(uint16_t)1))*(C_TIME-(C_SLOT));
		// Set a single-short timer to time c_extraction
		call TimerConsensus.startOneShot(c_extraction);
		// Set a single-short timer to time c_extraction + C_SLOT
		call TimerUpdateMatricesArgs.startOneShot((c_extraction + C_SLOT));
		
		curState = IDLE;

		#ifdef DEBUG
		printf(">> Next Extraction = %ld [ms]\n", c_extraction);
		printfflush();
		#endif
	}

	/*********************************** CONSENSUS ************************************/
	/**********************************************************************************/

	event void TimerConsensus.fired() {
		if (!(curState == IDLE)) {
			call chooseInstantConsensusAttempt();
		}
		else {
			curState = SEND_CONSENSUS;
			#ifdef DEBUG
			call Leds.led0On();
			call Leds.led1Off();
			printf(" ### CALLED CONSENSUS ### [TimerConsensus.fired()]\n");
			printfflush();
			#endif
			post sendConsensus();
		}
	}

	/****************************** RADIO: AMSendConsensus *********************************/

	task void sendConsensus() {
		if (!lockedRadio) {
			matrices_args_msg *rcmam;
			consensus_msg *rcsg;

			switch (curState) {
				case SEND_CONSENSUS:
				case SEND_CONFIRM_CONSENSUS:
					rcmam =
						(matrices_args_msg *) (call
						AMSendConsensus.getPayload
						(&matrixArgsPkt,
						sizeof(matrices_args_msg)));
					atomic
					{
						rcmam->Xi11 = mam.Xi11;
						rcmam->Xi12 = mam.Xi12;
						rcmam->Xi21 = mam.Xi21;
						rcmam->Xi22 = mam.Xi22;
						rcmam->Psi1 = mam.Psi1;
						rcmam->Psi2 = mam.Psi2;
						//rcmam->timeStamp = (uint8_t)(call Random.rand16());
						rcmam->nodeID = TOS_NODE_ID;
					}
					break;
				case SEND_SYM_GOSSIP:
				case SEND_CONFIRM_SYM_GOSSIP:
					rcsg =
						(consensus_msg *) (call
						AMSendConsensus.getPayload(&csgPkt,
						sizeof(consensus_msg)));
					atomic
					{
						rcsg->nodeID = TOS_NODE_ID;
						rcsg->random = rand(); // or call Random.rand16() - 32767
					}
					
					random = rcsg->random;
					
					break;
			}

			switch (curState) {
				case SEND_CONSENSUS:
					#ifdef DEBUG
						printf(" State: SEND_CONSENSUS [sendConsensus()]\n");
					#endif
					if (call
						AMSendConsensus.send(AM_BROADCAST_ADDR, &matrixArgsPkt,
						sizeof(matrices_args_msg)) ==
					SUCCESS) {
						#ifdef DEBUG
							printf("  Send Matrix to BROADCAST\n");
						#endif
						lockedRadio = TRUE;
					}
					//else { post sendConsensus(); }
					break;
				case SEND_CONFIRM_CONSENSUS:
					#ifdef DEBUG
						printf(" State: SEND_CONFIRM_CONSENSUS [sendConsensus()]\n");
					#endif
					call AMPktAck.requestAck(&matrixArgsPkt);
					if (call
						AMSendConsensus.send(yourMamIdle.nodeID, &matrixArgsPkt,
						sizeof(matrices_args_msg)) ==
					SUCCESS) {
						#ifdef DEBUG
							printf("  Send Matrix to Node %d\n", yourMamIdle.nodeID);
						#endif
						lockedRadio = TRUE;
					}
					//else { post sendConsensus(); }
					break;
				case SEND_SYM_GOSSIP:
					#ifdef DEBUG
						printf(" State: SEND_SYM_GOSSIP [sendConsensus()]\n");
					#endif
					if (call AMSendConsensus.send(yourMamCons.nodeID, &csgPkt,
					sizeof(consensus_msg)) == SUCCESS) {
						#ifdef DEBUG
							printf("  Send CSG to Node %d\n", yourMamCons.nodeID);
						#endif
						lockedRadio = TRUE;
					}
					//else { post sendConsensus(); }
					break;
				case SEND_CONFIRM_SYM_GOSSIP:
					#ifdef DEBUG
						printf(" State: SEND_CONFIRM_SYM_GOSSIP [sendConsensus()]\n");
					#endif
					call AMPktAck.requestAck(&csgPkt);
					if (call AMSendConsensus.send(yourMamIdle.nodeID, &csgPkt,
					sizeof(consensus_msg)) == SUCCESS) {
					#ifdef DEBUG
						printf("  Send CSG to Node %d\n", yourMamIdle.nodeID);
					#endif
						lockedRadio = TRUE;
					}
					//else { post sendConsensus(); }
					break;
			}
			#ifdef DEBUG
			printfflush();
			#endif
		}
	}

	event void AMSendConsensus.sendDone(message_t * msg, error_t error) {
		if (&matrixArgsPkt == msg) {
			switch (curState) {
				case SEND_CONSENSUS:
					curState = WAIT_REPLY_CONSENSUS;
					#ifdef DEBUG
						printf
							(" State: (SEND_CONSENSUS->) WAIT_REPLY_CONSENSUS [AMSendConsensus.sendDone(message_t*, error_t)]\n");
					#endif
					break;
				case SEND_CONFIRM_CONSENSUS:
					if (call AMPktAck.wasAcked(msg)) {
						curState = WAIT_CONFIRM_CONSENSUS;
						#ifdef DEBUG
							printf
								(" State: (SEND_CONFIRM_CONSENSUS->) WAIT_CONFIRM_CONSENSUS [AMSendConsensus.sendDone(message_t*, error_t)]\n");
						#endif
						call TimerWaitConfirmConsensus.startOneShot(TIMER_WAIT_CONFIRM_CONSENSUS);
					}
					else {
						curState = IDLE;
					#ifdef DEBUG
						printf
							(" State: (SEND_CONFIRM_CONSENSUS->) IDLE [AMSendConsensus.sendDone(message_t*, error_t)]\n");
					#endif
					}
					break;
				default:
				#ifdef DEBUG
					printf
						(" State: ??? [AMSendConsensus.sendDone(message_t*, error_t)]\n");
				#endif
					break;
			}
			#ifdef DEBUG
			printfflush();
			#endif

			lockedRadio = FALSE;
		}

		if (&csgPkt == msg) {
			switch (curState) {
				case SEND_SYM_GOSSIP:
					curState = WAIT_REPLY_SYM_GOSSIP;
					#ifdef DEBUG
						printf
							(" State: (SEND_SYM_GOSSIP->) WAIT_REPLY_SYM_GOSSIP [AMSendConsensus.sendDone(message_t*, error_t)]\n");
					#endif
					break;
				case SEND_CONFIRM_SYM_GOSSIP:
					if (call AMPktAck.wasAcked(msg)) {
						curState = UPDATE_RX_MATRICES;
						#ifdef DEBUG
							printf
								(" State: (SEND_CONFIRM_SYM_GOSSIP->) UPDATE_RX_MATRICES [AMSendConsensus.sendDone(message_t*, error_t)]\n");
						#endif
						post updateMatricesArgs();
					}
					else {
						curState = IDLE;
					#ifdef DEBUG
						printf
							(" State: (SEND_CONFIRM_SYM_GOSSIP->) IDLE [AMSendConsensus.sendDone(message_t*, error_t)]\n");
					#endif
					}
					break;
				default:
				#ifdef DEBUG
					printf
						(" State: ??? [AMSendConsensus.sendDone(message_t*, error_t)]\n");
				#endif
					break;
			}
			#ifdef DEBUG
			printfflush();
			#endif

			lockedRadio = FALSE;
		}
	}

	/****************************** RADIO: ReceiveConsensus ********************************/

	event message_t *ReceiveConsensus.receive(message_t * msg, void *payload,
	uint8_t len) {
		matrices_args_msg cons_matrix_args;
		consensus_msg csg;

		if (len == sizeof(matrices_args_msg)) {
			matrices_args_msg *receivedPkt = (matrices_args_msg *) payload;

			atomic
			{
				cons_matrix_args.Xi11 = receivedPkt->Xi11;
				cons_matrix_args.Xi12 = receivedPkt->Xi12;
				cons_matrix_args.Xi21 = receivedPkt->Xi21;
				cons_matrix_args.Xi22 = receivedPkt->Xi22;
				cons_matrix_args.Psi1 = receivedPkt->Psi1;
				cons_matrix_args.Psi2 = receivedPkt->Psi2;
				//cons_matrix_args.timeStamp = receivedPkt->timeStamp;
				cons_matrix_args.nodeID = receivedPkt->nodeID;
			}

			#ifdef DEBUG
			printf("  Receive Matrix from Node %d\n",
				cons_matrix_args.nodeID);
			#endif

			switch (curState) {
				case IDLE:
					atomic
					{
						yourMamIdle = cons_matrix_args;
					}
					curState = SEND_CONFIRM_CONSENSUS;
				#ifdef DEBUG
					call Leds.led0Off();
					call Leds.led1On();
					printf("  Saved Matrix of Node %d\n", yourMamIdle.nodeID);
					printf
						(" State: (IDLE->) SEND_CONFIRM_CONSENSUS [ReceiveConsensus.receive(message_t*, void*, uint8_t)]\n");
				#endif
					post sendConsensus();
					break;
				case WAIT_REPLY_CONSENSUS:
					atomic
					{
						yourMamCons = cons_matrix_args;
					}
					curState = SEND_SYM_GOSSIP;
				#ifdef DEBUG
					printf("  Saved Matrix of Node %d\n", yourMamCons.nodeID);
					printf
						(" State: (WAIT_REPLY_CONSENSUS->) SEND_SYM_GOSSIP [ReceiveConsensus.receive(message_t*, void, uint8_t)]\n");
				#endif
					post sendConsensus();
					break;
			}

			#ifdef DEBUG
			printfflush();
			#endif
		}

		if (len == sizeof(consensus_msg)) {
			consensus_msg *receivedPkt = (consensus_msg *) payload;

			atomic
			{
				csg.nodeID = receivedPkt->nodeID;
				csg.random = receivedPkt->random;				
			}
			
			random = csg.random;

			#ifdef DEBUG
			printf("  Receive CSG from Node %d\n", csg.nodeID);
			#endif

			switch (curState) {
				case WAIT_CONFIRM_CONSENSUS:
					if (csg.nodeID == yourMamIdle.nodeID) {
						curState = SEND_CONFIRM_SYM_GOSSIP;
					#ifdef DEBUG
						printf
							(" State: (WAIT_CONFIRM_CONSENSUS->) SEND_CONFIRM_SYM_GOSSIP [ReceiveConsensus.receive(message_t*, void*, uint8_t)]\n");
					#endif
						post sendConsensus();
					}
					else {
						curState = WAIT_CONFIRM_CONSENSUS;
						#ifdef DEBUG
							printf
								(" State: (WAIT_CONFIRM_CONSENSUS->) WAIT_CONFIRM_CONSENSUS [ReceiveConsensus.receive(message_t*, void*, uint8_t)]\n");
						#endif
					}
					break;
				case WAIT_REPLY_SYM_GOSSIP:
					if (csg.nodeID == yourMamCons.nodeID) {
						curState = UPDATE_TX_MATRICES;
					#ifdef DEBUG
						printf
							(" State: (WAIT_REPLY_SYM_GOSSIP->) UPDATE_RX_MATRICES [ReceiveConsensus.receive(message_t*, void, uint8_t)]\n");
					#endif
					}
					else {
						curState = WAIT_REPLY_SYM_GOSSIP;
					#ifdef DEBUG
						printf
							(" State: (WAIT_REPLY_SYM_GOSSIP->) WAIT_REPLY_SYM_GOSSIP [ReceiveConsensus.receive(message_t*, void*, uint8_t)]\n");
					#endif
					}
					break;
			}

			#ifdef DEBUG
			printfflush();
			#endif
		}

		return msg;
	}

	event void TimerWaitConfirmConsensus.fired() {
		if (curState == WAIT_CONFIRM_CONSENSUS) {
			curState = IDLE;
			#ifdef DEBUG
			printf(" State: (WAIT_CONFIRM_CSG->) IDLE [updateMatricesArgs()]\n");
			printfflush();
			#endif
		}
	}

	event void TimerUpdateMatricesArgs.fired() {
		post updateMatricesArgs();
	}

	task void updateMatricesArgs() {
		switch (curState) {
			case UPDATE_RX_MATRICES:
			#ifdef DEBUG
				call Leds.led2On();
			#endif
				call updateMatrices(&yourMamIdle);
				curState = IDLE;
			#ifdef DEBUG
				printf(" State: (UPDATE_RX_MATRICES->) IDLE [updateMatricesArgs()]\n");
				call Leds.led2Off();
			#endif
				post sendMatricesArgs();
				break;
			case UPDATE_TX_MATRICES:
			#ifdef DEBUG
				call Leds.led2On();
			#endif
				call updateMatrices(&yourMamCons);
				//curState = IDLE;
			#ifdef DEBUG
				printf(" State: (MATRIX_UPDATE_TX->) IDLE [updateMatricesArgs()]\n");
				call Leds.led2Off();
			#endif
				call chooseInstantConsensusAttempt();
				post sendMatricesArgs();
				break;
			case IDLE:
				#ifdef DEBUG
					printf(" State: IDLE [updateMatricesArgs()]\n");
				#endif
				break;
			default:
				//curState = IDLE;
				#ifdef DEBUG
					printf(" State: (?->) IDLE [updateMatricesArgs()]\n");
				#endif
				call chooseInstantConsensusAttempt();
				break;
		}
		#ifdef DEBUG
		printfflush();
		#endif
	}

	/************************ COMMAND: updateMatrices **************************/

	command void updateMatrices(matrices_args_msg * yourMam) {
		// round case
		/*
		mam.Xi11 = (int16_t)(((float)mam.Xi11 + (float)yourMam->Xi11) / (float)2);
		mam.Xi12 = (int16_t)(((float)mam.Xi12 + (float)yourMam->Xi12) / (float)2);
		mam.Xi21 = (int16_t)(((float)mam.Xi21 + (float)yourMam->Xi21) / (float)2);
		mam.Xi22 = (int16_t)(((float)mam.Xi22 + (float)yourMam->Xi22) / (float)2);
		mam.Psi1 = (int16_t)(((float)mam.Psi1 + (float)yourMam->Psi1) / (float)2);
		mam.Psi2 = (int16_t)(((float)mam.Psi2 + (float)yourMam->Psi2) / (float)2);
		*/
		
		// uniform random choice of ceil/floor
		
		if(random > 0)
		{		
			mam.Xi11 = (int16_t) ( ceilf ( ((float)mam.Xi11 + (float)yourMam->Xi11) / (float)2 ) );
			mam.Xi12 = (int16_t) ( ceilf ( ((float)mam.Xi12 + (float)yourMam->Xi12) / (float)2 ) );
			mam.Xi21 = (int16_t) ( ceilf ( ((float)mam.Xi21 + (float)yourMam->Xi21) / (float)2 ) );
			mam.Xi22 = (int16_t) ( ceilf ( ((float)mam.Xi22 + (float)yourMam->Xi22) / (float)2 ) );
			mam.Psi1 = (int16_t) ( ceilf ( ((float)mam.Psi1 + (float)yourMam->Psi1) / (float)2 ) );
			mam.Psi2 = (int16_t) ( ceilf ( ((float)mam.Psi2 + (float)yourMam->Psi2) / (float)2 ) );
			#ifdef DEBUG
				printf("#%d r> ", TOS_NODE_ID);
			#endif
		}
		else
		{		
			mam.Xi11 = (int16_t) ( floorf ( ((float)mam.Xi11 + (float)yourMam->Xi11) / (float)2 ) );
			mam.Xi12 = (int16_t) ( floorf ( ((float)mam.Xi12 + (float)yourMam->Xi12) / (float)2 ) );
			mam.Xi21 = (int16_t) ( floorf ( ((float)mam.Xi21 + (float)yourMam->Xi21) / (float)2 ) );
			mam.Xi22 = (int16_t) ( floorf ( ((float)mam.Xi22 + (float)yourMam->Xi22) / (float)2 ) );
			mam.Psi1 = (int16_t) ( floorf ( ((float)mam.Psi1 + (float)yourMam->Psi1) / (float)2 ) );
			mam.Psi2 = (int16_t) ( floorf ( ((float)mam.Psi2 + (float)yourMam->Psi2) / (float)2 ) );
			#ifdef DEBUG
				printf("#%d r< ", TOS_NODE_ID);	
			#endif	
		}

		mam.nodeID = yourMam->nodeID;
		#ifdef DEBUG
		/*printf(" xi[1][1] = %d\n", (mam.Xi11));
		printf(" xi[1][2] = %d\n", (mam.Xi12));
		printf(" xi[2][1] = %d\n", (mam.Xi21));
		printf(" xi[2][2] = %d\n", (mam.Xi22));
		printf(" psi[1] = %d\n", mam.Psi1);
		printf(" psi[2] = %d\n", mam.Psi2);
		printfflush();*/
		#endif
	}

	/************************* SERIAL: SAMSendMatricesArgs *****************************/

	task void sendMatricesArgs() {
		if (lockedSerial) {
			return;
		}
		else {
			matrices_args_msg *scmam =
				(matrices_args_msg *) call
				SAMSendMatricesArgs.getPayload(&matrixArgsPkt,
				sizeof(matrices_args_msg));
			atomic
			{
				scmam->Xi11 = mam.Xi11;
				scmam->Xi12 = mam.Xi12;
				scmam->Xi21 = mam.Xi21;
				scmam->Xi22 = mam.Xi22;
				scmam->Psi1 = mam.Psi1;
				scmam->Psi2 = mam.Psi2;
				scmam->nodeID = mam.nodeID; // THIS IS THE ID OF THE NODE WITH WHOM WE HAVE CONCLUDED A SYMMETRIC GOSSIP
			}
			if (call SAMSendMatricesArgs.send(AM_BROADCAST_ADDR, &matrixArgsPkt,
				sizeof(matrices_args_msg)) ==
			SUCCESS) {
				lockedSerial = TRUE;
			}
		}
	}

	event void SAMSendMatricesArgs.sendDone(message_t * msg, error_t error) {
		if (&matrixArgsPkt == msg) {
			lockedSerial = FALSE;
		}
	}
}
