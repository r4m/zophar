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
 * Configuration file of the module SensorP.nc
 *
 * @date 08/06/2011 16:11
 * @author Filippo Zanella <filippo.zanella@dei.unipd.it>
 */

#include <Timer.h>
#include <math.h>			// /usr/msp430/include
#include <stdlib.h>
#include <UserButton.h>
#include "../TmoteComm.h"

//#define DEBUG

configuration SensorC
{
}


implementation
{
	components MainC;
	components LedsC;

	components SensorP as App;

	components new TimerMilliC() as TimerStartDLSPI;
	components new TimerMilliC() as TimerCollectRSSMeasure;
	components new TimerMilliC() as TimerConsensus;
	components new TimerMilliC() as TimerWaitConfirmConsensus;
	components new TimerMilliC() as TimerUpdateMatricesArgs;

	components CC2420ControlC;
	components CC2420PacketC;

	components ActiveMessageC;
	components new AMSenderC(AM_COORDINATES_MSG) as AMSenderCoordinates;
	components new AMReceiverC(AM_COORDINATES_MSG) as AMReceiverCoordinates;
	components new AMSenderC(AM_CONSENSUS_MSG) as AMSenderConsensus;
	components new AMReceiverC(AM_CONSENSUS_MSG) as AMReceiverConsensus;

	components SerialActiveMessageC;
	components new SerialAMSenderC(AM_MATRICES_ARGS_MSG) as SAMSendMatricesArgs;

	components new QueueC(uint16_t, QUEUE_SIZE) as QueueDistance;
	components new QueueC(int8_t, QUEUE_SIZE) as QueueRss;

	components RandomC;

	App.Boot->MainC.Boot;
	App.Leds->LedsC.Leds;

	App.TimerStartDLSPI->TimerStartDLSPI.Timer;
	App.TimerCollectRSSMeasure->TimerCollectRSSMeasure.Timer;
	App.TimerConsensus->TimerConsensus.Timer;
	App.TimerWaitConfirmConsensus->TimerWaitConfirmConsensus.Timer;
	App.TimerUpdateMatricesArgs->TimerUpdateMatricesArgs.Timer;

	App.InfoRadio->CC2420PacketC;
	App.CC2420Config->CC2420ControlC.CC2420Config;

	App.AMCtrlRadio->ActiveMessageC;
	App.AMPktAck->ActiveMessageC;
	App.AMSendCoordinates->AMSenderCoordinates;
	App.ReceiveCoordinates->AMReceiverCoordinates;
	App.AMSendConsensus->AMSenderConsensus;
	App.ReceiveConsensus->AMReceiverConsensus;

	App.AMCtrlSerial->SerialActiveMessageC;
	App.SAMSendMatricesArgs->SAMSendMatricesArgs;

	App.QueueDistance->QueueDistance.Queue;
	App.QueueRss->QueueRss.Queue;

	App.Random->RandomC;
}
