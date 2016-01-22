/*******************************************************
 
Author : Mauro Laurenti
Version : 1.0
Date : 4/11/2007
 
CopyRight 2007 all rights are reserved


********************************************************
SOFTWARE LICENSE AGREEMENT
********************************************************

The usage of the supplied software imply the acceptance of the following license.

The software supplied herewith by Mauro Laurenti (the Author) 
is intended for use solely and exclusively on Microchip PIC Microcontroller (registered mark).  
The software is owned by the Author, and is protected under applicable copyright laws. 
All rights are reserved. 
Any use in violation of the foregoing restrictions may subject the 
user to criminal sanctions under applicable laws (Italian or International ones), as well as to 
civil liability for the breach of the terms and conditions of this license. 
Commercial use is forbidden without a written acknowledgement with the Author.
Personal or educational use is allowed if the application containing the following 
software doesn't aim to commercial use or monetary earning of any kind.    

THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, 
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE AUTHOR SHALL NOT, 
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************
PURPOSES
********************************************************

This library is supposed to be used for CAN bus  
application. All the basic functions are included.
Standard and Extended messages are supported
 
*******************************************************
*******************************************************/

#ifndef CAN_FLAG	//it allows to include the library in multiple points
#define CAN_FLAG


typedef unsigned char BYTE;


typedef struct 
{ 
	unsigned long identifier;
	BYTE data[8];
	BYTE type;			//1 = IDE	0=standard
	BYTE length;		// data length
	BYTE RTR;			//Remote flag. 1 it means remote frame

} CANmessage;


//************************************************************************
// constants used for CANOperationMode function 
//************************************************************************

enum CAN_OP_MODE 
{
	CAN_OP_MODE_NORMAL = 0b00000000,
	CAN_OP_MODE_SLEEP = 0b00100000,
	CAN_OP_MODE_LOOP = 0b01000000,
	CAN_OP_MODE_LISTEN = 0b01100000,
	CAN_OP_MODE_CONFIG = 0b10000000
};

//************************************************************************
// constants used for CANInitialize function 
//************************************************************************

enum CAN_CONFIG_FLAGS
{
    CAN_CONFIG_DEFAULT          = 0b11111111,   // 11111111

    CAN_CONFIG_PHSEG2_PRG_BIT   = 0b00000001,
    CAN_CONFIG_PHSEG2_PRG_ON    = 0b11111111,   // XXXXXXX1
    CAN_CONFIG_PHSEG2_PRG_OFF   = 0b11111110,   // XXXXXXX0

    CAN_CONFIG_LINE_FILTER_BIT  = 0b00000010,
    CAN_CONFIG_LINE_FILTER_ON   = 0b11111111,   // XXXXXX1X
    CAN_CONFIG_LINE_FILTER_OFF  = 0b11111101,   // XXXXXX0X

    CAN_CONFIG_SAMPLE_BIT       = 0b00000100,
    CAN_CONFIG_SAMPLE_ONCE      = 0b11111111,   // XXXXX1XX
    CAN_CONFIG_SAMPLE_THRICE    = 0b11111011,   // XXXXX0XX

    CAN_CONFIG_MSG_TYPE_BIT     = 0b00001000,
    CAN_CONFIG_STD_MSG          = 0b11111111,   // XXXX1XXX
    CAN_CONFIG_XTD_MSG          = 0b11110111,   // XXXX0XXX

    CAN_CONFIG_DBL_BUFFER_BIT   = 0b00010000,
    CAN_CONFIG_DBL_BUFFER_ON    = 0b11111111,   // XXX1XXXX
    CAN_CONFIG_DBL_BUFFER_OFF   = 0b11101111,   // XXX0XXXX

    CAN_CONFIG_MSG_BITS         = 0b01100000,
    CAN_CONFIG_ALL_MSG          = 0b11111111,   // X11XXXXX
    CAN_CONFIG_VALID_XTD_MSG    = 0b11011111,   // X10XXXXX
    CAN_CONFIG_VALID_STD_MSG    = 0b10111111,   // X01XXXXX
    CAN_CONFIG_ALL_VALID_MSG    = 0b10011111    // X00XXXXX
};


//************************************************************************
// constants used for CANsendMessage function 
//************************************************************************

 enum CAN_TX_MSG_FLAGS
 {
	CAN_TX_PRIORITY_MASK= 0b00000011,   // bit mask
    CAN_TX_PRIORITY_0   = 0b11111100,   // XXXXXX00
    CAN_TX_PRIORITY_1   = 0b11111101,   // XXXXXX01
    CAN_TX_PRIORITY_2   = 0b11111110,   // XXXXXX10
    CAN_TX_PRIORITY_3   = 0b11111111,   // XXXXXX11

    CAN_TX_FRAME_MASK   = 0b00001000,   // bit mask
    CAN_TX_STD_FRAME    = 0b11110111,   // XXXX0XXX
    CAN_TX_XTD_FRAME    = 0b11111111,   // XXXX1XXX

	CAN_TX_RTR_MASK     = 0b01000000,    // bit mask
    CAN_REMOTE_TX_FRAME = 0b11111111,	 // X1XXXXXX
    CAN_NORMAL_TX_FRAME = 0b10111111     // X0XXXXXX
    
};

//************************************************************************
// constants used for MASK function 
//************************************************************************

enum CAN_MASK
{
    CAN_MASK_B1 = 1,
    CAN_MASK_B2 = 2,
};

//************************************************************************
// constants used for Filter function 
//************************************************************************

enum CAN_FILTER
{
    CAN_FILTER_B1_F1,
    CAN_FILTER_B1_F2,
    CAN_FILTER_B2_F1,
    CAN_FILTER_B2_F2,
    CAN_FILTER_B2_F3,
    CAN_FILTER_B2_F4
};

//************************************************************************
// constants used for RX errors
//************************************************************************

enum CAN_RX_ERRORS
{
    CAN_RX_BUFFER_1_OVFL = 0b00000001,
	CAN_RX_BUFFER_2_OVFL = 0b00000010
};

//************************************************************************
// Functions Prototypes
//************************************************************************


void CANOperationMode (enum CAN_OP_MODE mode);
void CANInitialize (BYTE propSeg, BYTE phaseSeg1, BYTE phaseSeg2,BYTE SJW, BYTE BRP, enum CAN_CONFIG_FLAGS flags);

void CANsendMessage (unsigned long identifier, BYTE *data, BYTE dataLength, enum CAN_TX_MSG_FLAGS flags);
BYTE CANreceiveMessage (CANmessage *msg);

void CANSetMask(enum CAN_MASK code, unsigned long val, enum CAN_CONFIG_FLAGS type);
void CANSetFilter(enum CAN_FILTER code, unsigned long val, enum CAN_CONFIG_FLAGS type);

BYTE CANisTxReady (void);
BYTE CANisRxReady (void);

BYTE CANisTXpassive (void);
BYTE CANisRXpassive (void);
BYTE CANisTXwarningON (void);
BYTE CANisRXwarningON (void);
BYTE CANgetTXerrorCount (void);
BYTE CANgetRXerrorCount (void);

BYTE CANisBusOFF (void);
void CANAbortMessages (void);


//************************************************************************
// Functions Implementation
//************************************************************************


//*********************************************
//Set the CAN engine mode
//*********************************************


void CANOperationMode (enum CAN_OP_MODE mode)
{
	CANCON = mode;

	while((CANSTAT & 0b11100000) != mode );
}


//*********************************************
//Initialize the CAN bus
//*********************************************

void CANInitialize (BYTE propSeg, BYTE phaseSeg1, BYTE phaseSeg2,BYTE SJW, BYTE BRP, enum CAN_CONFIG_FLAGS flags)
{	
	BYTE FilterConfig1;
    BYTE FilterConfig2;

	CANOperationMode(CAN_OP_MODE_CONFIG);	//setting configuration mode

	BRGCON1 = 0x00;		//cleaning the value
	BRGCON2 = 0x00;		//cleaning the value
	BRGCON3 = 0x00;		//cleaning the value

	SJW= SJW << 6;
	BRGCON1 |= SJW;
	BRGCON1 |= BRP;

	BRGCON2 |= propSeg;
	phaseSeg1 = phaseSeg1 <<3;
	BRGCON2 |= phaseSeg1;

	if ( !(flags & CAN_CONFIG_SAMPLE_BIT) )	//SAM 
        BRGCON2bits.SAM= 1;

    if ( flags & CAN_CONFIG_PHSEG2_PRG_BIT )
        BRGCON2bits.SEG2PHTS = 1;

	BRGCON3 |= phaseSeg2;

	if ( flags & CAN_CONFIG_LINE_FILTER_BIT )
        BRGCON3bits.WAKFIL = 1;

	
	//Receiver Settings

	RXB0CON = flags & CAN_CONFIG_MSG_BITS;
    if ( (flags & CAN_CONFIG_DBL_BUFFER_BIT)== CAN_CONFIG_DBL_BUFFER_ON )
        RXB0CONbits.RXB0DBEN = 1;
	
    RXB1CON = RXB0CON;

	//Setting the mask to receive all the messages
	
	CANSetMask (CAN_MASK_B1, 0x00000000, CAN_CONFIG_XTD_MSG);
	CANSetMask (CAN_MASK_B2, 0x00000000, CAN_CONFIG_XTD_MSG);


	switch( (flags & CAN_CONFIG_MSG_BITS) | ~CAN_CONFIG_MSG_BITS )
    {
    case CAN_CONFIG_VALID_XTD_MSG:
        FilterConfig1 = CAN_CONFIG_XTD_MSG;
        FilterConfig2 = CAN_CONFIG_XTD_MSG;
        break;

    case CAN_CONFIG_VALID_STD_MSG:
        FilterConfig1 = CAN_CONFIG_STD_MSG;
        FilterConfig2 = CAN_CONFIG_STD_MSG;
        break;
    default:
        FilterConfig1 = CAN_CONFIG_STD_MSG;	//Buffer 1 will receive the Standard messages
        FilterConfig2 = CAN_CONFIG_XTD_MSG; //Buffer 2 will receive the Extended messages
        break;
    }

    CANSetFilter(CAN_FILTER_B1_F1, 0, FilterConfig1);
    CANSetFilter(CAN_FILTER_B1_F2, 0, FilterConfig1);
    CANSetFilter(CAN_FILTER_B2_F1, 0, FilterConfig2);
    CANSetFilter(CAN_FILTER_B2_F2, 0, FilterConfig2);
    CANSetFilter(CAN_FILTER_B2_F3, 0, FilterConfig2);
    CANSetFilter(CAN_FILTER_B2_F4, 0, FilterConfig2);

	CANOperationMode(CAN_OP_MODE_NORMAL);	//setting normal mode	

}

//*********************************************
// Send a message
//*********************************************
void CANsendMessage (unsigned long identifier, BYTE *data, BYTE dataLength, enum CAN_TX_MSG_FLAGS flags)
{	
	unsigned long tamp;	//used as a tamp to set the identifier

	if (TXB0CONbits.TXREQ == 0)
		{	TXB0DLC = dataLength;	// set the data length
			 if (0b01000000 & flags)	//set the RTR bit			
				TXB0DLCbits.TXRTR = 0x01;
			 else 
				TXB0DLCbits.TXRTR = 0x00;

			if (CAN_TX_FRAME_MASK & flags)	
			{
							
				tamp = identifier & 0x000000FF;		//EID0 - EID7 setting
				TXB0EIDL = (unsigned char) tamp;

				tamp = identifier & 0x0000FF00;		//EID8 - EID15 setting
				tamp = tamp >> 8;	
				TXB0EIDH = (unsigned char) tamp;
				
				TXB0SIDL = 0x00;
				TXB0SIDLbits.EXIDE = 0x01;

				if (identifier & 0x00010000)
				TXB0SIDLbits.EID16 = 0x01;
				if (identifier & 0x00020000)
				TXB0SIDLbits.EID17 = 0x01;
				if (identifier &  0x00040000)
				TXB0SIDLbits.SID0 = 0x01;
				if (identifier &  0x00080000)
				TXB0SIDLbits.SID1 = 0x01;
				if (identifier &  0x00100000)
				TXB0SIDLbits.SID2 = 0x01;

				tamp = (identifier >> 21);
				tamp = tamp & 0x000000FF;				
				TXB0SIDH = (unsigned char) tamp;				
			}

			else	

			{
				TXB0SIDLbits.EXIDE = 0x00;	//enable standard messages

				tamp = (identifier >> 3);
				tamp = tamp & 0x000000FF;				
				TXB0SIDH = (unsigned char) tamp;
				
				tamp = identifier & 0x00000007;
				tamp = tamp << 5;
				TXB0SIDL = TXB0SIDL & 0b00011111;
				TXB0SIDL = (unsigned char) tamp;
			}

			if (0b00000001 & flags)		//set transmission priority
				TXB0CONbits.TXPRI0 = 0x01;
			else	
				TXB0CONbits.TXPRI0 = 0x00;
			if (0b00000010 & flags)
				TXB0CONbits.TXPRI1 = 0x01;	
			else
				TXB0CONbits.TXPRI1 = 0x00;		

			TXB0D0 = data[0];
			TXB0D1 = data[1];
			TXB0D2 = data[2];
			TXB0D3 = data[3];
			TXB0D4 = data[4];
			TXB0D5 = data[5];
			TXB0D6 = data[6];
			TXB0D7 = data[7];

			TXB0CONbits.TXREQ = 0x01;	//enable trasmission 
			return;
		}

// Second TX Buffer setting 

	if (TXB1CONbits.TXREQ == 0)
		{	TXB1DLC = dataLength;	// set the data length

			if (0b01000000 & flags)	//set the RTR bit			
				TXB1DLCbits.TXRTR = 0x01;
			 else 
				TXB1DLCbits.TXRTR = 0x00;			
			
			if (CAN_TX_FRAME_MASK & flags)	
			{
				tamp = identifier & 0x000000FF;		//EID0 - EID7 setting
				TXB1EIDL = (unsigned char) tamp;

				tamp = identifier & 0x0000FF00;		//EID8 - EID15 setting
				tamp = tamp >> 8;	
				TXB1EIDH = (unsigned char) tamp;

				TXB1SIDL = 0x00;
				TXB1SIDLbits.EXIDE = 0x01;

				if (identifier & 0x00010000)
				TXB1SIDLbits.EID16 = 0x01;
				if (identifier & 0x00020000)
				TXB1SIDLbits.EID17 = 0x01;
				if (identifier &  0x00040000)
				TXB1SIDLbits.SID0 = 0x01;
				if (identifier &  0x00080000)
				TXB1SIDLbits.SID1 = 0x01;
				if (identifier &  0x00100000)
				TXB1SIDLbits.SID2 = 0x01;

				tamp = (identifier >> 21);
				tamp = tamp & 0x000000FF;				
				TXB1SIDH = (unsigned char) tamp;				
			}

			else	

			{
				TXB1SIDLbits.EXIDE = 0x00;	//enable standard messages

				tamp = (identifier >> 3);
				tamp = tamp & 0x000000FF;				
				TXB1SIDH = (unsigned char) tamp;
				
				tamp = identifier & 0x00000007;
				tamp = tamp << 5;
				TXB1SIDL = TXB1SIDL & 0b00011111;
				TXB1SIDL = (unsigned char) tamp;
			}

			if (0b00000001 & flags)		//set transmission priority
				TXB1CONbits.TXPRI0 = 0x01;
			else	
				TXB1CONbits.TXPRI0 = 0x00;
			if (0b00000010 & flags)
				TXB1CONbits.TXPRI1 = 0x01;	
			else
				TXB1CONbits.TXPRI1 = 0x00;				

			TXB1D0 = data[0];
			TXB1D1 = data[1];
			TXB1D2 = data[2];
			TXB1D3 = data[3];
			TXB1D4 = data[4];
			TXB1D5 = data[5];
			TXB1D6 = data[6];
			TXB1D7 = data[7];

			TXB1CONbits.TXREQ = 0x01;	//enable trasmission
			
			return;
		}

// Third TX Buffer setting 

	if (TXB2CONbits.TXREQ == 0)
		{	TXB2DLC = dataLength;	// set the data length
			
			if (0b01000000 & flags)	//set the RTR bit			
				TXB2DLCbits.TXRTR = 0x01;
			 else 
				TXB2DLCbits.TXRTR = 0x00;		
			
			if (CAN_TX_FRAME_MASK & flags)	
			{
				tamp = identifier & 0x000000FF;		//EID0 - EID7 setting
				TXB2EIDL = (unsigned char) tamp;

				tamp = identifier & 0x0000FF00;		//EID8 - EID15 setting
				tamp = tamp >> 8;	
				TXB2EIDH = (unsigned char) tamp;

				TXB2SIDL = 0x00;
				TXB2SIDLbits.EXIDE = 0x01;

				if (identifier & 0x00010000)
				TXB2SIDLbits.EID16 = 0x01;
				if (identifier & 0x00020000)
				TXB2SIDLbits.EID17 = 0x01;
				if (identifier &  0x00040000)
				TXB2SIDLbits.SID0 = 0x01;
				if (identifier &  0x00080000)
				TXB2SIDLbits.SID1 = 0x01;
				if (identifier &  0x00100000)
				TXB2SIDLbits.SID2 = 0x01;

				tamp = (identifier >> 21);
				tamp = tamp & 0x000000FF;				
				TXB2SIDH = (unsigned char) tamp;				
			}

			else	

			{
				TXB2SIDLbits.EXIDE = 0x00;	//enable standard messages

				tamp = (identifier >> 3);
				tamp = tamp & 0x000000FF;				
				TXB2SIDH = (unsigned char) tamp;
				
				tamp = identifier & 0x00000007;
				tamp = tamp << 5;
				TXB2SIDL = TXB2SIDL & 0b00011111;
				TXB2SIDL = (unsigned char) tamp;
			}

			if (0b00000001 & flags)		//set transmission priority
				TXB2CONbits.TXPRI0 = 0x01;
			else	
				TXB2CONbits.TXPRI0 = 0x00;
			if (0b00000010 & flags)
				TXB2CONbits.TXPRI1 = 0x01;	
			else
				TXB2CONbits.TXPRI1 = 0x00;				

			TXB2D0 = data[0];
			TXB2D1 = data[1];
			TXB2D2 = data[2];
			TXB2D3 = data[3];
			TXB2D4 = data[4];
			TXB2D5 = data[5];
			TXB2D6 = data[6];
			TXB2D7 = data[7];

			TXB2CONbits.TXREQ = 0x01;	//enable transmission 
			
			return;
		}
}

//*********************************************
// Read the message from the RX buffer
//*********************************************

BYTE CANreceiveMessage (CANmessage *msg)
{	
	BYTE tamp;
	BYTE error = 0;

	if (COMSTATbits.RXB0OVFL == 0x01)	//check for buffers overflows
		error |= CAN_RX_BUFFER_1_OVFL;
	
	if (COMSTATbits.RXB1OVFL == 0x01)
		error |= CAN_RX_BUFFER_2_OVFL;


	if (RXB1CONbits.RXFUL ==0x01)	//RX Buffer 1 is read
	{	
		msg->identifier = 0;
		msg->data[0] =RXB1D0;	//retrieve the data
		msg->data[1] =RXB1D1;
		msg->data[2] =RXB1D2;
		msg->data[3] =RXB1D3;
		msg->data[4] =RXB1D4;
		msg->data[5] =RXB1D5;
		msg->data[6] =RXB1D6;
		msg->data[7] =RXB1D7;

		msg->RTR = RXB1DLCbits.RXRTR;	//retrieve the RTR bit
		
		msg->length = RXB1DLC & 0x0F;	//retrieve the length

		msg->type = RXB1SIDLbits.EXID;	//retrieve the format (standard or extended)
		
		if (RXB1SIDLbits.EXID == 0)		//reading the identifier standard format
		{	
			msg->identifier = ((unsigned long)RXB1SIDH)<< 3;
			tamp = (RXB1SIDL >> 5 ) & 0x07;
			msg->identifier = msg->identifier + tamp;
		}
		else							// reading the identifier extended format
		{
			msg->identifier = (unsigned long) RXB1EIDL;			//retrieve EID0-EID7
			msg->identifier += ((unsigned long) RXB1EIDH) << 8;	//retrieve EID8-EID15
			
			if (RXB1SIDLbits.EID16) 
				msg->identifier |= 0x00010000;
			if (RXB1SIDLbits.EID17) 
				msg->identifier |= 0x00020000;
			if (RXB1SIDLbits.SID0) 
				msg->identifier |= 0x00040000;
			if (RXB1SIDLbits.SID1) 
				msg->identifier |= 0x00080000;
			if (RXB1SIDLbits.SID2) 
				msg->identifier |= 0x00100000;

			msg->identifier |= ((unsigned long) RXB1SIDH) << 21; 
		}
		
		RXB1CONbits.RXFUL = 0x00;	//release the RX buffer for new messages		
		return (error);
	}

	if (RXB0CONbits.RXFUL ==0x01)	//RX Buffer 0 is read
	{	
		msg->identifier = 0;
		msg->data[0] =RXB0D0;	//retrieve the data
		msg->data[1] =RXB0D1;
		msg->data[2] =RXB0D2;
		msg->data[3] =RXB0D3;
		msg->data[4] =RXB0D4;
		msg->data[5] =RXB0D5;
		msg->data[6] =RXB0D6;
		msg->data[7] =RXB0D7;

		msg->RTR = RXB0DLCbits.RXRTR;	//retrieve the RTR bit
		
		msg->length = RXB0DLC & 0x0F;	//retrieve the length

		msg->type = RXB0SIDLbits.EXID;	//retrieve the format (standard or extended)
		
		if (RXB0SIDLbits.EXID == 0)		//reading the identifier standard format
		{	
			msg->identifier = ((unsigned long)RXB0SIDH)<< 3;
			tamp = (RXB0SIDL >> 5 ) & 0x07;
			msg->identifier = msg->identifier + tamp;
		}
		else							// reading the identifier extended format
		{
			msg->identifier = (unsigned long) RXB0EIDL;			//retrieve EID0-EID7
			msg->identifier += ((unsigned long) RXB0EIDH) << 8;	//retrieve EID8-EID15
			
			if (RXB0SIDLbits.EID16) 
				msg->identifier |= 0x00010000;
			if (RXB0SIDLbits.EID17) 
				msg->identifier |= 0x00020000;
			if (RXB0SIDLbits.SID0) 
				msg->identifier |= 0x00040000;
			if (RXB0SIDLbits.SID1) 
				msg->identifier |= 0x00080000;
			if (RXB0SIDLbits.SID2) 
				msg->identifier |= 0x00100000;

			msg->identifier |= ((unsigned long) RXB0SIDH) << 21;
 
		}		
		RXB0CONbits.RXFUL = 0x00;	//release the RX buffer for new messages
		
		return (error);
	}
}

//*********************************************
// Set the RX MASK
//*********************************************

void CANSetMask(enum CAN_MASK numBuffer, unsigned long mask, enum CAN_CONFIG_FLAGS type)
{	unsigned long tamp;	//used as a tamp to set the identifier

	if ((numBuffer ==CAN_MASK_B1) && (type == CAN_CONFIG_STD_MSG))	//Standard MASK RX BUFFER 1
	{
		tamp = (mask >> 3);
		tamp = tamp & 0x000000FF;				
		RXM0SIDH = (unsigned char) tamp;
				
		tamp = mask & 0x00000007;
		tamp = tamp << 5;
		RXM0SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_MASK_B2) && (type == CAN_CONFIG_STD_MSG))	//Standard MASK RX BUFFER2
	{
		tamp = (mask >> 3);
		tamp = tamp & 0x000000FF;				
		RXM1SIDH = (unsigned char) tamp;
				
		tamp = mask & 0x00000007;
		tamp = tamp << 5;
		RXM1SIDL = (unsigned char) tamp;
	}	

	if ((numBuffer ==CAN_MASK_B1) && (type == CAN_CONFIG_XTD_MSG))	//Extended MASK RX BUFFER 1
	{
		tamp = mask & 0x000000FF;		//EID0 - EID7 setting
		RXM0EIDL = (unsigned char) tamp;

		tamp = mask & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXM0EIDH = (unsigned char) tamp;

		if (mask & 0x00010000) 
			RXM0SIDLbits.EID16 = 0x01; 
		else 
			RXM0SIDLbits.EID16 = 0x00; 

		if (mask & 0x00020000)
			RXM0SIDLbits.EID17 = 0x01;
		else
			RXM0SIDLbits.EID17 = 0x00;

		if (mask &  0x00040000)
			RXM0SIDLbits.SID0 = 0x01;
		else
		    RXM0SIDLbits.SID0 = 0x00;

		if (mask &  0x00080000)
			RXM0SIDLbits.SID1 = 0x01;
		else
			RXM0SIDLbits.SID1 = 0x00;
		
		if (mask &  0x00100000)
			RXM0SIDLbits.SID2 = 0x01 ;
		else
			RXM0SIDLbits.SID2 = 0x00 ;

		tamp = (mask >> 21);
		tamp = tamp & 0x000000FF;				
		RXM0SIDH = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_MASK_B2) && (type == CAN_CONFIG_XTD_MSG))	//Extended MASK BUFFER 2
	{
		tamp = mask & 0x000000FF;		//EID0 - EID7 setting
		RXM1EIDL = (unsigned char) tamp;

		tamp = mask & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXM1EIDH = (unsigned char) tamp;

		if (mask & 0x00010000) 
			RXM1SIDLbits.EID16 = 0x01; 
		else 
			RXM1SIDLbits.EID16 = 0x00; 

		if (mask & 0x00020000)
			RXM1SIDLbits.EID17 = 0x01;
		else
			RXM1SIDLbits.EID17 = 0x00;

		if (mask &  0x00040000)
			RXM1SIDLbits.SID0 = 0x01;
		else
		    RXM1SIDLbits.SID0 = 0x00;

		if (mask &  0x00080000)
			RXM1SIDLbits.SID1 = 0x01;
		else
			RXM1SIDLbits.SID1 = 0x00;
		
		if (mask &  0x00100000)
			RXM1SIDLbits.SID2 = 0x01 ;
		else
			RXM1SIDLbits.SID2 = 0x00 ;

		tamp = (mask >> 21);
		tamp = tamp & 0x000000FF;				
		RXM1SIDH = (unsigned char) tamp;
	}
}

//*********************************************
// Set the RX Filters
//*********************************************

void CANSetFilter(enum CAN_FILTER numBuffer, unsigned long filter, enum CAN_CONFIG_FLAGS type)
{
	unsigned long tamp;

	if ((numBuffer ==CAN_FILTER_B1_F1) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 1 BUFFER 1 
	{	
		RXF0SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF0SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF0SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B1_F2) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 2 BUFFER 1 
	{	
		RXF1SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF1SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF1SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F1) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 1 BUFFER 2
	{	
		RXF2SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF2SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF2SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F2) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 2 BUFFER 2
	{	
		RXF3SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF3SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF3SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F3) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 3 BUFFER 2
	{	
		RXF4SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF4SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF4SIDL = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F4) && (type == CAN_CONFIG_STD_MSG))	// STANDARD FILTER 4 BUFFER 2
	{	
		RXF5SIDLbits.EXIDEN = 0x00;	//standard filter

		tamp = (filter >> 3);
		tamp = tamp & 0x000000FF;				
		RXF5SIDH = (unsigned char) tamp;
				
		tamp = filter & 0x00000007;
		tamp = tamp << 5;
		RXF5SIDL = (unsigned char) tamp;
	}


	if ((numBuffer ==CAN_FILTER_B1_F1) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 1 BUFFER 1
	{	
		RXF0SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF0EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF0EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF0SIDLbits.EID16 = 0x01; 
		else 
			RXF0SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF0SIDLbits.EID17 = 0x01;
		else
			RXF0SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF0SIDLbits.SID0 = 0x01;
		else
		    RXF0SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF0SIDLbits.SID1 = 0x01;
		else
			RXF0SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF0SIDLbits.SID2 = 0x01 ;
		else
			RXF0SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF0SIDH = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B1_F2) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 2 BUFFER 1
	{	
		RXF1SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF1EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF1EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF1SIDLbits.EID16 = 0x01; 
		else 
			RXF1SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF1SIDLbits.EID17 = 0x01;
		else
			RXF1SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF1SIDLbits.SID0 = 0x01;
		else
		    RXF1SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF1SIDLbits.SID1 = 0x01;
		else
			RXF1SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF1SIDLbits.SID2 = 0x01 ;
		else
			RXF1SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF1SIDH = (unsigned char) tamp;
	}


	if ((numBuffer ==CAN_FILTER_B2_F1) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 1 BUFFER 2
	{	
		RXF2SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF2EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF2EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF2SIDLbits.EID16 = 0x01; 
		else 
			RXF2SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF2SIDLbits.EID17 = 0x01;
		else
			RXF2SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF2SIDLbits.SID0 = 0x01;
		else
		    RXF2SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF2SIDLbits.SID1 = 0x01;
		else
			RXF2SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF2SIDLbits.SID2 = 0x01 ;
		else
			RXF2SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF2SIDH = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F2) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 2 BUFFER 2
	{	
		RXF3SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF3EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF3EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF3SIDLbits.EID16 = 0x01; 
		else 
			RXF3SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF3SIDLbits.EID17 = 0x01;
		else
			RXF3SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF3SIDLbits.SID0 = 0x01;
		else
		    RXF3SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF3SIDLbits.SID1 = 0x01;
		else
			RXF3SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF3SIDLbits.SID2 = 0x01 ;
		else
			RXF3SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF3SIDH = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F3) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 3 BUFFER 2
	{	
		RXF4SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF4EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF4EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF4SIDLbits.EID16 = 0x01; 
		else 
			RXF4SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF4SIDLbits.EID17 = 0x01;
		else
			RXF4SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF4SIDLbits.SID0 = 0x01;
		else
		    RXF4SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF4SIDLbits.SID1 = 0x01;
		else
			RXF4SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF4SIDLbits.SID2 = 0x01 ;
		else
			RXF4SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF4SIDH = (unsigned char) tamp;
	}

	if ((numBuffer ==CAN_FILTER_B2_F4) && (type == CAN_CONFIG_XTD_MSG))// EXTENDED FILTER 4 BUFFER 2
	{	
		RXF5SIDLbits.EXIDEN = 0x01;	//standard filter

		tamp = filter & 0x000000FF;		//EID0 - EID7 setting
		RXF5EIDL = (unsigned char) tamp;

		tamp = filter & 0x0000FF00;		//EID8 - EID15 setting
		tamp = tamp >> 8;	
		RXF5EIDH = (unsigned char) tamp;

		if (filter & 0x00010000) 
			RXF5SIDLbits.EID16 = 0x01; 
		else 
			RXF5SIDLbits.EID16 = 0x00; 

		if (filter & 0x00020000)
			RXF5SIDLbits.EID17 = 0x01;
		else
			RXF5SIDLbits.EID17 = 0x00;

		if (filter &  0x00040000)
			RXF5SIDLbits.SID0 = 0x01;
		else
		    RXF5SIDLbits.SID0 = 0x00;

		if (filter &  0x00080000)
			RXF5SIDLbits.SID1 = 0x01;
		else
			RXF5SIDLbits.SID1 = 0x00;
		
		if (filter &  0x00100000)
			RXF5SIDLbits.SID2 = 0x01 ;
		else
			RXF5SIDLbits.SID2 = 0x00 ;

		tamp = (filter >> 21);
		tamp = tamp & 0x000000FF;				
		RXF5SIDH = (unsigned char) tamp;
	}
	
}


//*********************************************
// Abort all the pending messages
//*********************************************

void CANAbortMessages (void)
{
	CANCONbits.ABAT = 1;
}


//*********************************************
// Return 1 if the Bus is OFF
//*********************************************
BYTE CANisBusOFF (void)
{
	return (COMSTATbits.TXBO);		
}

//*********************************************
// Return 1 if there is a TX passive status
//*********************************************
BYTE CANisTXpassive (void)
{
	return (COMSTATbits.TXBP);
}


//*********************************************
// Return 1 if there is a TX passive status
//*********************************************
BYTE CANisRXpassive (void)
{
	return (COMSTATbits.RXBP);
}

//*********************************************
// Return 1 if TX warning is ON
//*********************************************
BYTE CANisTXwarningON (void)
{
	return (COMSTATbits.TXWARN);
}

//*********************************************
// Return 1 if RX warning is ON
//*********************************************
BYTE CANisRXwarningON (void)
{
	return (COMSTATbits.RXWARN);
}

//*********************************************
// Return TX error Count
//*********************************************

BYTE CANgetTXerrorCount (void)
{
	return (TXERRCNT);
}

//*********************************************
// Return RX error Count
//*********************************************

BYTE CANgetRXerrorCount (void)
{
	return (RXERRCNT);
}

//*********************************************
// Return 1 if at least 1 TX buffer is empty
//*********************************************

BYTE CANisTxReady (void)
{
	return (!TXB0CONbits.TXREQ || !TXB1CONbits.TXREQ || !TXB2CONbits.TXREQ); //if at least one flag is 0, it returns return 1
}

//*********************************************
// Return 1 if one or more RX buffer are full
//*********************************************

BYTE CANisRxReady (void)
{
	return (RXB0CONbits.RXFUL || RXB1CONbits.RXFUL);	
}

#endif