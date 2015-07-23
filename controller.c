#include "controller.h"
#include "xparameters.h"
#include "ascii_characters.h"
#include "xio.h"
#include "stdio.h"
#include "xuartlite_l.h"
#include "xuartlite.h"
#include <unistd.h>
#include "w3_userio.h"
#include "w3_ad_controller.h"
#include "w3_clock_controller.h"
#include "w3_iic_eeprom.h"
#include "radio_controller.h"

//Abstracts away the long xparameters name
//This way, if we change the name of the core or register,
//we only have to change this line.
#define RC_BASEADDR 		XPAR_RADIO_CONTROLLER_0_BASEADDR
#define AD_BASEADDR 		XPAR_W3_AD_CONTROLLER_0_BASEADDR
#define CLK_BASEADDR 		XPAR_W3_CLOCK_CONTROLLER_0_BASEADDR
#define Alpha_memory		XPAR_FULLDPDDESIGN_PLBW_0_MEMMAP_ALPHA
#define Alpha_user 			XPAR_FULLDPDDESIGN_PLBW_0_MEMMAP_USER_ALPHA
#define Delay				XPAR_FULLDPDDESIGN_PLBW_0_MEMMAP_DELAY_FOR_PA
#define Controls			XPAR_FULLDPDDESIGN_PLBW_0_MEMMAP_CONTROLS

/* The CONTROLS are defined as follows:
 * [Enable LTE Signal, Enable DPD, Enable Alpha Counter to store training, Enable Alpha to Write to Memory, Enable Manual Alpha, Reset, 0,0,...]
 *
 * */

#define Alpha_depth			XPAR_FULLDPDDESIGN_PLBW_0_MEMMAP_ALPHA_DEPTH

//Instantiates an instance of the UART driver (controls the serial)
static XUartLite UartLite;

//This is going to store our two gain parameters
unsigned char RxCoarseGain = 2; //range is 1 to 3
unsigned int  RxFineGain = 5; // range is 0 to 31 (0 to 0x1F)
unsigned int  TxGain = 60; // range is 0 to 63  (0x3c=60)
unsigned int  wifiChannel = 6;

int main(){
w3_node_init();

// Write some code here that gives a user instructions for controlling your transceiver.
// --- Enable/disable transmitter
// --- Enable/disable receiver
// --- Enable/disable/reset CFO Correction
// --- Enable/disable/reset timing correction
// --- Select output of DAC (to view signals at various stages in your design)
// --- Modify all filter coefficients (both for CFO and timing synchronization) for tuning


//Set up the UART
XUartLite_Initialize(&UartLite, XPAR_UARTLITE_0_DEVICE_ID);

//Set up the Radio - For details about these calls, see http://warp.rice.edu/svn/WARP/PlatformSupport/CustomPeripherals/pcores/radio_controller_v3_00_b/doc/html/api/index.html

// Configure TX enable, RX enable, and Rx HP filter for software control
radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, (RC_REG0_TXEN_CTRLSRC|RC_REG0_RXEN_CTRLSRC|RC_REG0_RXHP_CTRLSRC), RC_CTRLSRC_REG);

// Configure TX  low-pass filter response to have a corner frequency of 18 MHz:
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXLPF_BW, 0x01);

//Enable software Tx Gain control:
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA,RC_PARAMID_TXGAINS_SPI_CTRL_EN, 0x01);

//Enable software Rx Gain control:
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA,RC_PARAMID_RXGAINS_SPI_CTRL_EN, 0x01);

//Initialize the baseband Tx gain to its max value (0 dB)
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA,RC_PARAMID_TXGAIN_BB, 0x00);

//Initialize the RF Tx gain
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXGAIN_RF, TxGain);

// Configure receive HPF cutoff of 30kHz (DC block)
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXHPF_HIGH_CUTOFF_EN, 0x01);

//14MHz Low Pass Filter on RX (our max freq is 10MHz + 2.25MHz)
// 0: 7.5MHz<br>1: 9.5MHz<br>2: 14MHz<br>3: 18MHz
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXLPF_BW, 0x03);

//Initialize the Rx RF Gain (1:0dB, 2:15dB , 3:30dB )
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA,RC_PARAMID_RXGAIN_RF, RxCoarseGain);

//Initialize the Rx baseband Gain
radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXGAIN_BB, RxFineGain);

// Enable the receiver by default

// Set center frequency (i.e. choose the WiFi channel)
radio_controller_setCenterFrequency(RC_BASEADDR , RC_RFA, RC_24GHZ, wifiChannel);


//DECLERATION OF VARIABLES
int x = 0;											//Variable used for reading UART
Xuint32 dataIn;										//Variable used to temporarily store data read from memory using the XIO_In32() function
Xuint32 dataOut;									//Variable used to temporarily store data to write to memory using the XIO_Out32() function

XIo_Out32(Delay,43);								//Write the desired value for the PA delay

radio_controller_RxEnable(RC_BASEADDR, RC_RFB);		//Turn on Receiver RFB
radio_controller_TxEnable(RC_BASEADDR, RC_RFA);		//Turn on Transmitter RFA


dataIn = XIo_In32(Delay);							//Read in first delay for sanity check
xil_printf("first delay: %d  \n \r", dataIn);		//Print to terminal for user to look at

XIo_Out32(Controls,0xF0000000);						//Turn On LTE, DPD, and Training

while(1){
	x = XUartLite_RecvByte(STDIN_BASEADDRESS);	//Input from UART

	//Commands to look at and change delay
	if(x=='d'){									//Read current delay
		dataIn = XIo_In32(Delay);
		xil_printf("Current Delay: %d  \n \r", dataIn);
		}
	if(x=='e'){															//Increase delay
		radio_controller_TxRxDisable(RC_BASEADDR, RC_RFA|RC_RFB);		//Turn off TX and RX RFB
		XIo_Out32(Controls,0x04000000);									//Turn Off LTE, DPD, Training, and Reset
		dataIn = XIo_In32(Delay);										//Read current delay
		dataOut = dataIn + 1;											//step by 1
		XIo_Out32(Delay,dataOut);										//Write to delay
		dataIn = XIo_In32(Delay);										//Read current delay
		xil_printf("New Delay: %d  \n \r", dataIn);
		radio_controller_RxEnable(RC_BASEADDR, RC_RFB);		//Turn on Receiver RFB
		radio_controller_TxEnable(RC_BASEADDR, RC_RFA);		//Turn on Transmitter RFA
		}
	if(x=='c'){															//Decrease delay
		radio_controller_TxRxDisable(RC_BASEADDR, RC_RFA|RC_RFB);		//Turn off TX and RX RFB
		XIo_Out32(Controls,0x04000000);									//Turn Off LTE, DPD, Training, and Reset
		dataIn = XIo_In32(Delay);										//Read current delay
		dataOut = dataIn - 1;											//step by 1
		XIo_Out32(Delay,dataOut);										//Write to delay
		dataIn = XIo_In32(Delay);										//Read current delay
		xil_printf("New Delay: %d  \n \r", dataIn);
		radio_controller_RxEnable(RC_BASEADDR, RC_RFB);		//Turn on Receiver RFB
		radio_controller_TxEnable(RC_BASEADDR, RC_RFA);		//Turn on Transmitter RFA
		}

	//Report alpha from training
	if(x=='l'){
		int i = 0;
		for(i = 0;i<Alpha_depth;i=i+4 ){
			dataIn = XIo_In32(Alpha_memory+i);				//Read alpha
			xil_printf("%08x, address: %08x  \n \r", dataIn, Alpha_memory+i);
			}
		}

	//Commands for changing alpha manually
	if(x=='a'){								//Switch to Manual Alpha
		XIo_Out32(Controls,0x04000000); 	//Turn Off Signal, DPD, learning, and reset
		XIo_Out32(Controls,0xC8000000);		//Turn On LTE, DPD, and manual alpha
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		xil_printf("Current Alpha: %08x  \n \r", dataIn);
	}
	if(x=='q'){								//Increase Manual Alpha Real
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		dataOut = dataIn + 0x01000000;		//step by 2^-6 = 0.015625
		XIo_Out32(Alpha_user,dataOut);		//Write to user defined alpha
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		xil_printf("New Alpha: %08x  \n \r", dataIn);
	}
	if(x=='z'){								//Decrease Manual Alpha Real
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		dataOut = dataIn - 0x01000000;		//step by -2^-6 = -0.015625
		XIo_Out32(Alpha_user,dataOut);		//Write to user defined alpha
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		xil_printf("New Alpha: %08x  \n \r", dataIn);
	}
	if(x=='w'){								//Increase Manual Alpha Imag
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		dataOut = dataIn + 0x00000100;		//step by 2^-6 = 0.015625
		XIo_Out32(Alpha_user,dataOut);		//Write to user defined alpha
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		xil_printf("New Alpha: %08x  \n \r", dataIn);
	}
	if(x=='x'){								//Decrease Manual Alpha Imag
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		dataOut = dataIn - 0x00000100;		//step by -2^-6 = -0.015625
		XIo_Out32(Alpha_user,dataOut);		//Write to user defined alpha
		dataIn = XIo_In32(Alpha_user);		//Read current user defined alpha
		xil_printf("New Alpha: %08x  \n \r", dataIn);
	}
}
}
// Do NOT modify the code below
int w3_node_init() {

	int status;
	int ret = XST_SUCCESS;

	microblaze_enable_exceptions();

	//Initialize the AD9512 clock buffers (RF reference and sampling clocks)
	status = clk_init(CLK_BASEADDR, 2);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in clk_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	status = clk_config_dividers (CLK_BASEADDR, 2, CLK_SAMP_OUTSEL_AD_RFA);
	if(status != XST_SUCCESS) {
		xil_printf("wclk_config_dividers (%d)\n", status);
		ret = XST_FAILURE;
	}
	//Initialize the AD9963 ADCs/DACs for on-board RF interfaces
	ad_init(AD_BASEADDR, (RFA_AD_CS | RFB_AD_CS), 2);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in ad_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the radio_controller core and MAX2829 transceivers for on-board RF interfaces
	status = radio_controller_init(RC_BASEADDR, (RC_RFA | RC_RFB), 1, 1);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in radioController_initialize (%d)\n", status);
		ret = XST_FAILURE;
	}

	return ret;
}
