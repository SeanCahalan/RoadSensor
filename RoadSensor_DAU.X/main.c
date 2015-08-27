/**
 * \file
 * @brief Road Sensor Interface.
 *
 * This code runs on a DAU and communicates with a Lufft IRS31 Road Sensor at
 * 19200 baud. Every 3 seconds it polls to the sensor and receives a reading in response.
 * The response packet is processed and the corresponding relay is switched on.
 * As a diagnostic tool the BI-LED flashes red upon sending a packet and green
 * once ready to process a received packet.
 */


/*********************************************************************
 *  Include Files
 *********************************************************************/
#include <xc.h>
#include <stdlib.h>
#include <pic16f874a.h>
 
/*********************************************************************
 *  PIC Configuration Fuses
 *********************************************************************/
// CONFIG
#pragma config DEBUG = OFF
#pragma config LVP = OFF
#pragma config FOSC = HS        // Oscillator Selection bits
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)

/*********************************************************************
 *      Hardware Definitions
 *********************************************************************/
//BI-LED
#define GREEN_LED PORTCbits.RC0
#define GREEN_LED_TRIS TRISCbits.TRISC0
#define RED_LED PORTCbits.RC1
#define RED_LED_TRIS TRISCbits.TRISC1

#define CLK_OUT PORTCbits.RC3
#define CLK_OUT_TRIS TRISCbits.TRISC3
#define DATA_OUT PORTCbits.RC5
#define DATA_OUT_TRIS TRISCbits.TRISC5

// RELAYS
#define NOTDRY PORTDbits.RD3          //RELAY 1
#define NOTDRY_TRIS TRISDbits.TRISD3
#define MOIST PORTBbits.RB0           //RELAY 2
#define MOIST_TRIS TRISBbits.TRISB0
#define WET PORTBbits.RB1             //RELAY 3
#define WET_TRIS TRISBbits.TRISB1
#define SALT PORTBbits.RB2            //RELAY 4
#define SALT_TRIS TRISBbits.TRISB2
#define FRWET PORTBbits.RB3           //RELAY 5
#define FRWET_TRIS TRISBbits.TRISB3
#define CRIT PORTBbits.RB4            //RELAY 6
#define CRIT_TRIS TRISBbits.TRISB4
#define DRY PORTDbits.RD2             //RELAY 7
#define DRY_TRIS TRISDbits.TRISD2
#define UNDEF PORTBbits.RB5           //RELAY 8
#define UNDEF_TRIS TRISBbits.TRISB5

//Switch for testing relays
#define SWITCH PORTEbits.RE1
#define SWITCH_TRIS TRISEbits.TRISE1
//Switch for testing watchdog
#define SWITCH2 PORTEbits.RE2
#define SWITCH2_TRIS TRISEbits.TRISE2

#define DATA_OUT PORTCbits.RC5
#define DATA_OUT_TRIS TRISCbits.TRISC5
#define CLK_1296 PORTCbits.RC3
#define CLK_1296_TRIS TRISCbits.TRISC3
#define CS_1296 PORTAbits.RA0
#define CS_1296_TRIS TRISAbits.TRISA0
#define CS_1488 PORTAbits.RA5
#define CS_1488_TRIS TRISAbits.TRISA5

// EEPROM - not used
#define EEPROM_CS PORTAbits.RA0
#define EEPROM_CS_TRIS TRISAbits.TRISA0

// Analog converter - not used
#define ADC_CS PORTAbits.RA2
#define ADC_CS_TRIS TRISAbits.TRISA2

/*********************************************************************
 *  Constants
 *********************************************************************/
#define RXBUF_SIZE  29          // Serial port buffer
#define RX_TIMEOUT 50           // msec

/*********************************************************************
 *  Local Variables
 *********************************************************************/
unsigned char RxPtr = 0;
unsigned char RxBuf[RXBUF_SIZE];
unsigned char RxTimer = 0;
unsigned char PrRxMsg = 0;
unsigned int Rx_OK = 0;
unsigned char SecondInterrupt = 0;
//TxBuf: consisten poll to the Road Sensor
unsigned char TxBuf[19] = {0x01,0x10,0x01,0x10,0x01,0xf0,0x07,0x02,0x2f,
                           0x10,0x02,0x86,0x03,0x59,0x02,0x03,0x18,0x7e,0x04};
unsigned char PollTimer = 58;
unsigned int RedLED_timer = 0;
unsigned int GreenLED_timer = 0;

/*********************************************************************
 *  Function Definitions
 *********************************************************************/
static void InitPorts(void);
static void InitTimer(void);
static void Init_UART(void);
static void UART_putc(char c);
static void Read_UART(void);
static void Pr_UART_Data(void);
static void PollSensor(void);
unsigned short generateCRC(unsigned char *buf, int len);
unsigned short calc_crc(unsigned short crc_buff, unsigned char input);
static void TurnOffRelays(void);
static void TestRelays(void);

/*********************************************************************
 *  Main Program
 *********************************************************************/
 
 void main()
 {
    //Configure watchdog
    OPTION_REGbits.PSA = 1; //Prescale assigned to watchdog
    OPTION_REGbits.PS2 = 0; //Divide by 16

    InitPorts();
    InitTimer();
    Init_UART();

    if(SWITCH == 1)         //If switch RE1 is on test relays
        TestRelays();

     // Main loop
     for (;;)
     {
         CLRWDT();                 // Reset the watchdog
         Read_UART();              // Read incoming serial port data into RxBuf
         Pr_UART_Data();           // Process UART data if available

         if (SecondInterrupt)
         {
             SecondInterrupt = 0;
             //Get a reading every 3 seconds
             if (++PollTimer > 2)
             {
                 PollTimer = 0;
                 PollSensor(); //Write output to the Road Sensor
             }
         }
     }                           // repeat forever
 }

/*******************************************************************//**
 * @brief   Calculates CRC (CRC-CCITT)
 *
 * @param   Previous CRC, input: next byte to compare to
 *
 * @return  2 Byte CRC
 */
unsigned short calc_crc(unsigned short crc_buff, unsigned char input) 
{  
    unsigned char i;  
    unsigned short x16; // we'll use this to hold the XOR mask  
    for (i=0; i<8; i++)  {   // XOR current D0 and next input bit to determine x16 value   
        if( (crc_buff & 0x0001) ^ (input & 0x01) )    
            x16 = 0x8408;   
        else    
            x16 = 0x0000;   // shift crc buffer   
        crc_buff = crc_buff >> 1;   // XOR in the x16 value   
        crc_buff ^= x16;   // shift input for next iteration   
        input = input >> 1;  
    }  
    return(crc_buff); 
}

 /*******************************************************************//**
 * @brief   Generates the checksum (CRC-CCITT)
 *
 * @param   Packet: a character buffer, length: size of the buffer
 *
 * @return  2 Byte Checksum
 */
 unsigned short generateCRC(unsigned char *buf, int len)
 {
     unsigned short crc;
     
     crc = 0xFFFF; //CRC-CCITT uses a base of 0xFFFF
     for(int i = 0; i<len-3; i++){
        crc = calc_crc(crc, buf[i]); //Go though each byte leading to the CRC
     }
     return crc;
 }
 
 /*******************************************************************//**
 * @brief   Configure UART for 19200 baud, 8 data bits, no parity
 *
 * @param 
 *
 * @return  none
 */
 static void Init_UART(void)
 {
   TXSTAbits.BRGH = 1;  // High speed
   TXSTAbits.TXEN = 1;  // Enable transmit

   RCSTAbits.SPEN = 1;  // Enable serial port
   RCSTAbits.CREN = 1;  // Enable continuous receive

   SPBRG = 47;  // 19200 baud. SPBRG+1 =  14,745,600/(16 * Baudrate)
 }

// 1 Msec interrupt
// 14.7456/4 = 3.6864 MHz
//Timer 1 reload value: 65535 - 3686 = 0xF199, 3686 ticks = 1 second
#define TMR1H_VAL 0x0F1
#define TMR1L_VAL 0x099

 /*******************************************************************//**
 * @brief   Configure Timer 1 for 1 msec periodic operation
 *
 * @param
 *
 * @return  none
 */
 static void InitTimer(void)
 {
    T1CON = 0x01;               //Configure Timer1 interrupt
    TMR1L = TMR1L_VAL;
    TMR1H = TMR1H_VAL;
 
    PIE1bits.TMR1IE = 1;    // Timer 1 interrupt enable
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;     // Global interrupt enable
    T1CONbits.TMR1ON = 1;
    T1CONbits.T1CKPS0 = 0;  // Prescale
    T1CONbits.T1CKPS1 = 0;
 }

 /*******************************************************************//**
 * @brief   Interrupt Handler
 *
 * @param
 *
 * @return  none
 */
 void interrupt isr(void)
 {
    static int Count=0;

    // 1 msec timer
    if (PIR1bits.TMR1IF)
    {
        PIR1bits.TMR1IF = 0;
        TMR1L = TMR1L_VAL;
        TMR1H = TMR1H_VAL;

        if (++Count > 999)
        {
            Count = 0;
            SecondInterrupt = 1; //1 second has passed
        }

        // Check the receive timer
        if (RxTimer)
        {
            if (!--RxTimer){
                PrRxMsg = 1;    // Process the packet now.
                Rx_OK = 1;      // Packet is valid
            }
        }

        if (RedLED_timer)
        {
            if (!--RedLED_timer){
                RED_LED = 0;
                GREEN_LED = 0;
                if(Rx_OK){      // if valid packet received
                    Rx_OK = 0;
                    GREEN_LED = 1;  //flash green for packet received
                    GreenLED_timer = 100;
                }
            }
        }

        if (GreenLED_timer)
        {
            if (!--GreenLED_timer){
                GREEN_LED = 0;  //turn off green
            }
        }
    }
 }

 /*******************************************************************//**
 * @brief   Send command to Road Sensor to get a road condition reading.
 *          
 * @param
 *
 * @return  none
 */
 static void PollSensor(void)
 {
    for(int i = 0; i<19; i++){
            UART_putc(TxBuf[i]);
    }
    RED_LED = 1;             //Turn on LED and set timer for LED flash
    RedLED_timer = 100;
 }

 /*******************************************************************//**
 * @brief   Send character to serial port
 *
 * @param   c - character to send
 *
 * @return  none
 */
 static void UART_putc(char c)
 {
     //Wait until last char is sent
     while (TXSTAbits.TRMT == 0);

     TXREG = c;
 }

 /*******************************************************************//**
 * @brief   Configure I/O ports
 *
 * @param
 *
 * @return  none
 */
 static void InitPorts(void)
 {
     GREEN_LED_TRIS = 0; // Output
     GREEN_LED = 0;

     RED_LED_TRIS = 0; // Output
     RED_LED = 0;

     //RELAYS: each corresponds to a road condition
     DRY_TRIS = 0;
     DRY = 0;
     MOIST_TRIS = 0;
     MOIST = 0;
     WET_TRIS = 0;
     WET = 0;
     SALT_TRIS = 0;
     SALT = 0;
     FRWET_TRIS = 0;
     FRWET = 0;
     CRIT_TRIS = 0;
     CRIT = 0;
     NOTDRY_TRIS = 0;
     NOTDRY = 0;
     UNDEF_TRIS = 0;
     UNDEF = 0;

     CLK_OUT = 0;
     CLK_OUT_TRIS = 0;

     DATA_OUT = 0;
     DATA_OUT_TRIS = 0;

     EEPROM_CS = 1;
     EEPROM_CS_TRIS = 0;

     ADC_CS = 1;
     ADC_CS_TRIS = 0;

     //Switch for testing relays
     SWITCH_TRIS = 0;
     SWITCH = 0;
     //Switch for testing watchdog
     SWITCH2_TRIS = 0;
     SWITCH2 = 0;

    
     DATA_OUT_TRIS = 0;
     DATA_OUT = 0;

     CLK_1296_TRIS = 0;
     CLK_1296 = 0;

     CS_1296_TRIS = 0;
     CS_1296 = 1;

     CS_1488_TRIS = 0;
     CS_1488 = 1;



     //Set all inputs to digital
     ADCON1 = 6;
 }

 /*******************************************************************//**
 * @brief   Read data from Road Sensor on serial port
 *
 *          Store char in RxBuf and load timeout timer.
 *          Packet is processed after RX_TIMEOUT (msec).
 * @param
 *
 * @return  none
 */
 static void Read_UART(void)
 {
     unsigned char c;

     if (RCSTAbits.FERR || RCSTAbits.OERR)
     {
         c = RCREG;
         RCSTAbits.CREN = 0;
         NOP();
         RCSTAbits.CREN = 1;
     }

     // Check if char available
     if (PIR1bits.RCIF)
     {
         c = RCREG;
         if (RxPtr < RXBUF_SIZE)
         {             
             RxBuf[RxPtr++] = c;
             RxTimer = RX_TIMEOUT;
         }
     }
 }

 /*******************************************************************//**
 * @brief   Turms off all relays so the next can be turned on
 *
 * @param
 *
 * @return  none
 */
 static void TurnOffRelays(void){
     NOTDRY = 0;
     MOIST = 0;
     WET = 0;
     SALT = 0;
     FRWET = 0;
     CRIT = 0;
     DRY = 0;
     UNDEF = 0;
 }

 /*******************************************************************//**
 * @brief   Process response packet from Road Sensor
 *
 *
 * @param   RxBuf where the format is shown below. The Status corresponds
 *          to the road condition and value to the according measurement.
 *
 *          Packet Format:
 *           0  1  2  3  4  5  6  7  8  9  10 11 12 13 14
 *           01 10 01 f0 01 10 11 02 2f 10 00 02 05 00 86
 *           |                    |        |
 *           |                    |        |
 *           |-SOH                |-STX    |-Payload
 *
 *           15 16 17 18 19 20 21 22 23 24 25 26 27 28
 *           03 10 00 06 00 59 02 12 00 00 03 18 0a 04
 *                 |                 |     |  |     |
 *                 |                 |     |  |     |
 *                 |-Status          |-Val |  |-CS  |-EOT
 *
 * @return  none
 */
 static void Pr_UART_Data(void)
 {
     unsigned int sensorVal;  //Data read from the sensor
     unsigned short crc;
     unsigned char status;    //Sensor Status

     if (PrRxMsg)
     {
         PrRxMsg = 0;
         RxPtr = 0;   //reset pointer to 0 to fill Rxbuf with next UART reading

         crc = generateCRC(RxBuf, RXBUF_SIZE);
         //Check that packet has valid CRC
         if ((RxBuf[26] == (crc&0xff)) && (RxBuf[27] == (crc>>8))){
             status = RxBuf[17];

             //turn off other relays
             TurnOffRelays();
             //turn on relay
             switch(status){
                 case 0x00:
                    DRY = 1;
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ;
                    break;
                 case 0x01:
                    MOIST = 1;
                    NOTDRY = 1; //Not dry corresponds to all not dry conditions
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ;
                    break;
                 case 0x02:
                    WET = 1;
                    NOTDRY = 1;
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ;
                    break;
                 case 0x05:
                    SALT = 1;
                    NOTDRY = 1;
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ; //water height
                    break;
                 case 0x06:
                    FRWET = 1;
                    NOTDRY = 1;
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ;
                    break;
                 case 0x07:
                    CRIT = 1;
                    NOTDRY = 1;
                    sensorVal = (RxBuf[24] << 8) | RxBuf[23] ;
                    break;
                 default:
                     UNDEF = 1;
                     break;
             }   
         }
     }
 }

 /*******************************************************************//**
 * @brief   If switch RE1 is on then test all relays upon device start
 *
 * @param
 *
 * @return  none
 */
 static void TestRelays(void){
     int i = 0;
     for (;;)
     {
         CLRWDT();                    // Reset the watchdog
         
         if (SecondInterrupt)         // Test a relay each second
         {
             TurnOffRelays();
             SecondInterrupt = 0;
             
             switch(i){
                 case 0:              //RELAY 1
                     NOTDRY = 1;
                     break;
                 case 1:              //RELAY 2
                     MOIST = 1;
                     break;
                 case 2:              //RELAY 3
                     WET = 1;
                     break;
                 case 3:              //RELAY 4
                     SALT = 1;
                     break;
                 case 4:              //RELAY 5
                     FRWET = 1;
                     break;
                 case 5:              //RELAY 6
                     CRIT = 1;
                     break;
                 case 6:              //RELAY 7
                     DRY = 1;
                     break;
                 case 7:              //RELAY 8
                     UNDEF = 1;
                     break;
                 }
                 if(++i>8)
                    break;
         }
     }
 }