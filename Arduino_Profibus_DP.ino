#define BAUD 45450     //  DELAY_TBIT = 44 // Working on Arduino @16MHz

#define VCC_PIN     8
#define GND_PIN     9
#define LED_PIN     10
#define TOUCH_PIN   11

unsigned long samplingtime = 0;

#define DELAY_TBIT      44  // This is timer delay for 1 TBIT
#define TX_ENABLE_PIN   2   // This pin toggles the MAX485 IC from Transmit to Recieve
#define SLAVE_ADDRESS   6   // This is the address of this slave device
#define LED_ERROR_PIN   13  // This is the built in led on pin 13 Arduino Uno - PB5

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
#define LED_ERROR_ON    PORTB |= (1<<5);
#define LED_ERROR_OFF   PORTB &= ~(1<<5);

#define TX_ENABLE_ON    PORTD |= (1<<TX_ENABLE_PIN);
#define TX_ENABLE_OFF   PORTD &= ~(1<<TX_ENABLE_PIN); 
///////////////////////////////////////////////////////////////////////////////////////////////////
#define TIMER1_RUN TCCR1B |= (1 << CS11);
#define TIMER1_STOP TCCR1B &= ~(1 << CS11);
//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Ident Nummer DP Slave. Arbitrarily chosen. This ID not found in my .GSD library.
///////////////////////////////////////////////////////////////////////////////////////////////////
#define IDENT_HIGH_BYTE       0xC0
#define IDENT_LOW_BYTE        0xDE
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Addresses
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SAP_OFFSET            128   // Service Access Point Adress Offset
#define BROADCAST_ADD         127
#define DEFAULT_ADD           126   // Delivery address
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Command types
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SD1                   0x10  // Telegram without data field
#define SD2                   0x68  // Data telegram variable
#define SD3                   0xA2  // Data telegram fixed
#define SD4                   0xDC  // Token
#define SC                    0xE5  // Short acknowledgment
#define ED                    0x16  // End
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Function Codes
///////////////////////////////////////////////////////////////////////////////////////////////////
/* FC Request */
#define FDL_STATUS            0x09  // SPS: Status query
#define SRD_HIGH              0x0D  // SPS: Set outputs, read inputs
#define FCV_                  0x10
#define FCB_                  0x20
#define REQUEST_              0x40

/* FC Response */
#define FDL_STATUS_OK         0x00  // SLA: OK
#define DATA_LOW              0x08  // SLA: (Data low) Send data inputs
#define DIAGNOSE              0x0A  // SLA: (Data high) Diagnosis pending
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Service Access Points (DP Slave) MS0
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SAP_SET_SLAVE_ADR     55  // Master sets slave address, slave responds with SC
#define SAP_RD_INP            56  // Master requests input data, slave sends input data
#define SAP_RD_OUTP           57  // Master requests output data, slave sends output data
#define SAP_GLOBAL_CONTROL    58  // Master Control, Slave Does not answer
#define SAP_GET_CFG           59  // Master requests config., Slave sends configuration
#define SAP_SLAVE_DIAGNOSIS   60  // Master requests diagnosis, slave sends diagnosis Daten
#define SAP_SET_PRM           61  // Master sends parameters, slave sends SC
#define SAP_CHK_CFG           62  // Master sends configuration, Slave sends SC
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Global Control (Data Master)
///////////////////////////////////////////////////////////////////////////////////////////////////
#define CLEAR_DATA_           0x02
#define UNFREEZE_             0x04
#define FREEZE_               0x08
#define UNSYNC_               0x10
#define SYNC_                 0x20
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Diagnosis (Answer slave)
///////////////////////////////////////////////////////////////////////////////////////////////////
/* Status Byte 1 */
#define STATION_NOT_EXISTENT_ 0x01
#define STATION_NOT_READY_    0x02
#define CFG_FAULT_            0x04
#define EXT_DIAG_             0x08  // Extended diagnosis available
#define NOT_SUPPORTED_        0x10
#define INV_SLAVE_RESPONSE_   0x20
#define PRM_FAULT_            0x40
#define MASTER_LOCK           0x80

/* Status Byte 2 */
#define STATUS_2_DEFAULT      0x04
#define PRM_REQ_              0x01
#define STAT_DIAG_            0x02
#define WD_ON_                0x08
#define FREEZE_MODE_          0x10
#define SYNC_MODE_            0x20
#define DEACTIVATED_          0x80

/* Status Byte 3 */
#define DIAG_SIZE_OK          0x00
#define DIAG_SIZE_ERROR       0x80

/* Address */
#define MASTER_ADD_DEFAULT    0xFF

/* Extended diagnosis (EXT_DIAG_ = 1) */
#define EXT_DIAG_TYPE_        0xC0  // Bit 6-7 ist Diagnose Typ
#define EXT_DIAG_GERAET       0x00  // Wenn Bit 7 und 6 = 00, dann Geraetebezogen
#define EXT_DIAG_KENNUNG      0x40  // Wenn Bit 7 und 6 = 01, dann Kennungsbezogen
#define EXT_DIAG_KANAL        0x80  // Wenn Bit 7 und 6 = 10, dann Kanalbezogen

#define EXT_DIAG_BYTE_CNT_    0x3F  // Bit 0-5 sind Anzahl der Diagnose Bytes
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Set Parameters Request (Data master)
///////////////////////////////////////////////////////////////////////////////////////////////////
/* Command */
#define LOCK_SLAVE_           0x80  // Slave fuer andere Master gesperrt
#define UNLOCK_SLAVE_         0x40  // Slave fuer andere Master freigegeben
#define ACTIVATE_SYNC_        0x20
#define ACTIVATE_FREEZE_      0x10
#define ACTIVATE_WATCHDOG_    0x08

/* DPV1 Status Byte 1 */
#define DPV1_MODE_            0x80
#define FAIL_SAVE_MODE_       0x40
#define PUBLISHER_MODE_       0x20
#define WATCHDOG_TB_1MS       0x04

/* DPV1 Status Byte 2 */
#define PULL_PLUG_ALARM_      0x80
#define PROZESS_ALARM_        0x40
#define DIAGNOSE_ALARM_       0x20
#define VENDOR_ALARM_         0x10
#define STATUS_ALARM_         0x08
#define UPDATE_ALARM_         0x04
#define CHECK_CONFIG_MODE_    0x01

/* DPV1 Status Byte 3 */
#define PARAMETER_CMD_ON_     0x80
#define ISOCHRON_MODE_ON_     0x10
#define PARAMETER_BLOCK_      0x08
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Check Config Request (Data Master)
///////////////////////////////////////////////////////////////////////////////////////////////////
#define CFG_DIRECTION_        0x30  // Bit 4-5 is direction. 01 = input, 10 = output, 11 = input / output
#define CFG_INPUT             0x10  // Input
#define CFG_OUTPUT            0x20  // Output
#define CFG_INPUT_OUTPUT      0x30  // Input/Output
#define CFG_SPECIAL           0x00  // Special format if more than 16/32 bytes are to be transferred

#define CFG_KONSISTENZ_       0x80  // Bit 7 is consistency. 0 = byte or word, 1 = over entire module
#define CFG_KONS_BYTE_WORT    0x00  // Byte or word
#define CFG_KONS_MODUL        0x80  // Modul

#define CFG_WIDTH_            0x40  // Bit 6 is IO width. 0 = byte (8bit), 1 = word (16bit)
#define CFG_BYTE              0x00  // Byte
#define CFG_WORD              0x40  // Word

/* Compact format */
#define CFG_BYTE_CNT_         0x0F  // Bit 0-3 are number of bytes or words. 0 = 1 byte, 1 = 2 bytes etc.

/* Special format */
#define CFG_SP_DIRECTION_     0xC0  // Bit 6-7 is direction. 01 = input, 10 = output, 11 = input / output
#define CFG_SP_VOID           0x00  // Empty space
#define CFG_SP_INPUT          0x40  // Input
#define CFG_SP_OUTPUT         0x80  // Output
#define CFG_SP_INPUT_OPTPUT   0xC0  // Input/Output

#define CFG_SP_VENDOR_CNT_    0x0F  // Bits 0-3 are the number of manufacturer-specific bytes. 0 = none

/* Special format / length byte */
#define CFG_SP_BYTE_CNT_      0x3F  // Bit 0-5 are number of bytes or words. 0 = 1 byte, 1 = 2 bytes etc.
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
#define TIMEOUT_MAX_SYN_TIME  33 * DELAY_TBIT // 33 TBit = TSYN
#define TIMEOUT_MAX_RX_TIME   15 * DELAY_TBIT
#define TIMEOUT_MAX_TX_TIME   15 * DELAY_TBIT
#define TIMEOUT_MAX_SDR_TIME  15 * DELAY_TBIT // 15 Tbit = TSDR
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
#define MAX_BUFFER_SIZE       32

#define INPUT_DATA_SIZE       16    // Number of bytes coming from the master
#define OUTPUT_DATA_SIZE      16    // Number of bytes going to master
#define VENDOR_DATA_SIZE      0     // Number of bytes for manufacturer-specific data
#define EXT_DIAG_DATA_SIZE    0     // Number of bytes for extended diagnostics

#define OUTPUT_MODULE_CNT     1     // Number of output modules
#define INPUT_MODULE_CNT      1     // Number of input modules
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Profibus process control
///////////////////////////////////////////////////////////////////////////////////////////////////
#define PROFIBUS_WAIT_SYN     1
#define PROFIBUS_WAIT_DATA    2
#define PROFIBUS_GET_DATA     3
#define PROFIBUS_SEND_DATA    4
///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char uart_buffer[MAX_BUFFER_SIZE];
unsigned int  uart_byte_cnt = 0;
unsigned int  uart_byte_cnt0 = 0;
unsigned int  uart_transmit_cnt = 0;
unsigned int  uart_transmit_cnt0 = 0;
unsigned long lastmillis;
// Profibus Flags and Variables
unsigned char profibus_status;
unsigned char diagnose_status;
unsigned char slave_addr;
unsigned char master_addr;
unsigned char group;
volatile unsigned char new_data;

#if (OUTPUT_DATA_SIZE > 0)
volatile unsigned char Profibus_out_register[OUTPUT_DATA_SIZE];
#endif
#if (INPUT_DATA_SIZE > 0)
unsigned char Profibus_in_register [INPUT_DATA_SIZE];
#endif
#if (VENDOR_DATA_SIZE > 0)
unsigned char Vendor_Data[VENDOR_DATA_SIZE];
#endif
#if (EXT_DIAG_DATA_SIZE > 0)
unsigned char Diag_Data[EXT_DIAG_DATA_SIZE];
#endif
unsigned char Input_Data_size;
unsigned char Output_Data_size;
unsigned char Vendor_Data_size;   // Number of read-in manufacturer-specific bytes

void setup() {

  pinMode(LED_ERROR_PIN,OUTPUT); // CPU RUN/STOP LED
  pinMode(TX_ENABLE_PIN,OUTPUT); // TX enable output
  pinMode(VCC_PIN, OUTPUT);
  pinMode(GND_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  digitalWrite(VCC_PIN,HIGH); // Set HIGH at VCC pin Touch Button
  digitalWrite(GND_PIN,LOW);  // Set LOW at GND pin Touch Button
  init_Profibus();
  TX_ENABLE_OFF; // Disable Transmit (Switch to Recieve)
}

void loop() {
  if ( (unsigned long) (micros() - samplingtime) > 10  )
  {
    if (digitalRead(TOUCH_PIN) == LOW)
    {
    Profibus_out_register[0]=0X01;       
    }
    else
    {   
    Profibus_out_register[0]=0X00;
    
    }   
    samplingtime = micros();
  }       

  if(new_data==1)
  {
      Profibus_out_register[0] +=new_data;
      new_data=0;
      Profibus_out_register[0] = Profibus_in_register[0];     
    }
  digitalWrite(LED_PIN, bitRead(Profibus_in_register[0],0)==0?1:0); // Correct the led state on Touch button
}

void init_Profibus (void)
{
  unsigned char cnt;
  // Variable init
  profibus_status = PROFIBUS_WAIT_SYN;  // Wait at least Tsyn until allowing RXdata
  diagnose_status = false;
  Input_Data_size = 0;
  Output_Data_size = 0;
  Vendor_Data_size = 0;
  group = 0;

  slave_addr = SLAVE_ADDRESS;  // <<< Temporary address assignment.
  // TODO: Read address from EEPROM or switches.
  // Illegal addresses are forced to DEFAULT (126). Set Address can be used to change it.
  if((slave_addr == 0) || (slave_addr > 126))
    slave_addr = DEFAULT_ADD;

  // Clear data
  #if (OUTPUT_DATA_SIZE > 0)
  for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
  {
    Profibus_out_register[cnt] = 0xFF;
  }
  #endif
  #if (INPUT_DATA_SIZE > 0)
  for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
  {
    Profibus_in_register[cnt] = 0x00;
  }
  #endif
  #if (VENDOR_DATA_SIZE > 0)
  for (cnt = 0; cnt < VENDOR_DATA_SIZE; cnt++)
  {
    Vendor_Data[cnt] = 0x00;
  }
  #endif
  #if (DIAG_DATA_SIZE > 0)
  for (cnt = 0; cnt < DIAG_DATA_SIZE; cnt++)
  {
    Diag_Data[cnt] = 0x00;
  }
  #endif
  new_data=0;
  noInterrupts();           // Disable all interrupts
  init_UART0(BAUD);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = TIMEOUT_MAX_SYN_TIME;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // Prescaler    
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  interrupts();             // Enable all interrupts
}

void profibus_RX (void)
{
  unsigned char cnt;
  unsigned char telegramm_type;
  unsigned char process_data;

  // Profibus data types
  unsigned char destination_add;
  unsigned char source_add;
  unsigned char function_code;
  unsigned char FCS_data;   // Frame Check Sequence
  unsigned char PDU_size;   // PDU Size
  unsigned char DSAP_data;  // SAP Destination
  unsigned char SSAP_data;  // SAP Source

  process_data = false;
  telegramm_type = uart_buffer[0];
  
  switch (telegramm_type)
  {
    case SD1: // Telegram without data, max. 6 bytes

        if (uart_byte_cnt != 6) break;

        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        function_code   = uart_buffer[3];
        FCS_data        = uart_buffer[4];

        if (addmatch(destination_add)       == false) break;
        if (checksum(&uart_buffer[1], 3) != FCS_data) break;

        process_data = true;

        break;

    case SD2: // Telegram with variable data length

        if (uart_byte_cnt != uart_buffer[1] + 6) break;

        PDU_size        = uart_buffer[1];
        destination_add = uart_buffer[4];
        source_add      = uart_buffer[5];
        function_code   = uart_buffer[6];
        FCS_data        = uart_buffer[PDU_size + 4];

        if (addmatch(destination_add)              == false) break;
        if (checksum(&uart_buffer[4], PDU_size) != FCS_data) break;

        process_data = true;

        break;

    case SD3: // Telegram with 5 bytes data, max. 11 bytes
        if (uart_byte_cnt != 11) break;
        PDU_size        = 8;              // DA + SA + FC + PDU
        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        function_code   = uart_buffer[3];
        FCS_data        = uart_buffer[9];
        if (addmatch(destination_add)       == false) break;
        if (checksum(&uart_buffer[1], 8) != FCS_data) break;
        process_data = true;
        break;
    case SD4: // Token with 3 Byte Data
        if (uart_byte_cnt != 3) break;
        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        if (addmatch(destination_add)       == false) break;
        break;
    default:
        break;
  }

  // Only evaluate if data is OK
  if (process_data == true)
  {
    master_addr = source_add; // Master address is Source address
    //Service Access Point detected?
    if ((destination_add & 0x80) && (source_add & 0x80))
    {
      DSAP_data = uart_buffer[7];
      SSAP_data = uart_buffer[8];
      // 1) SSAP 62 -> DSAP 60 (Get Diagnostics Request)
      // 2) SSAP 62 -> DSAP 61 (Set Parameters Request)
      // 3) SSAP 62 -> DSAP 62 (Check Config Request)
      // 4) SSAP 62 -> DSAP 60 (Get Diagnostics Request)
      // 5) Data Exchange Request (normal cycle)      
      switch (DSAP_data)
      {
        case SAP_SET_SLAVE_ADR: // Set Slave Address (SSAP 62 -> DSAP 55)
            // Only possible in the "Wait Parameter" (WPRM) state
            slave_addr = uart_buffer[9];
            profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            break;
        case SAP_GLOBAL_CONTROL: // Global Control Request (SSAP 62 -> DSAP 58)
            // If "Clear Data" high, then PLC CPU on "Stop"
            if (uart_buffer[9] & CLEAR_DATA_)
            {
              LED_ERROR_ON;  // Status "PLC not ready"
            }
            else
            {
              LED_ERROR_OFF; // Status "PLC OK"
            }

            // Calculate group
            for (cnt = 0;  uart_buffer[10] != 0; cnt++) uart_buffer[10]>>=1;

            // If command is for us
            if (cnt == group)
            {
              if (uart_buffer[9] & UNFREEZE_)
              {
                // Delete FREEZE state
              }
              else if (uart_buffer[9] & UNSYNC_)
              {
                // Delete SYNC state
              }
              else if (uart_buffer[9] & FREEZE_)
              {
                // Do not read inputs again
              }
              else if (uart_buffer[9] & SYNC_)
              {
                // Set outputs only with SYNC command
              }
            }

            break;

        case SAP_SLAVE_DIAGNOSIS: // Get Diagnostics Request (SSAP 62 -> DSAP 60)
           
              // After receiving the diagnosis, the DP slave changes state
              // "Power on Reset" (POR) in the state "Wait Parameter" (WPRM)
              // At the end of initialization ("Data Exchange" state (DXCHG))
              // the master sends a Diagnostics Request a second time to check correct configuration
            if (function_code == (REQUEST_ + FCB_ + SRD_HIGH))
            {
              uart_buffer[7]  = SSAP_data;                    // Target SAP Master
              uart_buffer[8]  = DSAP_data;                    // Source SAP Slave
              uart_buffer[9]  = STATION_NOT_READY_;           // Status 1
              uart_buffer[10] = STATUS_2_DEFAULT + PRM_REQ_;  // Status 2
              uart_buffer[11] = DIAG_SIZE_OK;                 // Status 3
              uart_buffer[12] = MASTER_ADD_DEFAULT;           // Address Master
              uart_buffer[13] = IDENT_HIGH_BYTE;              // Ident high
              uart_buffer[14] = IDENT_LOW_BYTE;               // Ident low
              #if (EXT_DIAG_DATA_SIZE > 0)
              uart_buffer[15] = EXT_DIAG_DATA_SIZE;           // Device-related diagnosis (number of bytes)
              for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
              {
                uart_buffer[16+cnt] = Diag_Data[cnt];
              }
              #endif
              profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET, &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
            }
            else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
                     function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
            {
              // Diagnostic request to check data from Check Config Request
              uart_buffer[7]  = SSAP_data;                    // Target SAP Master
              uart_buffer[8]  = DSAP_data;                    // Source SAP slave
              if (diagnose_status == true)
                uart_buffer[9]  = EXT_DIAG_;                  // Status 1
              else
                uart_buffer[9]  = 0x00;
              uart_buffer[10] = STATUS_2_DEFAULT;             // Status 2
              uart_buffer[11] = DIAG_SIZE_OK;                 // Status 3
              uart_buffer[12] = master_addr - SAP_OFFSET;     // Address Master
              uart_buffer[13] = IDENT_HIGH_BYTE;              // Ident high
              uart_buffer[14] = IDENT_LOW_BYTE;               // Ident low
              #if (EXT_DIAG_DATA_SIZE > 0)
              uart_buffer[15] = EXT_DIAG_DATA_SIZE;           // Device-related diagnosis (number of bytes)
              for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
              {
                uart_buffer[16+cnt] = Diag_Data[cnt];
              }
              #endif

              profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET, &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
            }
            
            break;

        case SAP_SET_PRM: // Set Parameters Request (SSAP 62 -> DSAP 61)

            // After receiving the parameters, the DP slave changes state
            // "Wait Parameter" (WPRM) in the state "Wait Configuration" (WCFG)

            // Only accept data for our device
            if ((uart_buffer[13] == IDENT_HIGH_BYTE) && (uart_buffer[14] == IDENT_LOW_BYTE))
            {
              // User Parameter Size = Length - DA, SA, FC, DSAP, SSAP, 7 Parameter Bytes
              Vendor_Data_size = PDU_size - 12;
              // Read in user parameters
              #if (VENDOR_DATA_SIZE > 0)
              for (cnt = 0; cnt < Vendor_Data_size; cnt++) Vendor_Data[cnt] = uart_buffer[16+cnt];
              #endif
              // Read group
              for (group = 0; uart_buffer[15] != 0; group++) uart_buffer[15]>>=1;
              profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            }
            break;

        case SAP_CHK_CFG: // Check Config Request (SSAP 62 -> DSAP 62)
           
        // After receiving the configuration, the DP slave changes state
        // "Wait Configuration" (WCFG) in the "Data Exchange" state (DXCHG)
        // IO configuration:
        // Compact format for max. 16/32 bytes IO
        // special format for max. 64/132 bytes IO
        // evaluate several bytes depending on the PDU data size
        // LE / LEr - (DA + SA + FC + DSAP + SSAP) = number of config bytes
      Output_Data_size=0;
      Input_Data_size=0;
            for (cnt = 0; cnt < uart_buffer[1] - 5; cnt++)
            {
              switch (uart_buffer[9+cnt] & CFG_DIRECTION_)
              {
                case CFG_INPUT:
                    Input_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                      Input_Data_size += Input_Data_size*2;
                    break;

                case CFG_OUTPUT:
                    Output_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                      Output_Data_size += Output_Data_size*2;
                    break;

                case CFG_INPUT_OUTPUT:
                    Input_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    Output_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                    {
                      Input_Data_size += Input_Data_size*2;
                      Output_Data_size += Output_Data_size*2;
                    }
                    break;

                case CFG_SPECIAL:
                    // Special format
                    // Manufacturer-specific bytes available?
                    if (uart_buffer[9+cnt] & CFG_SP_VENDOR_CNT_)
                    {
                      // Save the number of manufacturer data
                      Vendor_Data_size = uart_buffer[9+cnt] & CFG_SP_VENDOR_CNT_;
                      // Deduct number of total
                      uart_buffer[1] -= Vendor_Data_size;
                    }

                    // I/O Data
                    switch (uart_buffer[9+cnt] & CFG_SP_DIRECTION_)
                    {
                      case CFG_SP_VOID:
                          // Empty data field
                          break;

                      case CFG_SP_INPUT:
                          Input_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Input_Data_size += Input_Data_size*2;
                          cnt++;  // We already have this byte
                          break;

                      case CFG_SP_OUTPUT:
                          Output_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Output_Data_size += Output_Data_size*2;
                          cnt++;  //We already have this byte
                          break;

                      case CFG_SP_INPUT_OPTPUT:
                          // Erst Ausgang...
                          Output_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Output_Data_size += Output_Data_size*2;
                          // Dann Eingang
                          Input_Data_size += (uart_buffer[11+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[11+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Input_Data_size += Input_Data_size*2;
                          cnt += 2;  // We already have these bytes
                          break;

                    } // Switch End
                    break;

                default:
                    Input_Data_size = 0;
                    Output_Data_size = 0;
                    break;

              } // Switch End
            } // For End

            if (Vendor_Data_size != 0)
            {
              // Evaluate
            }
            //In case of error -> send CFG_FAULT_ to diagnosis
            // short acknowledgment
            profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            break;

        default:
            // Unknown SAP
            break;
      } //Switch DSAP_data end
    }
    // Destination: Slave address
    else if (destination_add == slave_addr)
    {
      // Status query
      if (function_code == (REQUEST_ + FDL_STATUS))
      {
        profibus_send_CMD(SD1, FDL_STATUS_OK, 0, &uart_buffer[0], 0);
      }
      // Master sends output data and requests input data(Send and Request Data)
      else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
               function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
      {

        // Read data from master
        #if (INPUT_DATA_SIZE > 0)
        for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
        {
          Profibus_in_register[cnt] = uart_buffer[cnt + 7];
      new_data=1;
        }
        #endif


        // Write data for master in buffer
        #if (OUTPUT_DATA_SIZE > 0)
        for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
        {
          uart_buffer[cnt + 7] = Profibus_out_register[cnt];
        }
        #endif


        #if (OUTPUT_DATA_SIZE > 0)
        if (diagnose_status == true)
          profibus_send_CMD(SD2, DIAGNOSE, 0, &uart_buffer[7], 0);  // Request a diagnosis
        else
          profibus_send_CMD(SD2, DATA_LOW, 0, &uart_buffer[7], Input_Data_size);  // send data
        #else
        if (diagnose_status == true)
          profibus_send_CMD(SD1, DIAGNOSE, 0, &uart_buffer[7], 0);  // Request a diagnosis
        else
          profibus_send_CMD(SC, 0, 0, &uart_buffer[7], 0);          // short acknowledgment
        #endif
      }
    }

  } //Data valid at the end

}

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
  Compile and send * \ brief Profibus telegram
  * \ param type telegram type (SD1, SD2 etc.)
  * \ param function_code Function code to be transmitted
  * \ param sap_offset Value of the SAP offset
  * \ param pdu pointer to data field (PDU)
  * \ param length_pdu Length of pure PDU without DA, SA or FC
  */
void profibus_send_CMD (unsigned char type,
                        unsigned char function_code,
                        unsigned char sap_offset,
                        char *pdu,
                        unsigned char length_pdu)
{
  unsigned char length_data;


  switch(type)
  {
    case SD1:
      uart_buffer[0] = SD1;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      uart_buffer[3] = function_code;
      uart_buffer[4] = checksum(&uart_buffer[1], 3);
      uart_buffer[5] = ED;
      length_data = 6;
      break;

    case SD2:
      uart_buffer[0] = SD2;
      uart_buffer[1] = length_pdu + 3;  // Length of the PDU incl. DA, SA and FC
      uart_buffer[2] = length_pdu + 3;
      uart_buffer[3] = SD2;
      uart_buffer[4] = master_addr;
      uart_buffer[5] = slave_addr + sap_offset;
      uart_buffer[6] = function_code;
      //Data is already filled in before the function is called
      uart_buffer[7+length_pdu] = checksum(&uart_buffer[4], length_pdu + 3);
      uart_buffer[8+length_pdu] = ED;
      length_data = length_pdu + 9;
      break;

    case SD3:
      uart_buffer[0] = SD3;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      uart_buffer[3] = function_code;
      // Data is already filled in before the function is called
      uart_buffer[9] = checksum(&uart_buffer[4], 8);
      uart_buffer[10] = ED;
      length_data = 11;
      break;

    case SD4:
      uart_buffer[0] = SD4;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      length_data = 3;
      break;

    case SC:
      uart_buffer[0] = SC;
      length_data = 1;
      break;

    default:
      break;
  }
  profibus_TX(&uart_buffer[0], length_data);

}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
  Send * \ letter telegram
  * \ param data pointer to data field
  * \ param length Length of the data
  */
void profibus_TX (char *data, unsigned char length)
{
  TX_ENABLE_ON;  // Enable Transmit (Switch to Transmit)
  OCR1A = TIMEOUT_MAX_TX_TIME;  
  profibus_status = PROFIBUS_SEND_DATA;

  uart_byte_cnt = length;         // Number of bytes to send
  uart_transmit_cnt = 0;          // Payer for sent bytes
  UCSR0B |= _BV(UDRIE0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
  Calculate brief checksum. Simple adding of all data bytes in the telegram.
  * \ param data pointer to data field
  * \ param length Length of the data
  * \ return checksum
  */
unsigned char checksum(char *data, unsigned char length)
{
  unsigned char csum = 0;

  while(length--)
  {
    csum += data[length];
  }

  return csum;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
  Check the letter destination. Address must be with slave address or broadcast (including SAP offset)
           to match
  * \ param destination destination address
  * \ return TRUE if destination is ours, FALSE if not
  */
unsigned char addmatch (unsigned char destination)
{
  if ((destination != slave_addr) &&                // Slave
      (destination != slave_addr + SAP_OFFSET) &&   // SAP Slave
      (destination != BROADCAST_ADD) &&             // Broadcast
      (destination != BROADCAST_ADD + SAP_OFFSET)){  // SAP Broadcast
        return false;
      }
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 * \brief ISR TIMER1
 */

ISR (TIMER1_COMPA_vect)    // Timer1 Output Compare 1A.
{
  TIMER1_STOP;  // Guard ourselves.
  switch (profibus_status)
  {
    case PROFIBUS_WAIT_SYN: // TSYN expired
        profibus_status = PROFIBUS_WAIT_DATA;
        OCR1A = TIMEOUT_MAX_SDR_TIME;
        uart_byte_cnt = 0;
        break;

    case PROFIBUS_WAIT_DATA:  // TSDR expired but no data there
        break;

    case PROFIBUS_GET_DATA:   // TSDR expired and data there
        profibus_status = PROFIBUS_WAIT_SYN;
        // We have already waited TIMEOUT_MAX_RX_TIME of bus idle. So subtract that from Tsyn.
        OCR1A = TIMEOUT_MAX_SYN_TIME-TIMEOUT_MAX_RX_TIME;
        profibus_RX(); 
        break;

    case PROFIBUS_SEND_DATA:  // Transmission timeout expired, go back to reception
        profibus_status = PROFIBUS_WAIT_SYN;
        OCR1A = TIMEOUT_MAX_SYN_TIME;   
        TX_ENABLE_OFF;  // Disable Transmit (Switch to Recieve)
        break;

    default:
        break;

  }
  // Timer 1 Start
  TIMER1_RUN;
  TCNT1 = 0;
}

/*!
 * ISR UART0 Receive
 */
ISR(USART_RX_vect)
{
  // Retrieve RXdata to buffer
  uart_buffer[uart_byte_cnt] = UDR0;
  // Only read data after Tsyn have expired
  if (profibus_status == PROFIBUS_WAIT_DATA)
  {
    profibus_status = PROFIBUS_GET_DATA;
  }

  // Read data allowed?
  if (profibus_status == PROFIBUS_GET_DATA)
  {
    uart_byte_cnt++;
    // Check for buffer overflow!
    if(uart_byte_cnt >= MAX_BUFFER_SIZE) uart_byte_cnt--;
  }
  TCNT1 = 0; 
}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 * ISR UART Transmit
 */
ISR(USART_UDRE_vect)
{
  // Everything sent?
  if (uart_transmit_cnt < uart_byte_cnt)
  {
    // TX Buffer
    UDR0 = uart_buffer[uart_transmit_cnt++];    
  }
  else
  {
    // All sent, interrupt again
    UCSR0B &= ~( 1 << UDRIE0 );
  }
  TCNT1=0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 * Initialize UART0
 * Even parity, 1 stop bit.
 */
/* */
void init_UART0(unsigned long baud)
{
  // Try U2X mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  UCSR0A = 1 << U2X0;
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
  {
    UCSR0A = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }
  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;
  UCSR0B |= ( 1 << RXEN0 ) | ( 1 << TXEN0 );
  UCSR0B |= ( 1 << RXCIE0 ) | ( 1 << UDRIE0 );
  //set the data bits, parity, and stop bits
  UCSR0C = 0x26; // 8 bits, EVEN parity and 1 Stop bit - 8E1
}
