#include "delay_x.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
// define data, addr and control ports
#define DATA1_PORT  PORTF
#define DATA1_PIN   PINF
#define DATA1_DDR   DDRF
#define IN           0
#define OUT          1

#define DATA2_PORT  PORTB
#define DATA2_PIN   PINB
#define DATA2_DDR   DDRB

#define ADDR1_PORT  PORTD
#define ADDR1_PIN   PIND
#define ADDR1_DDR   DDRD

#define ADDR2_PORT  PORTC
#define ADDR2_PIN   PINC
#define ADDR2_DDR   DDRC

#define CONT_PORT   PORTA
#define CONT_PIN    PINA
#define CONT_DDR    DDRA

#define RP          0  // ON   = CONT_PORT != (1<<pin)
#define CE          1  // OFF  = CONT_PORT &= ~(1<<pin) 
#define IRQ         2  // ON   = CONT_PORT != CE
#define RDY         3  // OFF  = CONT_PORT &= ~CE
#define AVD         4  // Address Valid
#define CLK         5  // Clock
#define OE          6  // Ouput Enabled
#define WE          7  // Write Enabled


// define register address map 
#define MAN_ID    0xF000  // R Samsung 0xEC
#define DEV_ID    0xF001  // R default is 0x35 for 2B not 2D
#define INFO_ID   0xF006  // R 0x00 SLC, MLS, reserved
#define FBA       0xF100  // R/W FBA[9:0] 1023 Blocks NAND Flash Block Address; Start Address1 Register
#define FPA_FPS   0xF107  // R/W FPA[15:2] FSA[1:0] NAND Fash Page & Sector Address; Start Address8 Register
#define COMM_REG  0xF220  // Command Register 2.8.18
#define SYS_CONF  0xF221  // System Conf. 1 Register 2.8.19
#define WP_STAT   0xF24E  // R default 0x2 2.8.25


// setup pins

void contPortInit() {
  CONT_DDR = 0xFF;            // all control ports are alaways output
  //except
  CONT_DDR &= ~((1<<IRQ) | (1<<RDY));   // INT and RDY pin are input
  
  CONT_PORT &= ~((1<<CLK) | (1<<CE));   // CLK and CE are held LOW
  CONT_PORT |= ((1<<RP) | (1<<OE) | (1<<AVD) | (1<<WE)); // RESET, OE, AVE and WE are HIGH  
}

void dataDir(bool in) {
  if (in == true) 
    DATA1_DDR = DATA2_DDR = 0xFF; // outgoing
  if (in == false) {
    DATA1_PORT = DATA2_PORT = 0;    // set all to 0s
    DATA1_DDR = DATA2_DDR = 0;    // incoming
  }
}   

void addrInit() 
{
  ADDR1_DDR  = ADDR2_DDR  = 0xFF; // address pins are always OUPUT
  ADDR1_PORT = ADDR2_PORT = 0;    // set address to zeroes
}

void coldReset() {
  CONT_PORT &= ~(1<<RP);       // RESET LOW
  delay(1);
  CONT_PORT |= (1<<RP);       // RESET HIGH
  
}

void fbaSet(uint16_t address) {
  dataDir(OUT);
  ADDR2_PORT = 0xF1;  ADDR1_PORT = 0x00;
  DATA2_PORT = (address >> 8);  DATA1_PORT = address & 0xFF;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH
 }

 void fpaSet(uint16_t fpa) {
  registerSet(0xF107, fpa);  
 }

uint16_t registerSet(uint16_t address, uint16_t data) {
  dataDir(OUT);
  CONT_PORT &= ~(1<<WE);    // WE goes HIGH
  ADDR2_PORT = (address >> 8);
  ADDR1_PORT = address & 0xFF;
  DATA2_PORT = (data >> 8);
  DATA1_PORT = data & 0xFF;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  return registerRead(address);
}

uint16_t registerRead(uint16_t address) {
  dataDir(IN);
  ADDR2_PORT = (address >> 8);
  ADDR1_PORT = address & 0xFF;
  CONT_PORT &= ~(1<<AVD); // AVD goes LOW
  CONT_PORT &= ~(1<<OE);  // LOW
  _delay_ns(100);
  //Serial.print("LSB byte is: ");
  //Serial.println(DATA1_PIN, BIN);
  return ((DATA2_PIN<<8) | (DATA1_PIN & 0xFF));       // DATA2_PIN is irrelevat; defaults to 0xEC for SAMSUNG
  
}

void loadPage() {
  dataDir(OUT);
  ADDR2_PORT = ADDR1_PORT = 0x00;           // bootRam addr space
  DATA2_PORT = 0x00; DATA1_PORT = 0xE0;     // 1st cycle command
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  DATA2_PORT = 0x00;  DATA1_PORT = 0x00;    // 2nd cycle command
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  delayMicroseconds(60);
  dataDir(IN);
}


uint8_t buf_read[64], buf_ix; // used to utilize USB buffer size of 64bytes
uint16_t readSector(uint16_t address, bool test) {
  //dataDir(IN);                // moved to loadPage
  ADDR2_PORT = (address >> 8);      // DataRAM0 start address 0x200 ~ 0x500 for total of 4 SECTORS
  ADDR1_PORT = address & 0xFF;
  CONT_PORT &= ~(1<<AVD);     // AVD goes LOW
  CONT_PORT &= ~(1<<OE);      // OE goes LOW
  _delay_ns(100);             // OE enabled to Output Valid ~= 20ns
  buf_read[buf_ix++] = DATA2_PIN;
  buf_read[buf_ix++] = DATA1_PIN;
  //Serial.write(DATA2_PIN);
  //Serial.write(DATA1_PIN);
  CONT_PORT |= (1<<AVD);      // AVD goes HIGH
  CONT_PORT |= (1<<OE);       // OE goes HIGH
  if (buf_ix == 64 && test == 0) {
    Serial.write(buf_read, buf_ix);
    buf_ix = 0;
  }
  return ((DATA2_PIN<<8) | (DATA1_PIN & 0xFF));
}

void writeSector(uint16_t address, uint16_t data) {
  //dataDir(OUT);
  ADDR2_PORT = (address >> 8);      // DataRAM0 start address 0x200 ~ 0x500 for total of 4 SECTORS
  ADDR1_PORT = address & 0xFF;
  DATA2_PORT = (data >> 8);      
  DATA1_PORT = data & 0xFF;   
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  
}

void dump() {
  for ( int block = 0; block < 1024; block++) { 
    fbaSet(block);
    for (int page = 0; page < 0x40; page++) {
      loadPage();
      for (int sector = 0x200; sector < 0x600; sector++) {
        readSector(sector, 0);
      }
    }
  }
}

void bbInfo() {
  for ( int block = 0; block < 0x400; block++) { 
    fbaSet(block);
    for (int page = 0; page < 0x2; page++) {
      loadPage();
      Serial.println(readSector(0x8010, 1), HEX);      
    }
  }
}

void wpStatus() {
  for (int block = 0; block < 0x400; block++) {
    fbaSet(block);
    Serial.println(registerRead(0xF24E), HEX);
  } 
}




/*void bootRAM () {
  loadBuffer();
  for (int i = 0x200; i < 0x600; i++) {
    readSector(i);
  }
} */
    
void testWrite() {
 dataDir(OUT);
 fbaSet(0x3ff);
 fpaSet(0x0000);
 registerSet(0xF200, 0x800); // BSA, BSC set
 for (int s = 0x200; s < 0x600; s++) {
  writeSector(s, 0xAA);
 }
 registerSet(0xF220, 0x0080); // Write command
  while ( (registerRead(0xF241) & (1<<15) )) {};
  while ( (registerRead(0xF241) & (1<<6) )) {};
  if ( registerRead(0xF240) & (1<<10))
    Serial.println("Done writing");
 else 
    Serial.println("Writing failed!!!");
}


void setup() {
 
  //disable JTAG
  MCUCR = (1<<JTD) | (1<<IVCE) | (0<<PUD);
  MCUCR = (1<<JTD) | (0<<IVSEL) | (0<<IVCE) | (0<<PUD);
  contPortInit();
  addrInit();
  Serial.begin(57600);
  delay(10000);
  coldReset();
  Serial.flush();
}

void loop() {
  
 Serial.println(registerRead(0xF200), HEX);
 delay(1000);
 Serial.println(registerRead(0xF241), HEX);
 delay(1000);
 Serial.println(registerRead(MAN_ID), HEX);
 
 
  
   
}

  
 
    



// OLD FUNCTIONS //
/*uint8_t buf_read[64], buf_ix;
void readSector(uint8_t address2, uint8_t address1) {
  dataDir(IN);
  ADDR2_PORT = address2;
  ADDR1_PORT = address1;
  CONT_PORT &= ~(1<<AVD); // AVD goes LOW
  //delayMicroseconds(1);
  CONT_PORT &= ~(1<<OE); // OE goes LOW
  //delayMicroseconds(1);
  _delay_ns(100);
  buf_read[buf_ix++] = DATA2_PIN;
  buf_read[buf_ix++] = DATA1_PIN;
  //Serial.write(DATA2_PIN);
  //Serial.write(DATA1_PIN);
  CONT_PORT |= (1<<AVD); // AVD goes HIGH
  CONT_PORT |= (1<<OE); // OE goes HIGH
  //delayMicroseconds(1);
  if (buf_ix == 64) {
    Serial.write(buf_read, buf_ix);
    buf_ix = 0;
  }
}

void loadBuffer() {
  dataDir(OUT);
  ADDR2_PORT = ADDR1_PORT = 0x00;
  DATA2_PORT = 0x00;
  DATA1_PORT = 0xE0;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
 //delayMicroseconds(1);
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  //delayMicroseconds(1);
  DATA2_PORT = 0x00;
  DATA1_PORT = 0x00;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  //delayMicroseconds(1);
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  delayMicroseconds(60);
}


*/
  
