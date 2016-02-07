#include "delay_x.h"

// set CPU speed to 8Mhz as teensy is runing on 3.3V
#define CPU_PRESCALE(n)  (CLKPR = 0x80, CLKPR = (n))

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
#define INFO_ID   0xF006  // R 0x00 SLC, MLC, reserved
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

void dataDir(bool dir) {
  if (dir == true) 
    DATA1_DDR = DATA2_DDR = 0xFF; // outgoing
    DATA1_DDR = DATA2_DDR = 0;    // clear
  if (dir == false) {
    DATA1_PORT = DATA2_PORT = 0;  // input
    DATA1_DDR = DATA2_DDR = 0;    // clear
  }
}   

void addrInit() {
  ADDR1_DDR  = ADDR2_DDR  = 0xFF; // address pins are always OUPUT
  ADDR1_PORT = ADDR2_PORT = 0;    // set address to zeroes
}

void coldReset() {
  CONT_PORT &= ~(1<<RP);       // RESET LOW
  delay(1);
  CONT_PORT |= (1<<RP);       // RESET HIGH
}

void addrSet(uint16_t address) {
  ADDR2_PORT = address & 0xFF;
  ADDR1_PORT = address >> 8;
}



void fbaSet(uint16_t block) {  // set FlashBlockAddress register
  dataDir(OUT);
  CONT_PORT &= ~(1<<WE);    // WE goes HIGH
  ADDR2_PORT = 0xF1;
  ADDR1_PORT = 0x00;
  DATA2_PORT = (block >> 8);
  DATA1_PORT = (block & 0xFF);
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  
}

void blockIncremet(uint16_t block) {
  if (block >= 0x3FF)
    return;
   if (block == 0)
    fbaSet(block);
  else 
    fbaSet(block + 1);
}

uint16_t registerSet(uint16_t address, uint16_t data) {
  dataDir(OUT);
  CONT_PORT &= ~(1<<WE);    // WE goes HIGH
  ADDR2_PORT = (address >> 8);
  ADDR1_PORT = (address & 0xFF);
  DATA2_PORT = (data >> 8);
  DATA1_PORT = (data & 0xFF);
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  return registerRead(address);
}

uint16_t registerRead(uint16_t address) {
  dataDir(IN);
  ADDR2_PORT = (address & 0xFF);
  ADDR1_PORT = (address >> 8);
  CONT_PORT &= ~(1<<AVD); // AVD goes LOW
  CONT_PORT &= ~(1<<OE);  // LOW
  _delay_ns(100);
  //Serial.print("LSB byte is: ");
  //Serial.println(DATA1_PIN, BIN);
  return ((DATA2_PIN<<8) | (DATA1_PIN & 0xFF));       // DATA2_PIN is irrelevat; defaults to 0xEC for SAMSUNG
}

void loadPage() {           // this is embeded command to load one page into DataRAM0 buffer. Works within a BLOCK bounds.
  dataDir(OUT);
  CONT_PORT &= ~(1<<WE);    // WE goes HIGH
  ADDR2_PORT = ADDR1_PORT = 0x00;
  DATA2_PORT = 0x00; DATA1_PORT = 0xE0;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  DATA2_PORT = 0x00; DATA1_PORT = 0x00;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  CONT_PORT |= (1<<WE);     // WE goes HIGH. It's better to rise it here and not in read command(loop) which must follow!
  delayMicroseconds(200);   // takes some time to load a PAGE; Trd2 = 45micro seconds
  dataDir(IN);              // usualy folows read command
}

uint8_t buf_read[64], buf_ix; // used to utilize USB packet size of 64bytes
void readSector(uint16_t address) {
  //dataDir(IN);                // moved to loadPage
  ADDR2_PORT = (address >> 8);      // DataRAM0 start address 0x200 ~ 0x500 for total of 4 SECTORS
  ADDR1_PORT = (address & 0xFF);
  CONT_PORT &= ~(1<<AVD);     // AVD goes LOW
  CONT_PORT &= ~(1<<OE);      // OE goes LOW
  _delay_ns(100);             // OE enabled to Output Valid ~= 20ns
  buf_read[buf_ix++] = DATA2_PIN;
  buf_read[buf_ix++] = DATA1_PIN;
  //Serial.write(DATA2_PIN);
  //Serial.write(DATA1_PIN);
  CONT_PORT |= (1<<AVD);      // AVD goes HIGH
  CONT_PORT |= (1<<OE);       // OE goes HIGH
  if (buf_ix == 64) {
    Serial.write(buf_read, buf_ix);
    buf_ix = 0;
  }
}

void dump() { 
  // dump chip = 128MB
  for ( int block = 0; block < 0x400; block++) {
    fbaSet(block);
    for (int page = 0; page < 0x40; page++) {
      loadPage();
      for (int address = 0x200; address < 0x600; address++) { 
        readSector(address);
      }
    }
  }
}
        

void bootRAM () {
  loadPage();
  for (int address = 0x200; address < 0x600; address++) { 
        readSector(address);
  }
}
  


void setup() {
 
  // set for 8 MHz clock
  CPU_PRESCALE(1);
  // set for 16 MHz clock
  //CPU_PRESCALE(0);
  //disable JTAG
  MCUCR = (1<<JTD) | (1<<IVCE) | (0<<PUD);
  MCUCR = (1<<JTD) | (0<<IVSEL) | (0<<IVCE) | (0<<PUD);
  contPortInit();
  addrInit();
  coldReset();
  Serial.begin(9600);
  delay(10000);
  
}

void loop() {
  
  registerRead(MAN_ID);
   
}

  
 
    
// ====== TEST FUNCTIONS ======
/* void syncMode() {
  dataDir(OUT);
  ADDR2_PORT = 0xF2;
  ADDR1_PORT = 0x21;
  DATA2_PORT = 0xC0;
  DATA1_PORT = 0xC0;
  CONT_PORT &= ~(1<<WE);    // WE goes LOW
  //delayMicroseconds(1);
  CONT_PORT |= (1<<WE);     // WE goes HIGH
  //delayMicroseconds(1);
  ADDR2_PORT = 0xF2;
  ADDR1_PORT = 0x21;
  CONT_PORT &= ~(1<<AVD); // AVD goes LOW
 // delayMicroseconds(1);
  CONT_PORT &= ~(1<<OE); // OE goes LOW
  _delay_ns(100);
  //Serial.print(DATA2_PIN, HEX);
  //Serial.println(DATA1_PIN, HEX);
  CONT_PORT |= (1<<AVD); // AVD goes HIGH
  CONT_PORT |= (1<<OE); // OE goes HIGH
  //delayMicroseconds(1);
  
  
}
void clkSet() {
   CONT_PORT &= ~(1<<CE);   // CE LOW
   CONT_PORT &= ~(1<<AVD);  // AVD LOW
   CONT_PORT |= (1<<CLK);   // CLK HIGH -1
   delayMicroseconds(1);
   CONT_PORT |= (1<<AVD);   // AVD HIGH
   CONT_PORT &= ~(1<<CLK);   // CLK LOW -1

   CONT_PORT |= (1<<CLK);   // CLK HIGH 0
   CONT_PORT &= ~(1<<OE);   // OE LOW
   CONT_PORT &= ~(1<<CLK);   // CLK LOW 0

   CONT_PORT |= (1<<CLK);   // CLK HIGH 1
   CONT_PORT &= ~(1<<CLK);   // CLK LOW 1

   CONT_PORT |= (1<<CLK);   // CLK HIGH 2
   CONT_PORT &= ~(1<<CLK);   // CLK LOW 2

   CONT_PORT |= (1<<CLK);   // CLK HIGH 3
   CONT_PORT &= ~(1<<CLK);   // CLK LOW 3

   CONT_PORT |= (1<<CLK);   // CLK HIGH 4
   CONT_PORT &= ~(1<<CLK);   // CLK LOW 4
}

void clkPulse() {
    CONT_PORT |= (1<<CLK);   // CLK HIGH 
    CONT_PORT &= ~(1<<CLK);   // CLK LOW 
} */

