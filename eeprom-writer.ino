// EEPROM Programmer - code for an Arduino Mega 2560
//
// Written by K Adcock.
// Distributed under an acknowledgement licence, because I'm a shallow, attention-seeking tart. :)
//       Jan 2016 - Initial release
//       Dec 2017 - Slide code tartups, to remove compiler errors for new Arduino IDE (1.8.5).
//   7th Dec 2017 - Updates from Dave Curran of Tynemouth Software, adding commands to enable/disable SDP.
//  10th Dec 2017 - Fixed one-byte EEPROM corruption (always byte 0) when unprotecting an EEPROM
//                  (doesn't matter if you write a ROM immediately after, but does matter if you use -unprotect in isolation)
//                - refactored code a bit (split loop() into different functions)
//                - properly looked at timings on the Atmel datasheet, and worked out that my delays
//                  during reads and writes were about 10,000 times too big!
//                  Reading and writing is now orders-of-magnitude quicker.
//
//  Improved (?) by JOliverasC
//  20th June 2019 - Modify Protect / Unprotect function (address changed to include full range) 
//                 - Add Erase EEPROM by software (if supported by the device!
//                 - Add commands to support Page Write functions
//                 - Increase revision number from 0.2 to 0.3
//
//
// http://danceswithferrets.org/geekblog/?page_id=903
//
// This software presents a 9600-8N1 serial port.
//
// R[hex address]                         - reads 16 bytes of data from the EEPROM
// W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the EEPROM
// P                                      - set write-protection bit (if supported)
// U                                      - clear write-protection bit (ditto)
// V                                      - prints the version string
//
// B                                      - Erases EEPROM (if supported)
// N[hex address]                         - Prepare buffer for Page Write (if supported)
// W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to buffer
// P                                      - Transfers buffer to EEPROM 
//
// Any data read from the EEPROM will have a CRC checksum appended to it (separated by a comma).
// If a string of data is sent with an optional checksum, then this will be checked
// before anything is written.
//

#include <avr/pgmspace.h>

const char hex[] =
{
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

const char version_string[] = {"EEPROM Version=0.03"};

static const int kPin_Addr14  = 24;
static const int kPin_Addr12  = 26;
static const int kPin_Addr7   = 28;
static const int kPin_Addr6   = 30;
static const int kPin_Addr5   = 32;
static const int kPin_Addr4   = 34;
static const int kPin_Addr3   = 36;
static const int kPin_Addr2   = 38;
static const int kPin_Addr1   = 40;
static const int kPin_Addr0   = 42;
static const int kPin_Data0   = 44;
static const int kPin_Data1   = 46;
static const int kPin_Data2   = 48;
static const int kPin_nWE     = 27;
static const int kPin_Addr13  = 29;
static const int kPin_Addr8   = 31;
static const int kPin_Addr9   = 33;
static const int kPin_Addr11  = 35;
static const int kPin_nOE     = 37;
static const int kPin_Addr10  = 39;
static const int kPin_nCE     = 41;
static const int kPin_Data7   = 43;
static const int kPin_Data6   = 45;
static const int kPin_Data5   = 47;
static const int kPin_Data4   = 49;
static const int kPin_Data3   = 51;
static const int kPin_WaitingForInput  = 13;
static const int kPin_LED_Red = 22;
static const int kPin_LED_Grn = 53;

byte g_cmd[80]; // strings received from the controller will go in here
static const int kMaxBufferSize = 16;
byte buffer[kMaxBufferSize];

static const int kPageSize = 128;  // Variables to hold data for Page Write mode
byte pageWr[kPageSize];
int  baseAddr=0;

static const long int k_uTime_WritePulse_uS = 1; 
static const long int k_uTime_ReadPulse_uS = 1;
// (to be honest, both of the above are about ten times too big - but the Arduino won't reliably
// delay down at the nanosecond level, so this is the best we can do.)

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);

  pinMode(kPin_WaitingForInput, OUTPUT); digitalWrite(kPin_WaitingForInput, HIGH);
  pinMode(kPin_LED_Red, OUTPUT); digitalWrite(kPin_LED_Red, LOW);
  pinMode(kPin_LED_Grn, OUTPUT); digitalWrite(kPin_LED_Grn, LOW);

  // address lines are ALWAYS outputs
  pinMode(kPin_Addr0,  OUTPUT);
  pinMode(kPin_Addr1,  OUTPUT);
  pinMode(kPin_Addr2,  OUTPUT);
  pinMode(kPin_Addr3,  OUTPUT);
  pinMode(kPin_Addr4,  OUTPUT);
  pinMode(kPin_Addr5,  OUTPUT);
  pinMode(kPin_Addr6,  OUTPUT);
  pinMode(kPin_Addr7,  OUTPUT);
  pinMode(kPin_Addr8,  OUTPUT);
  pinMode(kPin_Addr9,  OUTPUT);
  pinMode(kPin_Addr10, OUTPUT);
  pinMode(kPin_Addr11, OUTPUT);
  pinMode(kPin_Addr12, OUTPUT);
  pinMode(kPin_Addr13, OUTPUT);
  pinMode(kPin_Addr14, OUTPUT);

  // control lines are ALWAYS outputs
  pinMode(kPin_nCE, OUTPUT); digitalWrite(kPin_nCE, LOW); // might as well keep the chip enabled ALL the time
  pinMode(kPin_nOE, OUTPUT); digitalWrite(kPin_nOE, HIGH);
  pinMode(kPin_nWE, OUTPUT); digitalWrite(kPin_nWE, HIGH); // not writing

  SetDataLinesAsInputs();
  SetAddress(0);  
}

void loop()
{
  while (true)
  {
    digitalWrite(kPin_WaitingForInput, HIGH);
    ReadString();
    digitalWrite(kPin_WaitingForInput, LOW);
    
    switch (g_cmd[0])
    {
      case 'V': Serial.println(version_string); break;
      case 'P': SetSDPState(true);  break;
      case 'U': SetSDPState(false); break;
      case 'R': ReadEEPROM();       break;
      case 'W': WriteEEPROM();      break;
      case '?': Help();             break;
      
      case 'B': BlankEEPROM();      break;  // Only if supported 
      //=========================================== Page Write
      case 'N': PrepareBuffer();    break;  // N[hex address]
      case 'S': FillBuffer();       break;  // S[hex address]:[data in two-char hex]
      case 'T': ProgramBuffer();    break;  // T
      //=========================================== 
      case 0: break; // empty string. Don't mind ignoring this.
      default: Serial.println("ERR Unrecognised command"); break;
    }
  }
}

void Help()
{
  Serial.println(version_string);
  Serial.println("   R[hex address]                         - reads 16 bytes of data from the EEPROM");
  Serial.println("   W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the EEPROM");
  Serial.println("   P                                      - set write-protection bit (SDS only, AFAIK)");
  Serial.println("   U                                      - clear write-protection bit (ditto)");
  Serial.println("   V                                      - prints the version string");
  Serial.println("   B                                      - blanks Silicon Storage Technology EEPROM");
  Serial.println("   N[hex address]                         - init buffer for EEPROM Page Write");
  Serial.println("   S[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the buffer");
  Serial.println("   T                                      - Writes buffer to EEPROM");
  Serial.println("   ?                                      - print this lines");
  Serial.println("");
  Serial.println("Any data read from the EEPROM will have a CRC checksum appended to it (separated by a comma).");
  Serial.println("If a string of data is sent with an optional checksum, then this will be checked");
  Serial.println("before anything is written.");
}

void ReadEEPROM() // R<address>  - read kMaxBufferSize bytes from EEPROM, beginning at <address> (in hex)
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  // decode ASCII representation of address (in hex) into an actual value
  int addr = 0;
  int x = 1;
  while (x < 5 && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x++]);
  }     

  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsInputs();
  digitalWrite(kPin_nOE, LOW); // makes the EEPROM output the byte
  delayMicroseconds(1);
      
  ReadEEPROMIntoBuffer(addr, kMaxBufferSize);

  // now print the results, starting with the address as hex ...
  Serial.print(hex[ (addr & 0xF000) >> 12 ]);
  Serial.print(hex[ (addr & 0x0F00) >> 8  ]);
  Serial.print(hex[ (addr & 0x00F0) >> 4  ]);
  Serial.print(hex[ (addr & 0x000F)       ]);
  Serial.print(":");
  PrintBuffer(kMaxBufferSize);

  Serial.println("OK");

  digitalWrite(kPin_nOE, HIGH); // stops the EEPROM outputting the byte
}

void WriteEEPROM() // W<four byte hex address>:<data in hex, two characters per byte, max of 16 bytes per line>
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  int addr = 0;
  int x = 1;
  while (g_cmd[x] != ':' && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x]);
    ++x;
  }

  // g_cmd[x] should now be a :
  if (g_cmd[x] != ':')
  {
    Serial.println("ERR");
    return;
  }
  
  x++; // now points to beginning of data
  uint8_t iBufferUsed = 0;
  while (g_cmd[x] && g_cmd[x+1] && iBufferUsed < kMaxBufferSize && g_cmd[x] != ',')
  {
    uint8_t c = (HexToVal(g_cmd[x]) << 4) | HexToVal(g_cmd[x+1]);
    buffer[iBufferUsed++] = c;
    x += 2;
  }

  // if we're pointing to a comma, then the optional checksum has been provided!
  if (g_cmd[x] == ',' && g_cmd[x+1] && g_cmd[x+2])
  {
    byte checksum = (HexToVal(g_cmd[x+1]) << 4) | HexToVal(g_cmd[x+2]);

    byte our_checksum = CalcBufferChecksum(iBufferUsed);

    if (our_checksum != checksum)
    {
      // checksum fail!
      iBufferUsed = -1;
      Serial.print("ERR ");
      Serial.print(checksum, HEX);
      Serial.print(" ");
      Serial.print(our_checksum, HEX);
      Serial.println("");
      return;
    }
  }

  // buffer should now contains some data
  if (iBufferUsed > 0)
  {
    WriteBufferToEEPROM(addr, iBufferUsed);
  }

  if (iBufferUsed > -1)
  {
    Serial.println("OK");
  }
}

// Important note: the EEPROM needs to have data written to it immediately after sending the "unprotect" command, so that the buffer is flushed.
// So we read byte 0 from the EEPROM first, then use that as the dummy write afterwards.
// It wouldn't matter if this facility was used immediately before writing an EEPROM anyway ... but it DOES matter if you use this option
// in isolation (unprotecting the EEPROM but not changing it).

// JEDEC Standard Software Data Protection (SDP)
void SetSDPState(bool bWriteProtect)
{
  digitalWrite(kPin_LED_Red, HIGH);

  digitalWrite(kPin_nWE, HIGH); // disables write
  digitalWrite(kPin_nOE, LOW); // makes the EEPROM output the byte
  SetDataLinesAsInputs();
  
  byte bytezero = ReadByteFrom(0);
  
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nCE, HIGH);
  SetDataLinesAsOutputs();

  if (bWriteProtect)
  {
    // SDP standard
    WriteByteTo(0x5555, 0xAA);
    WriteByteTo(0x2AAA, 0x55);
    WriteByteTo(0x5555, 0xA0);
  }
  else
  {
    // SDP standard
    WriteByteTo( 0x5555, 0xAA );
    WriteByteTo( 0x2AAA, 0x55 );
    WriteByteTo( 0x5555, 0x80 );
    WriteByteTo( 0x5555, 0xAA );
    WriteByteTo( 0x2AAA, 0x55 );
    WriteByteTo( 0x5555, 0x20 );
  }
  
  WriteByteTo(0x0000, bytezero); // this "dummy" write is required so that the EEPROM will flush its buffer of commands.

  digitalWrite(kPin_nCE, LOW); // return to on by default for the rest of the code
  digitalWrite(kPin_LED_Red, LOW);

  Serial.print("OK SDP ");
  if (bWriteProtect)
  {
    Serial.println("enabled");
  }
  else
  {
    Serial.println("disabled");
  }
}

// ----------------------------------------------------------------------------------------
void BlankEEPROM()
{
 // Software Chip Erase 5555H AAH 2AAAH 55H 5555H 80H 5555H AAH 2AAAH 55H 5555H 10H 
 // SST29EE010 and others??
  digitalWrite(kPin_LED_Red, HIGH);

  digitalWrite(kPin_nWE, HIGH); // disables write
  digitalWrite(kPin_nOE, LOW); // makes the EEPROM output the byte
  SetDataLinesAsInputs();
  
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nCE, HIGH);
  SetDataLinesAsOutputs();
//----------------
  WriteByteTo(0x5555, 0xAA); 
  WriteByteTo(0x2AAA, 0x55); 
  WriteByteTo(0x5555, 0x80); 
  WriteByteTo(0x5555, 0xAA); 
  WriteByteTo(0x2AAA, 0x55); 
  WriteByteTo(0x5555, 0x10);
//----------------

  digitalWrite(kPin_nCE, LOW); // return to on by default for the rest of the code
  digitalWrite(kPin_LED_Red, LOW); 

  Serial.println("OK");
}
// ----------------------------------------------------------------------------------------

void ReadEEPROMIntoBuffer(int addr, int size)
{
  digitalWrite(kPin_LED_Grn, HIGH);
  digitalWrite(kPin_nWE, HIGH);
  SetDataLinesAsInputs();
  digitalWrite(kPin_nOE, LOW);
  
  for (int x = 0; x < size; ++x)
  {
    buffer[x] = ReadByteFrom(addr + x);
  }

  digitalWrite(kPin_nOE, HIGH);
  digitalWrite(kPin_LED_Grn, LOW);
}

void WriteBufferToEEPROM(int addr, int size)
{
  digitalWrite(kPin_LED_Red, HIGH);
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsOutputs();

  for (uint8_t x = 0; x < size; ++x)
  {
    WriteByteTo(addr + x, buffer[x]);
  }
  
  digitalWrite(kPin_LED_Red, LOW);
}

// ----------------------------------------------------------------------------------------

// this function assumes that data lines have already been set as INPUTS, and that
// nOE is set LOW.
byte ReadByteFrom(int addr)
{
  SetAddress(addr);
  digitalWrite(kPin_nCE, LOW);
  delayMicroseconds(k_uTime_ReadPulse_uS);
  byte b = ReadData();
  digitalWrite(kPin_nCE, HIGH);

  return b;
}

// this function assumes that data lines have already been set as OUTPUTS, and that
// nOE is set HIGH.
void WriteByteTo(int addr, byte b)
{
  SetAddress(addr);
  SetData(b);

  digitalWrite(kPin_nCE, LOW);
  digitalWrite(kPin_nWE, LOW); // enable write  
  delayMicroseconds(k_uTime_WritePulse_uS);
  
  digitalWrite(kPin_nWE, HIGH); // disable write
  digitalWrite(kPin_nCE, HIGH);
}

// ----------------------------------------------------------------------------------------

void SetDataLinesAsInputs()
{
  pinMode(kPin_Data0, INPUT);
  pinMode(kPin_Data1, INPUT);
  pinMode(kPin_Data2, INPUT);
  pinMode(kPin_Data3, INPUT);
  pinMode(kPin_Data4, INPUT);
  pinMode(kPin_Data5, INPUT);
  pinMode(kPin_Data6, INPUT);
  pinMode(kPin_Data7, INPUT);
}

void SetDataLinesAsOutputs()
{
  pinMode(kPin_Data0, OUTPUT);
  pinMode(kPin_Data1, OUTPUT);
  pinMode(kPin_Data2, OUTPUT);
  pinMode(kPin_Data3, OUTPUT);
  pinMode(kPin_Data4, OUTPUT);
  pinMode(kPin_Data5, OUTPUT);
  pinMode(kPin_Data6, OUTPUT);
  pinMode(kPin_Data7, OUTPUT);
}

void SetAddress(int a)
{
  digitalWrite(kPin_Addr0,  (a&1)?HIGH:LOW    );
  digitalWrite(kPin_Addr1,  (a&2)?HIGH:LOW    );
  digitalWrite(kPin_Addr2,  (a&4)?HIGH:LOW    );
  digitalWrite(kPin_Addr3,  (a&8)?HIGH:LOW    );
  digitalWrite(kPin_Addr4,  (a&16)?HIGH:LOW   );
  digitalWrite(kPin_Addr5,  (a&32)?HIGH:LOW   );
  digitalWrite(kPin_Addr6,  (a&64)?HIGH:LOW   );
  digitalWrite(kPin_Addr7,  (a&128)?HIGH:LOW  );
  digitalWrite(kPin_Addr8,  (a&256)?HIGH:LOW  );
  digitalWrite(kPin_Addr9,  (a&512)?HIGH:LOW  );
  digitalWrite(kPin_Addr10, (a&1024)?HIGH:LOW );
  digitalWrite(kPin_Addr11, (a&2048)?HIGH:LOW );
  digitalWrite(kPin_Addr12, (a&4096)?HIGH:LOW );
  digitalWrite(kPin_Addr13, (a&8192)?HIGH:LOW );
  digitalWrite(kPin_Addr14, (a&16384)?HIGH:LOW);
}

// this function assumes that data lines have already been set as OUTPUTS.
void SetData(byte b)
{
  digitalWrite(kPin_Data0, (b&1)?HIGH:LOW  );
  digitalWrite(kPin_Data1, (b&2)?HIGH:LOW  );
  digitalWrite(kPin_Data2, (b&4)?HIGH:LOW  );
  digitalWrite(kPin_Data3, (b&8)?HIGH:LOW  );
  digitalWrite(kPin_Data4, (b&16)?HIGH:LOW );
  digitalWrite(kPin_Data5, (b&32)?HIGH:LOW );
  digitalWrite(kPin_Data6, (b&64)?HIGH:LOW );
  digitalWrite(kPin_Data7, (b&128)?HIGH:LOW);
}

// this function assumes that data lines have already been set as INPUTS.
byte ReadData()
{
  byte b = 0;

  if (digitalRead(kPin_Data0) == HIGH) b |= 1;
  if (digitalRead(kPin_Data1) == HIGH) b |= 2;
  if (digitalRead(kPin_Data2) == HIGH) b |= 4;
  if (digitalRead(kPin_Data3) == HIGH) b |= 8;
  if (digitalRead(kPin_Data4) == HIGH) b |= 16;
  if (digitalRead(kPin_Data5) == HIGH) b |= 32;
  if (digitalRead(kPin_Data6) == HIGH) b |= 64;
  if (digitalRead(kPin_Data7) == HIGH) b |= 128;

  return(b);
}

// ----------------------------------------------------------------------------------------
// Aux function for debbuging purposes ... currently not used
void PrintWord(int pWord)
{
   PrintByte((pWord & 0xFF00)>>8);
   PrintByte((pWord & 0xFF  )   );
   Serial.print(":");
}

void PrintByte(byte pByte)
{
  Serial.print(hex[(pByte & 0xF0)>>4]);
  Serial.print(hex[(pByte & 0x0F)   ]);
}
// ----------------------------------------------------------------------------------------

void PrintBuffer(int size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    Serial.print(hex[ (buffer[x] & 0xF0) >> 4 ]);
    Serial.print(hex[ (buffer[x] & 0x0F)      ]);

    chk = chk ^ buffer[x];
  }

  Serial.print(",");
  Serial.print(hex[ (chk & 0xF0) >> 4 ]);
  Serial.print(hex[ (chk & 0x0F)      ]);
  Serial.println("");
}

void ReadString()
{
  int i = 0;
  byte c;

  g_cmd[0] = 0;
  do
  {
    if (Serial.available())
    {
      c = Serial.read();
      if (c > 31)
      {
        g_cmd[i++] = c;
        g_cmd[i] = 0;
      }
    }
  } 
  while (c != 10);
}

uint8_t CalcBufferChecksum(uint8_t size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    chk = chk ^  buffer[x];
  }

  return(chk);
}

// converts one character of a HEX value into its absolute value (nibble)
byte HexToVal(byte b)
{
  if (b >= '0' && b <= '9') return(b - '0');
  if (b >= 'A' && b <= 'F') return((b - 'A') + 10);
  if (b >= 'a' && b <= 'f') return((b - 'a') + 10);
  return(0);
}

//============================================================================
// Page Write commands

void PrepareBuffer()
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  // decode ASCII representation of address (in hex) into an actual value
  baseAddr = 0;
  int x = 1;
  while (x < 5 && g_cmd[x] != 0)
  {
    baseAddr = baseAddr << 4;
    baseAddr |= HexToVal(g_cmd[x++]);
  }     

  for (uint8_t x = 0; x < kPageSize; ++x)
  {
    pageWr[kPageSize]=0xFF;  
  }
  Serial.println("OK");
}

void FillBuffer()// S<four byte hex address>:<data in hex, two characters per byte, max of 16 bytes per line>
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    return;
  }

  int addr = 0;
  int x = 1;
  while (g_cmd[x] != ':' && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x]);
    ++x;
  }

  // g_cmd[x] should now be a :
  if (g_cmd[x] != ':')
  {
    Serial.println("ERR");
    return;
  }
  
  x++; // now points to beginning of data
  uint8_t iBufferUsed = 0;
  while (g_cmd[x] && g_cmd[x+1] && iBufferUsed < kMaxBufferSize && g_cmd[x] != ',')
  {
    int8_t c = (HexToVal(g_cmd[x]) << 4) | HexToVal(g_cmd[x+1]);
    buffer[iBufferUsed++] = c;
    x += 2;
  }

  // if we're pointing to a comma, then the optional checksum has been provided!
  if (g_cmd[x] == ',' && g_cmd[x+1] && g_cmd[x+2])
  {
    byte checksum = (HexToVal(g_cmd[x+1]) << 4) | HexToVal(g_cmd[x+2]);

    byte our_checksum = CalcBufferChecksum(iBufferUsed);

    if (our_checksum != checksum)
    {
      // checksum fail!
      iBufferUsed = -1;
      Serial.print("ERR ");
      Serial.print(checksum, HEX);
      Serial.print(" ");
      Serial.print(our_checksum, HEX);
      Serial.println("");
      return;
    }
  }
  
  iBufferUsed++;
  while (iBufferUsed>0)
  {
    iBufferUsed--;
    pageWr[(addr & 0x7F)+ iBufferUsed] = buffer[iBufferUsed];
  }
  Serial.println("OK");
}

void ProgramBuffer()
{
  digitalWrite(kPin_LED_Red, HIGH);
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsOutputs();

  for (uint8_t x = 0; x < kPageSize; ++x)
  {
    WriteByteTo(baseAddr + x, pageWr[x]);    // Programa pageWr[(addr & 0x7F)  @ baseAddr
  }  
  delayMicroseconds(10000);  // Espera TWC 10 ms (?) 
  digitalWrite(kPin_LED_Red, LOW);
  Serial.println("OK");
}
