/*
ActisenseASCIIReader.cpp

Copyright (c) 2015-2024 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is class for reading Actisense format messages from given stream.
*/
#include "ActisenseASCIIReader.h"
#include "N2kTimer.h"
#include <sensesp.h>

using namespace sensesp;



tActisenseASCIIReader::tActisenseASCIIReader() {
  DefaultSource=65;
  ReadStream=0;
  ClearBuffer();
}

void tActisenseASCIIReader::readStringUntil(char terminator) {
    if (ReadStream == 0) return;

    int c = ReadStream->read();
    while(c >= 0 && c != terminator) {
        AddByteToBuffer (c);
        c = ReadStream->read();
    }
    AddByteToBuffer(0);
    if (MsgBuf[0] != 0) debugD ("Returning %s",MsgBuf);

}


//*****************************************************************************
void tActisenseASCIIReader::ClearBuffer() {
  MsgWritePos=0;
}

//*****************************************************************************
bool tActisenseASCIIReader::AddByteToBuffer(char NewByte) {
  if (MsgWritePos>=MAX_STREAM_MSG_BUF_LEN) return false;

  MsgBuf[MsgWritePos]=NewByte;
  MsgWritePos++;

  return true;
}

#define Escape 'A'


//*****************************************************************************
// Read Actisense formatted NMEA2000 message from stream
//*****************************************************************************
/*
Decodes a message fomatted in ActiSense ASCII, similarly to the below 

A173321.107 23FF7 1F513 012F3070002F30709F        <CR><LF>


Where
A173321.107
A = Message is an N2K (or J1939) data PGN
(we only do A ...)

Time = 17:33:21.107 (ignored)

23FF7
23 = Message came from source address = 0x23
FF = Message went to destination address = 0xff (Global)
7 = Message was priority 7

1F513 – Decoded PGN number

012F3070002F30709F = data payload, 9 bytes
” ” – optional whitespace for example purposes – receiver must ignore.

<CR><LF> – end delimiters*/
bool tActisenseASCIIReader::GetMessageFromStream(tN2kMsg &N2kMsg) {
  bool result=false;

  if (ReadStream==0) return false;
  ClearBuffer();
  readStringUntil('\n');
  if (MsgBuf[0] == '\0') return result;
  
  return ParseMessage (N2kMsg,(char *)MsgBuf);
}

//*****************************************************************************
void tActisenseASCIIReader::ParseMessages() {
  tN2kMsg N2kMsg;

    while (GetMessageFromStream(N2kMsg)) {
      
      if (MsgHandler!=0) MsgHandler(N2kMsg);
    }
}

/**
 * @brief converts an ActiSense ASCII string into the buffer from the N2KMSG
 *
 * @param N2kMsg    the message to receive the conversion
 * @param buffer    the buffer to receive the conversion
 * @return true     when all OK
 */
bool tActisenseASCIIReader::ParseMessage(tN2kMsg &N2kMsg, char *buffer) {
  bool result = false;

  debugD("Parsing message [%s] length %d", buffer, strlen(buffer));

  if (buffer[0] != Escape) {
    debugE("Invalid ActiSense message starter [%x]. Are you sending ASCII?",
           buffer[0]);
    return false;
  }
  if (strlen((char *)buffer) < 24) {
    debugE("Invalid ActiSense message length [%d]. Are you sending ASCII? %x",
           strlen((char *)buffer));
    return false;
  }
 
  N2kMsg.Clear();   // start with a clean sheet

  // get the timestamp out
  char timestamp[11];  // 10 characters + null terminator
  strncpy(timestamp,buffer + 1, 10);
  timestamp[10] = '\0';
  debugV("Processing message with timestamp %s", timestamp);

  // get the address out
  char address[6];  // 5 characters + null terminator
  strncpy(address, buffer + 12, 5);
  address[5] = '\0';
  debugV("Address is %s", address);

  // split address in source, destination and prio
  char srcStr[3];
  strncpy(srcStr, address, 2);
  srcStr[2] = '\0';
  N2kMsg.Source = strtol(srcStr, nullptr, 16);
  debugV("Source is %s", srcStr);

  char destStr[3];
  strncpy(destStr, address + 2, 2);
  destStr[2] = '\0';
  N2kMsg.Destination = strtol(destStr, nullptr, 16);
  debugV("Dest is %s", destStr);

  char prioStr[2];
  prioStr[0] = address[4];
  prioStr[1] = '\0';
  N2kMsg.Priority = strtol(prioStr, nullptr, 16);
  debugV("Prio is %s", prioStr);

  // get the PGN
  char pgns[6];  // 5 characters + null terminator
  strncpy(pgns,buffer + 18, 5);
  pgns[5] = '\0';
  N2kMsg.PGN = strtol(pgns, nullptr, 16);
  debugD("PGN is %d [%s]", N2kMsg.PGN, pgns);

  // .. and finally the payload
  for (int i = 24; i < strlen(buffer); i += 2) {
    char byteStr[3];
    strncpy(byteStr, buffer + i, 2);
    byteStr[2] = '\0';

    // Check if the byteStr is of length 2 (not empty or shorter)
    if (strlen(byteStr) == 2) {
      char byteValue = (char)strtol(byteStr, nullptr, 16);
      N2kMsg.AddByte(byteValue);
      debugV("Adding byte %s [%x]", byteStr, byteValue);
    } else {
      debugV("Dropping byte [%02X] at index %d", byteStr[0], i);
    }
  }
  return true;
}

/**
 * @brief builds an ActiSense ASCII string into the buffer from the N2KMSG
 *
 * @param N2kMsg    the message to convert
 * @param buffer    the buffer to receive the conversion
 * @param bufsize   the maximum size of the buffer
 * @return u_int16_t  the lenth of the string (zero terminated)
 */
u_int16_t tActisenseASCIIReader::buildMessage (const tN2kMsg &N2kMsg, char *buffer, int bufsize) {
   int i = 0; int j = 0;

  //A000000.000 02FF2 1F801 E1CBD61EBCE34A02
  //0123467890123456789012345678901234567890
  //0        1         2         3
  //            ^ ^ ^ ^     ^ 
   buffer[0] = Escape;
   strncpy (buffer + 1,"000000.000 ",11);
   debugV ("messageBuilder time: %s",buffer);
   sprintf (buffer + 12,"%02X",N2kMsg.Source & 0xff);
   sprintf (buffer + 14,"%02X",N2kMsg.Destination & 0xff);
   sprintf (buffer + 16,"%1X ",N2kMsg.Priority & 0xff);
   debugV ("messageBuilder address: %s",buffer);
   sprintf (buffer + 18,"%05X ",N2kMsg.PGN & 0x1ffff);
   debugV ("messageBuilder PGN: %s",buffer);
 
   for (i = 24;j < N2kMsg.DataLen;i+=2) {
      if (i < bufsize) {
        sprintf (buffer + i,"%02X",N2kMsg.GetByte(j) & 0xff);
      } else {
        debugE ("Writing to exhausted ActisenseBuffer! PGN = %d", N2kMsg.PGN);
      }
   }

   buffer[i] = '\0';
   debugD ("message built: %s, l=%d , i=%d",buffer, strlen(buffer),i);
 
   return i;
  
}
