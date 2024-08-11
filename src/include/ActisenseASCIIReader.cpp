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

    char buffer[1024];
    int index = 0;

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
bool tActisenseASCIIReader::GetMessageFromStream(tN2kMsg &N2kMsg, bool ReadOut) {
  bool result=false;

  if (ReadStream==0) return false;

  ClearBuffer();

  readStringUntil('\n');
  if (MsgBuf[0] == '\0') return result;
  
  debugD("Parsing message [%s] length %d",(unsigned char *)MsgBuf,strlen ((char *)MsgBuf));

  if (MsgBuf[0] != Escape) {
    debugE ("Invalid ActiSense message starter [%x]. Are you sending ASCII?",MsgBuf[0]);
    return result;
  }
  if (strlen ((char *)MsgBuf) < 24) {
    debugE ("Invalid ActiSense message length [%d]. Are you sending ASCII? %x",strlen ((char*)MsgBuf));
    return result;
  }
  String timestamp = String ((char *)MsgBuf + 1,10);
  debugV ("Processing message with timestamp %s",timestamp);
  String address = String ((char *)MsgBuf+12,5);
  debugV ("Address is %s",address.c_str());
  N2kMsg.Source = strtol (address.substring(0,2).c_str(),nullptr,16);
  debugV ("Source is %s",address.substring(0,2).c_str()); 
  N2kMsg.Destination = strtol (address.substring(2,4).c_str(),nullptr,16);
  debugV ("Dest is %s",address.substring(2,4).c_str());
  N2kMsg.Priority = strtol (address.substring(4,5).c_str(),nullptr,16);
  debugV ("Prio is %s",address.substring(4,5).c_str());
  String pgns = String ((char *)MsgBuf+18,5);
  N2kMsg.PGN = strtol (pgns.c_str(),nullptr,16);
  debugD ("PGN is %d [%s]",N2kMsg.PGN,pgns.c_str());
  
  for (int i = 24; i < strlen ((char *)MsgBuf);i+=2) {
      String byte = String ((char *)MsgBuf+i,2);
      byte.trim();
      if (byte.length() == 2) {
        N2kMsg.AddByte ((char)strtol (byte.c_str(),nullptr,16));
        debugV ("Adding byte %s [%x]",byte,(char)strtol (byte.c_str(),nullptr,16));
      } else {
        debugV ("Dropping byte %s at index %d",byte,i);
      }
  }

  result = true;


  return result;
}

//*****************************************************************************
void tActisenseASCIIReader::ParseMessages() {
  tN2kMsg N2kMsg;

    while (GetMessageFromStream(N2kMsg)) {
      if (MsgHandler!=0) MsgHandler(N2kMsg);
    }
}

/**
 * @brief builds an ActiSense ASCII string into the buffer from the N2KMSG
 *
 * @param N2kMsg    the message to convert
 * @param buffer    the buffer to receive the conversion
 * @param bufsize   the maximum size of the buffer
 * @return u_int16_t  the lenth of the string (zero terminated)
 */
u_int16_t tActisenseASCIIReader::buildMessage (const tN2kMsg &N2kMsg, char* buffer, int bufsize) {
   int i = 0; int j = 0;

  
   buffer[0] = Escape;
   strncpy (buffer+1,"000000.000 ",11);
   sprintf (buffer+11,"%02X",(N2kMsg.Source != 0?N2kMsg.Source:DefaultSource) & 0xff);
   sprintf (buffer+13,"%02X",N2kMsg.Destination & 0xff);
   sprintf (buffer+15,"%1X ",N2kMsg.Priority & 0xff);
   sprintf (buffer+17,"%05X ",N2kMsg.PGN & 0x1ffff);
   for (i = 23;j < N2kMsg.DataLen;i+=2) {
      if (i < bufsize) {
        sprintf (buffer+i,"%02X",N2kMsg.GetByte(j) & 0xff);
      }
   }

   buffer[i] = '\0';

   return i;
  
}
