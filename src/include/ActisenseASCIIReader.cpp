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
  byteSum=0;
  StartOfTextReceived=false;
  MsgIsComing=false;
  EscapeReceived=false;
}

//*****************************************************************************
bool tActisenseASCIIReader::AddByteToBuffer(char NewByte) {
  if (MsgWritePos>=MAX_STREAM_MSG_BUF_LEN) return false;

  MsgBuf[MsgWritePos]=NewByte;
  MsgWritePos++;

  return true;
}

#define Escape 'A'
#define StartOfText 'A'
#define EndOfText '\r'
#define MsgTypeN2kData 0x93
#define MsgTypeN2kRequest 0x94

//*****************************************************************************
bool tActisenseASCIIReader::CheckMessage(tN2kMsg &N2kMsg) {

   N2kMsg.Clear();

   int i=2;
   
   N2kMsg.PGN=GetBuf3ByteUInt(i,MsgBuf);
   N2kMsg.Destination=MsgBuf[i++];
   N2kMsg.Priority=MsgBuf[i++];
   if ( MsgBuf[0]==MsgTypeN2kData ) {
     N2kMsg.Source=MsgBuf[i++];
     N2kMsg.MsgTime=GetBuf4ByteUInt(i,MsgBuf);
   } else {
     N2kMsg.Source=DefaultSource;
     N2kMsg.MsgTime=N2kMillis();
   }
   N2kMsg.DataLen=MsgBuf[i++];

   if ( N2kMsg.DataLen>tN2kMsg::MaxDataLen ) {
     N2kMsg.Clear();
     Serial.println ("Msg too long");
     return false; // Too long data
   }

   for (int j=0; i<MsgWritePos-1; i++, j++) N2kMsg.Data[j]=MsgBuf[i];

   return true;
}

//*****************************************************************************
bool tActisenseASCIIReader::IsStart(char ch) {
  return (ch==Escape);
}

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
  
  debugD("Processing %s %d",(unsigned char *)MsgBuf,strlen ((char *)MsgBuf));

  if (MsgBuf[0] != 'A') {
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
  debugV ("PGN is %d [%s]",N2kMsg.PGN,pgns.c_str());
  
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


