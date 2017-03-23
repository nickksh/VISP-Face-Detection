#include "Serial.h"
#include <iostream>

//#include <windows.h>


Serial::Serial(tstring &commPortName, int bitRate)
{

	commHandle = CreateFile(commPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

			if (commHandle == INVALID_HANDLE_VALUE)
			{
				throw("Error communication with comm port");
			}
			else
			{

				COMMTIMEOUTS cto = { MAXDWORD,0,0,0,0 };
				DCB dcb;
						if (!SetCommTimeouts(commHandle,&cto))
						{
							Serial::~Serial();
							throw("Error.. Could not set timeouts");
						}
				memset(&dcb, 0, sizeof(dcb));
				dcb.DCBlength = sizeof(dcb);
				dcb.BaudRate = 9600;
				dcb.fBinary = 1;
				dcb.fRtsControl = RTS_CONTROL_DISABLE;
				dcb.fDtrControl = DTR_CONTROL_DISABLE;

				dcb.Parity = NOPARITY;
				dcb.StopBits = ONESTOPBIT;
				dcb.ByteSize = 8;

						if (!SetCommState(commHandle, &dcb))
						{
							Serial::~Serial();
							throw("Error.. Could not set timeouts");
						}
			}
}

Serial::~Serial()
{
	CloseHandle(commHandle);
}


//int Serial::Write(const char *buffer, int bufflen)
//{
//	DWORD numbytesWritten;
//	bool checker = WriteFile(commHandle, buffer, bufflen, &numbytesWritten, NULL);
//	return numbytesWritten;
//}


bool Serial::Write(const char *buffer, int bufflen)
{
	DWORD numbytesWritten;
	bool checker = WriteFile(commHandle, buffer, bufflen, &numbytesWritten, NULL);
	return checker;
}


int Serial::Read(char *buffer, int bufflen, bool nullTerminate)
{
	DWORD NumBytesToRead;
/*	if (nullTerminate)
	{
		--bufflen;
	}*/
	BOOL returner = ReadFile(commHandle, buffer, bufflen, &NumBytesToRead, NULL);

	for (size_t i = 0; i < NumBytesToRead; i++)
	{
		std::cout << buffer[i];

	}
	if (!returner)
	{
		return 0;
	}
	
	if (nullTerminate)
	{
		buffer[NumBytesToRead] = '\0';
	}

	return NumBytesToRead;

}


void Serial::Flush()
{
	char buffer[32];
	int numBytesRead = Read(buffer, 32, false);
	while (numBytesRead != 0)
	{
		numBytesRead = Read(buffer, 32, false);
	}
}