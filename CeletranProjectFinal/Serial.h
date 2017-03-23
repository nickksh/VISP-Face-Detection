#pragma once


#ifndef __SERIAL_H__
#define __SERIAL_H__


#include <string>
#include <windows.h>


typedef std::basic_string<TCHAR> tstring;


class Serial
{
private:
	HANDLE commHandle;

public:
	Serial(tstring &commPortName, int bitRate = 9600);

	bool Write(const char *buffer, int buffLen);

	int Read(char *buffer, int buffLen, bool nullTerminate = true);

	void Flush();

public:
	Serial();
	virtual ~Serial();
};


#endif