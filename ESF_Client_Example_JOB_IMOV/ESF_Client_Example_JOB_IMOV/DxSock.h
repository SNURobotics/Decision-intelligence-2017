//DxSock.h
//
#ifndef DX_SOCK_H
#define DX_SOCK_H
#pragma once

#include "CESocket.h"

class CESF_Client_Example_JOB_IMOVDlg;

//reusing previously created socket code...
//just quicker and lazier
class DxSock : public CCESocket
{
public:
	DxSock(CESF_Client_Example_JOB_IMOVDlg* parent);
	~DxSock();

	bool bNoReceive;
private:
	virtual void OnReceive();

	CESF_Client_Example_JOB_IMOVDlg* pParent;
};

#endif