//DxSock.cpp
//
#include <atlconv.h>
#include "stdafx.h"
#include "DxSock.h"
#include "ESF_Client_Example_JOB_IMOVDlg.h"

DxSock::DxSock(CESF_Client_Example_JOB_IMOVDlg* parent)
{
	pParent = parent;
	bNoReceive = false;
}

DxSock::~DxSock()
{

}

void DxSock::OnReceive()
{
	if (!bNoReceive)
	{

		/*if (pParent->bClkAnnounceTest)
		{
		if (pParent->nClkAnnounceCount < 10)
		{
		pParent->nClkAnnounceCount += 1;
		pParent->timerHighRes.Start();
		}
		else
		{
		pParent->bClkAnnounceTest = false;
		}
		}*/

		CString msg;
		char *buf;
		int bytesRead;
		int len = GetDataSize();
		if (len > 0)
		{
			buf = new char[len + 1];
			bytesRead = Read(buf, len);
			buf[len] = NULL;

			memset(pParent->m_receiveBuf, 0, MAX_TEXT_LENGTH);
			memcpy(pParent->m_receiveBuf, buf, len);
			pParent->UpdateRcvMsg(FALSE);
			pParent->m_bAnswerRecv = TRUE;
			delete[] buf;
		}
		else if (len < 0)
		{
			pParent->UpdateRcvMsg(TRUE);
		}
	}
}