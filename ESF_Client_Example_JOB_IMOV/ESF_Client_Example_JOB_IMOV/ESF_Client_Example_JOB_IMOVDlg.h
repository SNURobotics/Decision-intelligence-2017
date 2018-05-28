
// ESF_Client_Example_JOB_IMOVDlg.h : 헤더 파일
//

#pragma once
#include "afxwin.h"
#include "DxSock.h"
#include "Resource.h"

#include <iostream>
#pragma region Includes and Manifest Dependencies
// for communicate
#include <stdio.h>
#include <Windows.h>
#include <windowsx.h>
#include "../LieGroup/LieGroup.h"
#include <string>
#include <vector>
// Enable Visual Style
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#pragma endregion


#define MAX_TEXT_LENGTH		1024

#define BASE_COORD	0
#define ROBOT_COORD	1
#define USER_COORD1	2
#define USER_COORD2	3

#define RPOSC_NO_EX_AXIS	0
#define RPOSC_EX_AXIS		1

#define IMOV_SPEED			20		//500


#define MOVE_SIGNAL 1
#define GET_CURPOS_SIGNAL 2
#define GRIPPER_ON_SIGNAL 3
#define GRIPPER_OFF_SIGNAL 4



enum {X = 0, Y, Z, RX, RY, RZ};

// CESF_Client_Example_JOB_IMOVDlg 대화 상자
class CESF_Client_Example_JOB_IMOVDlg : public CDialogEx
{
// 생성입니다.
public:
	CESF_Client_Example_JOB_IMOVDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
	enum { IDD = IDD_ESF_CLIENT_EXAMPLE_JOB_IMOV_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:

	DxSock*		m_dxsock;
	CWinThread* m_sendThread;
	CHAR*		m_sendBuf;
	CHAR*		m_receiveBuf;

	CListBox m_listCommState;
	CString m_strList;
	CEdit m_edtServerIP;
	CComboBox m_cmbJOBList;
	CStatic m_txtPortNum;
	CStatic m_txtCurrentX;
	CStatic m_txtCurrentY;
	CStatic m_txtCurrentZ;
	CStatic m_txtCurrentRx;
	CStatic m_txtCurrentRy;
	CStatic m_txtCurrentRz;
	CEdit m_edtMoveX;
	CEdit m_edtMoveY;
	CEdit m_edtMoveZ;
	CEdit m_edtMoveRx;
	CEdit m_edtMoveRy;
	CEdit m_edtMoveRz;
	sockaddr_in m_stAddr;

	BOOL m_bIsConnect;
	BOOL m_bCmdActive;
	BOOL m_bAnswerRecv;
	INT m_iSendLength;
	INT m_RecvTimeout;
	FLOAT m_MoveVal[RZ+1];

	BOOL Connect(INT nType, CString sIP, INT nPort);
	VOID Disconnect();
	BOOL MsgSend(CString SendStr, BOOL IsWait);
	VOID UpdateRcvMsg(BOOL bException);
	VOID AddStrToList(CString AddStr);
	BOOL ReadAllJobNames();
	BOOL ServoControl(BOOL IsOn);
	BOOL JOBExcute(CString strJOBName);
	BOOL CurPosRead();
	BOOL RelativePosMove();
	BOOL IncrementalMove();
	std::vector<double> GetCurPos();

	afx_msg void OnDestroy();
	afx_msg void OnBnClickedBtnClear();
	afx_msg void OnBnClickedBtnJoblist();
	afx_msg void OnBnClickedBtnJobrun();
	afx_msg void OnBnClickedBtnImove();
	afx_msg void OnBnClickedBtnGetcurpos();

	virtual BOOL PreTranslateMessage(MSG* pMsg);
	LRESULT OnTraceMsg(WPARAM wParam, LPARAM lParam);
};
