
// ESF_Client_Example_JOB_IMOVDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "ESF_Client_Example_JOB_IMOV.h"
#include "ESF_Client_Example_JOB_IMOVDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CESF_Client_Example_JOB_IMOVDlg 대화 상자



CESF_Client_Example_JOB_IMOVDlg::CESF_Client_Example_JOB_IMOVDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CESF_Client_Example_JOB_IMOVDlg::IDD, pParent)
{
	m_dxsock = NULL;
	m_sendBuf = NULL;
	m_receiveBuf = NULL;
	m_bCmdActive = FALSE;
	m_bAnswerRecv = FALSE;
	m_iSendLength = 0;
	m_RecvTimeout = 5000;
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_sendBuf = new char[MAX_TEXT_LENGTH];
	m_receiveBuf = new char[MAX_TEXT_LENGTH];
}

void CESF_Client_Example_JOB_IMOVDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_listCommState);
	DDX_Control(pDX, IDC_EDIT_IPADDRESS, m_edtServerIP);
	DDX_Control(pDX, IDC_CMB_JOBLIST, m_cmbJOBList);
	DDX_Control(pDX, IDC_TXT_CURX, m_txtCurrentX);
	DDX_Control(pDX, IDC_TXT_CURY, m_txtCurrentY);
	DDX_Control(pDX, IDC_TXT_CURZ, m_txtCurrentZ);
	DDX_Control(pDX, IDC_TXT_CURRX, m_txtCurrentRx);
	DDX_Control(pDX, IDC_TXT_CURRY, m_txtCurrentRy);
	DDX_Control(pDX, IDC_TXT_CURRZ, m_txtCurrentRz);
	DDX_Control(pDX, IDC_EDIT_MOVX, m_edtMoveX);
	DDX_Control(pDX, IDC_EDIT_MOVY, m_edtMoveY);
	DDX_Control(pDX, IDC_EDIT_MOVZ, m_edtMoveZ);
	DDX_Control(pDX, IDC_EDIT_MOVRX, m_edtMoveRx);
	DDX_Control(pDX, IDC_EDIT_MOVRY, m_edtMoveRy);
	DDX_Control(pDX, IDC_EDIT_MOVRZ, m_edtMoveRz);
	DDX_Control(pDX, IDC_TXT_PORTNUM, m_txtPortNum);
}

BEGIN_MESSAGE_MAP(CESF_Client_Example_JOB_IMOVDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BTN_CLEAR, &CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnClear)
	ON_BN_CLICKED(IDC_BTN_JOBLIST, &CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnJoblist)
	ON_BN_CLICKED(IDC_BTN_JOBRUN, &CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnJobrun)
	ON_BN_CLICKED(IDC_BTN_IMOVE, &CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnImove)
	ON_BN_CLICKED(IDC_BTN_GETCURPOS, &CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnGetcurpos)
	// for IPC with WM_COPYDATA (ksh add)
	ON_MESSAGE(WM_COPYDATA, &CESF_Client_Example_JOB_IMOVDlg::OnTraceMsg)
END_MESSAGE_MAP()



// for IPC with WM_COPYDATA (ksh add)

//struct MOVE_POS
//{
//	char X[256];
//	char Y[256];
//	char Z[256];
//	char Rx[256];
//	char Ry[256];
//	char Rz[256];
//};

struct MOVE_POS
{
	double X;
	double Y;
	double Z;
	double Rx;
	double Ry;
	double Rz;
};

struct CUR_POS
{
	// 임시 값
	double X;
	double Y;
	double Z;
	double Rx ;
	double Ry ;
	double Rz ;

};				 

struct CUR_JOINT
{
	// 임시 값
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;

};

struct MY_STRUCT
{
	int Number;

	char Message[256];
};

LRESULT CESF_Client_Example_JOB_IMOVDlg::OnTraceMsg(WPARAM wParam, LPARAM lParam)
{
	
	//char aa[256];
	//MY_STRUCT myStruct;
	COPYDATASTRUCT* pcds = (COPYDATASTRUCT*)lParam;
	//memcpy_s(&aa, sizeof(aa), pcds->lpData, pcds->cbData);
	//memcpy_s(&myStruct, sizeof(myStruct), pcds->lpData, pcds->cbData);
	
	// pcds->dwData가 COPYDATASTRUCT의 identifier 
	// pcds->dwData == 1 이면 값 읽어서 move, pcds->dwData == 2 이면 로봇으로부터 current pos 받아서 srLib에 넘기는 식으로 가능할 듯
	// 1, 2 대신 특정 변수로 Header 등에 define 시켜서 사용해도 되고 e.g. #define MOVE = 1

	// srLib으로부터 Move Position 받아서 로봇에 전달 (지금은 연결 안되어있어서 실험 못함)
	if (pcds->dwData == CONNECTION_START_SIGNAL)
	{
		////////////////////// initializer (필요한건지 나중에 확인) //////////////////////
		if (m_dxsock)
		{
			m_dxsock->bNoReceive = TRUE;
			m_bCmdActive = FALSE;
			Sleep(600);
			delete m_dxsock;
			m_dxsock = NULL;
			m_sendBuf = NULL;
			m_receiveBuf = NULL;
			m_bAnswerRecv = FALSE;
			m_iSendLength = 0;
			m_RecvTimeout = 5000;
			m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
			m_sendBuf = new char[MAX_TEXT_LENGTH];
			m_receiveBuf = new char[MAX_TEXT_LENGTH];
			delete m_sendThread;	// disconnect에서 delete 안하는데 connect함수에서 생성했었음.
			m_bIsConnect = FALSE;
		}
		/////////////////////////////////////////////////////////////////////////////////


		AddStrToList(_T("startConnection() from srLib reached!"));
		BOOL bRet;
		INT PosValLeng = 0;
		CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
		CString strSendData = _T("\r");

		m_edtServerIP.GetWindowTextW(strIP);
		m_txtPortNum.GetWindowTextW(strPortNum);
		bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

		if (bRet != TRUE)
		{
			Disconnect();
			AddStrToList(_T("Connect Failed!"));
			return FALSE;
		}

		AddStrToList(_T("Connect Success!"));
		// When using multiple commands in a row 
		// Here, -1 means infinite number of commands. If the number is specified, change -1 to corresponding number.
		// Check enthernet server function manual 14 page for more details.
		bRet = MsgSend(_T("CONNECT Robot_access Keep-Alive:-1\r\n"), TRUE);
		// turn off servo if turned on
		ServoControlFunc(TRUE);
	}
	if (pcds->dwData == CONNECTION_END_SIGNAL)
	{
		ServoControlFunc(FALSE);
		AddStrToList(_T("endConnection() from srLib reached!"));
		Disconnect();
	}
	if (pcds->dwData == MOVE_SIGNAL)
	{

		AddStrToList(_T("goToWaypoint() from srLib reached!"));
		MOVE_POS move_pos_from_srLib;
		memcpy_s(&move_pos_from_srLib, sizeof(move_pos_from_srLib), pcds->lpData, pcds->cbData);
		std::cout << "Do robot move" << std::endl;
		m_MoveVal[0] = move_pos_from_srLib.X;
		m_MoveVal[1] = move_pos_from_srLib.Y;
		m_MoveVal[2] = move_pos_from_srLib.Z;
		m_MoveVal[3] = move_pos_from_srLib.Rx;
		m_MoveVal[4] = move_pos_from_srLib.Ry;
		m_MoveVal[5] = move_pos_from_srLib.Rz;
#ifdef USE_DISCONNECT
		ServoControl(TRUE);
		IncrementalMove();
		ServoControl(FALSE);
#endif
#ifndef USE_DISCONNECT
#ifndef CONTINUE_SERVOING
		ServoControlFunc(TRUE);
#endif
		IncrementalMoveFunc();
#ifndef CONTINUE_SERVOING
		ServoControlFunc(FALSE);
#endif
#endif
	}
	if (pcds->dwData == GET_CURPOS_SIGNAL)
	{
		AddStrToList(_T("GetCurPosSignal() from srLib reached!"));
		BOOL bRet;
		INT PosValLeng = 0;
		CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
		CString strSendData = _T("\r");
#ifdef USE_DISCONNECT
		m_edtServerIP.GetWindowTextW(strIP);
		m_txtPortNum.GetWindowTextW(strPortNum);
		bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

		if (bRet != TRUE)
		{
			Disconnect();
			AddStrToList(_T("Connect Failed!"));
			return FALSE;
		}

		AddStrToList(_T("Connect Success!"));

		bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

		if (bRet != TRUE)
		{
			Disconnect();
			AddStrToList(_T("CONNECT Robot_access Failed!"));
			return FALSE;
		}
#endif
		// 수신 메시지 비교(OK여부) 처리 추가

		strSendData.Format(_T("%d,%d\r"), ROBOT_COORD, RPOSC_NO_EX_AXIS);
		strSendCmd.Format(_T("HOSTCTRL_REQUEST RPOSC %d\r\n"), strSendData.GetLength());
		bRet = MsgSend(strSendCmd, TRUE);

		if (bRet != TRUE)
		{
#ifdef USE_DISCONNECT
			Disconnect();
#endif
			AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
			return FALSE;
		}

		// 수신 메시지 비교(OK여부) 처리 추가

		bRet = MsgSend(strSendData, TRUE);

		if (bRet != TRUE)
		{
#ifdef USE_DISCONNECT
			Disconnect();
#endif
			AddStrToList(_T("DataSend Failed!"));
			return FALSE;
		}

		strPosVal = m_strList;

		std::vector<double> tmpCurPos(0);
		for (int i = 0; i < 6; i++)
		{
			PosValLeng = strPosVal.Find(',');

			if (PosValLeng == -1)
				PosValLeng = strPosVal.GetLength();

			strVal = strPosVal.Left(PosValLeng);
			tmpCurPos.push_back(_wtof(strVal));
			GetDlgItem(IDC_TXT_CURX + i)->SetWindowTextW(strVal);
			strPosVal.Delete(0, strVal.GetLength() + 1);
		}
#ifdef USE_DISCONNECT
		Disconnect();
#endif
		CUR_POS cur_pos;
		cur_pos.X = tmpCurPos[0];
		cur_pos.Y = tmpCurPos[1];
		cur_pos.Z = tmpCurPos[2];
		cur_pos.Rx = tmpCurPos[3];
		cur_pos.Ry = tmpCurPos[4];
		cur_pos.Rz = tmpCurPos[5];
		CWnd *hTargetWnd = CWnd::FindWindow(L"srLibServer" , NULL);
		if (hTargetWnd == NULL)
		{
			AddStrToList(_T("Can't find the target window!"));
			return 0;
		}
		if (hTargetWnd) {
			COPYDATASTRUCT cds;
			cds.dwData = 222 /* Flag 용도 */;
			cds.cbData = sizeof(cur_pos);
			cds.lpData = &cur_pos;
			hTargetWnd->SendMessage(WM_COPYDATA, (WPARAM)AfxGetApp()->m_pMainWnd->GetSafeHwnd(),(LPARAM)&cds);
		}
	}

	if (pcds->dwData == GRIPPER_ON_SIGNAL)
	{
		AddStrToList(_T("GripperOnSignal() from srLib reached!"));
		// ExcuteJOBName에 GRIPPER ON 관련 스트링 넣기
		CString ExcuteJOBName = _T("GRIPPER-ON");

		//m_cmbJOBList.GetLBText(m_cmbJOBList.GetCurSel(), ExcuteJOBName);
#ifdef USE_DISCONNECT
		ServoControl(TRUE);
		JOBExcute(ExcuteJOBName);
#endif
#ifndef USE_DISCONNECT
		ServoControlFunc(TRUE);
		JOBExcuteFunc(ExcuteJOBName);
#endif // !1

	}

	if (pcds->dwData == GRIPPER_OFF_SIGNAL)
	{
		AddStrToList(_T("GripperOffSignal() from srLib reached!"));
		// ExcuteJOBName에 GRIPPER OFF 관련 스트링 넣기
		CString ExcuteJOBName = _T("GRIPPER-OFF");

		//m_cmbJOBList.GetLBText(m_cmbJOBList.GetCurSel(), ExcuteJOBName);
#ifdef USE_DISCONNECT
		ServoControl(TRUE);
		JOBExcute(ExcuteJOBName);
#endif
#ifndef USE_DISCONNECT
		ServoControlFunc(TRUE);
		JOBExcuteFunc(ExcuteJOBName);
#endif
	}

	if (pcds->dwData == GET_CURJOINT_SIGNAL)
	{
		AddStrToList(_T("GetCurJointSignal() from srLib reached!"));
		BOOL bRet;
		INT PosValLeng = 0;
		CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
		CString strSendData = _T("\r");
#ifdef USE_DISCONNECT
		m_edtServerIP.GetWindowTextW(strIP);
		m_txtPortNum.GetWindowTextW(strPortNum);
		bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

		if (bRet != TRUE)
		{
			Disconnect();
			AddStrToList(_T("Connect Failed!"));
			return FALSE;
		}

		AddStrToList(_T("Connect Success!"));

		bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

		if (bRet != TRUE)
		{
			Disconnect();
			AddStrToList(_T("CONNECT Robot_access Failed!"));
			return FALSE;
		}
#endif
		// 수신 메시지 비교(OK여부) 처리 추가
		
		strSendCmd.Format(_T("HOSTCTRL_REQUEST RPOSJ 0\r\n"));
		bRet = MsgSend(strSendCmd, TRUE);

		if (bRet != TRUE)
		{
#ifdef USE_DISCONNECT
			Disconnect();
#endif
			AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
			return FALSE;
		}

		bRet = MsgSend(strSendData, TRUE);

		strPosVal = m_strList;

		std::vector<int> tmpCurJoint(0);
		for (int i = 0; i < 6; i++)
		{
			PosValLeng = strPosVal.Find(',');

			if (PosValLeng == -1)
				PosValLeng = strPosVal.GetLength();

			strVal = strPosVal.Left(PosValLeng);
			tmpCurJoint.push_back(_wtoi(strVal));
			GetDlgItem(IDC_TXT_CURX + i)->SetWindowTextW(strVal);
			strPosVal.Delete(0, strVal.GetLength() + 1);
		}
#ifdef USE_DISCONNECT
		Disconnect();
#endif
		CUR_JOINT cur_joint;
		
		cur_joint.q1 = (double) tmpCurJoint[0] / PULSE_PER_DEG;
		cur_joint.q2 = (double) tmpCurJoint[1] / PULSE_PER_DEG;
		cur_joint.q3 = (double) tmpCurJoint[2] / PULSE_PER_DEG;
		cur_joint.q4 = (double) tmpCurJoint[3] / PULSE_PER_DEG;
		cur_joint.q5 = (double) tmpCurJoint[4] / PULSE_PER_DEG;
		cur_joint.q6 = (double) tmpCurJoint[5] / PULSE_PER_DEG;
		CWnd *hTargetWnd = CWnd::FindWindow(L"srLibServer", NULL);
		if (hTargetWnd == NULL)
		{
			AddStrToList(_T("Can't find the target window!"));
			return 0;
		}
		if (hTargetWnd) {
			COPYDATASTRUCT cds;
			cds.dwData = 223 /* Flag 용도 */;
			cds.cbData = sizeof(cur_joint);
			cds.lpData = &cur_joint;
			hTargetWnd->SendMessage(WM_COPYDATA, (WPARAM)AfxGetApp()->m_pMainWnd->GetSafeHwnd(), (LPARAM)&cds);
		}
	}

	return 0;
	
}

// CESF_Client_Example_JOB_IMOVDlg 메시지 처리기

BOOL CESF_Client_Example_JOB_IMOVDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	// 프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	m_edtServerIP.SetWindowTextW(_T("192.168.255.1"));
	m_edtMoveX.SetWindowTextW(_T("0.0"));
	m_edtMoveY.SetWindowTextW(_T("0.0"));
	m_edtMoveZ.SetWindowTextW(_T("0.0"));
	m_edtMoveRx.SetWindowTextW(_T("0.0"));
	m_edtMoveRy.SetWindowTextW(_T("0.0"));
	m_edtMoveRz.SetWindowTextW(_T("0.0"));

	UpdateData(TRUE);
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CESF_Client_Example_JOB_IMOVDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CESF_Client_Example_JOB_IMOVDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CESF_Client_Example_JOB_IMOVDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CESF_Client_Example_JOB_IMOVDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.

	Sleep(1000);

	if (m_dxsock)
	{
		m_dxsock->bNoReceive = TRUE;
		m_bCmdActive = FALSE;
		Sleep(600);
		delete m_dxsock;
		m_dxsock = NULL;
	}
}


void CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnClear()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_listCommState.ResetContent();
}

UINT SendThread(LPVOID pParam)
{
	CESF_Client_Example_JOB_IMOVDlg* pThis = (CESF_Client_Example_JOB_IMOVDlg*)pParam;

	while (true)
	{
		if (pThis->m_bCmdActive)
		{
			pThis->m_bCmdActive = FALSE;
			pThis->m_dxsock->Send(pThis->m_sendBuf, pThis->m_iSendLength);
		}

		//This thread does not need immediate response time
		//so give the CPU a break
		Sleep(30);
	}

	return 0;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::Connect(INT nType, CString sIP, INT nPort)
{
	m_dxsock = new DxSock(this);

	m_bIsConnect = m_dxsock->Create(nType);

	m_bIsConnect = m_bIsConnect && m_dxsock->Connect(sIP, nPort);

	if (!m_bIsConnect)
	{
		delete m_dxsock;
		m_dxsock = NULL;
		AddStrToList(_T("Could not connect... Please check your connection setting.\r\n"));
		return FALSE;
	}

	AddStrToList(_T("Connected"));
	m_sendThread = AfxBeginThread(::SendThread, this);
	return TRUE;
}

VOID CESF_Client_Example_JOB_IMOVDlg::Disconnect()
{
	AddStrToList(_T("Disconnecting..."));

	if (m_dxsock)
	{
		m_dxsock->bNoReceive = TRUE;
		m_bCmdActive = FALSE;
		Sleep(600);
		delete m_dxsock;
		m_dxsock = NULL;
	}

	AddStrToList(_T("Not Connected"));
}

BOOL CESF_Client_Example_JOB_IMOVDlg::MsgSend(CString SendStr, BOOL IsWait)
{
	CStringA strSendMsg;
	CHAR SendMsg[MAX_TEXT_LENGTH];

	strSendMsg = SendStr;

	strcpy_s(SendMsg, strSendMsg);
	memcpy(m_sendBuf, SendMsg, sizeof(CHAR)* MAX_TEXT_LENGTH);
	m_iSendLength = strSendMsg.GetLength();
	m_bCmdActive = TRUE;

	if (IsWait == FALSE)
	{
		m_bAnswerRecv = FALSE;
		return TRUE;
	}
	
	do{
		MSG msg;
		DWORD dwStart = GetTickCount();

		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		if ((GetTickCount() - dwStart) >= m_RecvTimeout)
		{
			return FALSE;
		}
	}while (!m_bAnswerRecv);

	m_bAnswerRecv = FALSE;

	return TRUE;
}

VOID CESF_Client_Example_JOB_IMOVDlg::UpdateRcvMsg(BOOL bException)
{
	CString strSendMsg;
	CStringA strSendMsgTemp;

	if (bException == TRUE)
	{
		m_strList.Format(_T("Server Disconnect"));
		Disconnect();
	}
	else
	{
		m_strList = m_receiveBuf;
		AddStrToList(m_strList);
	}
}
BOOL CESF_Client_Example_JOB_IMOVDlg::PreTranslateMessage(MSG* pMsg)
{

	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_RETURN)
			return TRUE;
		if (pMsg->wParam == VK_ESCAPE)
			return TRUE;
	}

	return CDialogEx::PreTranslateMessage(pMsg);
}

VOID CESF_Client_Example_JOB_IMOVDlg::AddStrToList(CString AddStr)
{
	m_strList.Format(AddStr);
	m_listCommState.SetCurSel(m_listCommState.AddString(m_strList));
}

void CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnJoblist()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ReadAllJobNames();
}

void CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnJobrun()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString ExcuteJOBName;

	m_cmbJOBList.GetLBText(m_cmbJOBList.GetCurSel(), ExcuteJOBName);
	ServoControl(TRUE);
	JOBExcute(ExcuteJOBName);
	//ServoControl(FALSE);
}

void CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnGetcurpos()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CurPosRead();
}

void CESF_Client_Example_JOB_IMOVDlg::OnBnClickedBtnImove()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strVal;
	
	for (int i = 0; i < 6; i++)
	{
		GetDlgItem(IDC_EDIT_MOVX + i)->GetWindowTextW(strVal);
		m_MoveVal[i] = _ttof(strVal);
	}

	ServoControl(TRUE);
	IncrementalMove();
	ServoControl(FALSE);
}

BOOL CESF_Client_Example_JOB_IMOVDlg::ReadAllJobNames()
{
	BOOL bRet;
	INT JOBCnt = 0, JOBNameLeng = 0, FirtLineJOBLeng = 0;
	CString strIP, strPortNum, strSendCmd, strTotalJOB, strJOBName, strFirstJOBS;
	CString strSendData = _T("*\r");

	m_edtServerIP.GetWindowTextW(strIP);
	m_txtPortNum.GetWindowTextW(strPortNum);
	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("Connect Failed!"));
		return FALSE;
	}

	AddStrToList(_T("Connect Success!"));

	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("CONNECT Robot_access Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가 

	strSendCmd.Format(_T("HOSTCTRL_REQUEST RJDIR %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가 

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	JOBNameLeng = 0;
	m_cmbJOBList.ResetContent();
	strTotalJOB = m_strList;

	FirtLineJOBLeng = strTotalJOB.Find('\r');
	strFirstJOBS = strTotalJOB.Left(FirtLineJOBLeng);

	while (strFirstJOBS.GetLength())
	{
		JOBNameLeng = strFirstJOBS.Find(',');
		if (JOBNameLeng == -1)
			JOBNameLeng = strFirstJOBS.GetLength();
		strJOBName = strFirstJOBS.Left(JOBNameLeng);
		m_cmbJOBList.AddString(strJOBName);
		strFirstJOBS.Delete(0, strJOBName.GetLength() + 1);
	}

	m_cmbJOBList.SetCurSel(0);

	Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::ServoControl(BOOL IsOn)
{
	BOOL bRet;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("0\r");
	
	m_edtServerIP.GetWindowTextW(strIP);
	m_txtPortNum.GetWindowTextW(strPortNum);
	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

	if (bRet != TRUE)
	{
	Disconnect();
	AddStrToList(_T("Connect Failed!"));
	return FALSE;
	}

	AddStrToList(_T("Connect Success!"));

	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

	if (bRet != TRUE)
	{
	Disconnect();
	AddStrToList(_T("CONNECT Robot_access Failed!"));
	return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d\r"), IsOn);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST SVON %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
	Disconnect();
	AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
	return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
	Disconnect();
	AddStrToList(_T("DataSend Failed!"));
	return FALSE;
	}
	
	Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::JOBExcute(CString strJOBName)
{
	BOOL bRet;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("\r");

	m_edtServerIP.GetWindowTextW(strIP);
	m_txtPortNum.GetWindowTextW(strPortNum);
	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("Connect Failed!"));
		return FALSE;
	}

	AddStrToList(_T("Connect Success!"));

	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("CONNECT Robot_access Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%s\r"),strJOBName);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST START %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::CurPosRead()
{
	BOOL bRet;
	INT PosValLeng = 0;
	CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
	CString strSendData = _T("\r");

	m_edtServerIP.GetWindowTextW(strIP);
	m_txtPortNum.GetWindowTextW(strPortNum);
	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("Connect Failed!"));
		return FALSE;
	}

	AddStrToList(_T("Connect Success!"));

	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("CONNECT Robot_access Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d,%d\r"), ROBOT_COORD, RPOSC_NO_EX_AXIS);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST RPOSC %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	strPosVal = m_strList;

	for (int i = 0; i < 6; i++)
	{
		PosValLeng = strPosVal.Find(',');

		if (PosValLeng == -1)
			PosValLeng = strPosVal.GetLength();

		strVal = strPosVal.Left(PosValLeng);
		GetDlgItem(IDC_TXT_CURX + i)->SetWindowTextW(strVal);
		strPosVal.Delete(0, strVal.GetLength() + 1);
	}

	Disconnect();

	return TRUE;
}
//
//BOOL CESF_Client_Example_JOB_IMOVDlg::IncrementalMove()
//{
//	BOOL bRet;
//	FLOAT fMoveSpd = IMOV_SPEED;
//	CString strIP, strPortNum, strSendCmd;
//	CString strSendData = _T("\r");
//
//	m_edtServerIP.GetWindowTextW(strIP);
//	m_txtPortNum.GetWindowTextW(strPortNum);
//	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));
//
//	if (bRet != TRUE)
//	{
//		Disconnect();
//		AddStrToList(_T("Connect Failed!"));
//		return FALSE;
//	}
//
//	AddStrToList(_T("Connect Success!"));
//
//	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);
//
//	if (bRet != TRUE)
//	{
//		Disconnect();
//		AddStrToList(_T("CONNECT Robot_access Failed!"));
//		return FALSE;
//	}
//
//	// 수신 메시지 비교(OK여부) 처리 추가
//
//	// get initial pos
//	std::vector<double> initPos = GetCurPos();
//	SE3 Tinit = EulerXYZ(Vec3(DEG2RAD(initPos[3]), DEG2RAD(initPos[4]), DEG2RAD(initPos[5])), 0.001*Vec3(initPos[0], initPos[1], initPos[2]));
//	SE3 Timov = EulerXYZ(Vec3(DEG2RAD(m_MoveVal[3]), DEG2RAD(m_MoveVal[4]), DEG2RAD(m_MoveVal[5])), 0.001*Vec3(m_MoveVal[0], m_MoveVal[1], m_MoveVal[2]));
//	SE3 Tgoal = Tinit * Timov;
//	strSendData.Format(_T("%d,%.1f,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d\r")
//		, 0, fMoveSpd, ROBOT_COORD, m_MoveVal[0], m_MoveVal[1], m_MoveVal[2], m_MoveVal[3], m_MoveVal[4], m_MoveVal[5], 0, 20, 0, 0, 0, 0, 0, 0);
//
//	strSendCmd.Format(_T("HOSTCTRL_REQUEST IMOV %d\r\n"), strSendData.GetLength());
//	bRet = MsgSend(strSendCmd, TRUE);
//
//	if (bRet != TRUE)
//	{
//		Disconnect();
//		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
//		return FALSE;
//	}
//
//	// 수신 메시지 비교(OK여부) 처리 추가
//
//	bRet = MsgSend(strSendData, TRUE);
//
//	if (bRet != TRUE)
//	{
//		Disconnect();
//		AddStrToList(_T("DataSend Failed!"));
//		return FALSE;
//	}
//
//	while (1)
//	{
//		std::vector<double> tmpCurPos = GetCurPos();
//		SE3 Tcur = EulerXYZ(Vec3(DEG2RAD(tmpCurPos[3]), DEG2RAD(tmpCurPos[4]), DEG2RAD(tmpCurPos[5])), 0.001*Vec3(tmpCurPos[0], tmpCurPos[1], tmpCurPos[2]));
//		double distance = distSE3(Tcur, Tgoal);
//		if (distance < EPS__)
//		{
//			break;
//		}
//			
//	}
//	//Sleep(5000);
//
//
//
//	Disconnect();
//
//	return TRUE;
//}

BOOL CESF_Client_Example_JOB_IMOVDlg::IncrementalMove()
{
	BOOL bRet;
	FLOAT fMoveSpd = IMOV_SPEED;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("\r");

	m_edtServerIP.GetWindowTextW(strIP);
	m_txtPortNum.GetWindowTextW(strPortNum);
	bRet = Connect(SOCK_STREAM, strIP, StrToInt(strPortNum));

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("Connect Failed!"));
		return FALSE;
	}

	AddStrToList(_T("Connect Success!"));

	bRet = MsgSend(_T("CONNECT Robot_access\r\n"), TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("CONNECT Robot_access Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d,%.1f,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d\r")
		, 0, fMoveSpd, ROBOT_COORD, m_MoveVal[0], m_MoveVal[1], m_MoveVal[2], m_MoveVal[3], m_MoveVal[4], m_MoveVal[5], 0, 20, 0, 0, 0, 0, 0, 0);

	strSendCmd.Format(_T("HOSTCTRL_REQUEST IMOV %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	//Sleep(20000);
	int sleepTime;
	sleepTime = max(sqrt(m_MoveVal[0]* m_MoveVal[0] + m_MoveVal[1]*m_MoveVal[1] + m_MoveVal[2]* m_MoveVal[2]), sqrt(m_MoveVal[3]* m_MoveVal[3] + m_MoveVal[4] * m_MoveVal[4] + m_MoveVal[5] * m_MoveVal[5])) / IMOV_SPEED;
	Sleep((sleepTime + 1)*1000);
	Disconnect();
	return TRUE;
}
std::vector<double> CESF_Client_Example_JOB_IMOVDlg::GetCurPos()
{
	BOOL bRet;
	INT PosValLeng = 0;
	CString strSendCmd2, strPosVal, strVal;
	CString strSendData2 = _T("\r");

	strSendData2.Format(_T("%d,%d\r"), ROBOT_COORD, RPOSC_NO_EX_AXIS);
	strSendCmd2.Format(_T("HOSTCTRL_REQUEST RPOSC %d\r\n"), strSendData2.GetLength());
	bRet = MsgSend(strSendCmd2, TRUE);
	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
	}
	bRet = MsgSend(strSendData2, TRUE);
	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("DataSend Failed!"));
	}

	strPosVal = m_strList;
	std::vector<double> tmpCurPos(0);
	for (int i = 0; i < 6; i++)
	{
		PosValLeng = strPosVal.Find(',');

		if (PosValLeng == -1)
			PosValLeng = strPosVal.GetLength();

		strVal = strPosVal.Left(PosValLeng);
		tmpCurPos.push_back(_wtof(strVal));
		strPosVal.Delete(0, strVal.GetLength() + 1);
	}
	return tmpCurPos;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::ServoControlFunc(BOOL IsOn)
{
	BOOL bRet;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("0\r");

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d\r"), IsOn);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST SVON %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	//Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::JOBExcuteFunc(CString strJOBName)
{
	BOOL bRet;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("\r");

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%s\r"), strJOBName);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST START %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	//Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::CurPosReadFunc()
{
	BOOL bRet;
	INT PosValLeng = 0;
	CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
	CString strSendData = _T("\r");

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d,%d\r"), ROBOT_COORD, RPOSC_NO_EX_AXIS);
	strSendCmd.Format(_T("HOSTCTRL_REQUEST RPOSC %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	strPosVal = m_strList;

	for (int i = 0; i < 6; i++)
	{
		PosValLeng = strPosVal.Find(',');

		if (PosValLeng == -1)
			PosValLeng = strPosVal.GetLength();

		strVal = strPosVal.Left(PosValLeng);
		GetDlgItem(IDC_TXT_CURX + i)->SetWindowTextW(strVal);
		strPosVal.Delete(0, strVal.GetLength() + 1);
	}

	//Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::CurJointReadFunc()
{
	BOOL bRet;
	INT PosValLeng = 0;
	CString strIP, strPortNum, strSendCmd, strPosVal, strVal;
	CString strSendData = _T("\r");

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendCmd.Format(_T("HOSTCTRL_REQUEST RPOSJ\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}
	
	strPosVal = m_strList;

	for (int i = 0; i < 6; i++)
	{
		PosValLeng = strPosVal.Find(',');

		if (PosValLeng == -1)
			PosValLeng = strPosVal.GetLength();

		strVal = strPosVal.Left(PosValLeng);
		GetDlgItem(IDC_TXT_CURX + i)->SetWindowTextW(strVal);
		strPosVal.Delete(0, strVal.GetLength() + 1);
	}

	//Disconnect();

	return TRUE;
}

BOOL CESF_Client_Example_JOB_IMOVDlg::IncrementalMoveFunc()
{
	BOOL bRet;
	FLOAT fMoveSpd = IMOV_SPEED;
	CString strIP, strPortNum, strSendCmd;
	CString strSendData = _T("\r");

	// 수신 메시지 비교(OK여부) 처리 추가

	strSendData.Format(_T("%d,%.1f,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d\r")
		, 0, fMoveSpd, ROBOT_COORD, m_MoveVal[0], m_MoveVal[1], m_MoveVal[2], m_MoveVal[3], m_MoveVal[4], m_MoveVal[5], 0, 20, 0, 0, 0, 0, 0, 0);

	strSendCmd.Format(_T("HOSTCTRL_REQUEST IMOV %d\r\n"), strSendData.GetLength());
	bRet = MsgSend(strSendCmd, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("HOSTCTRL_REQUEST Failed!"));
		return FALSE;
	}

	// 수신 메시지 비교(OK여부) 처리 추가

	bRet = MsgSend(strSendData, TRUE);

	if (bRet != TRUE)
	{
		//Disconnect();
		AddStrToList(_T("DataSend Failed!"));
		return FALSE;
	}

	//Sleep(20000);
	int sleepTime;
	sleepTime = max(sqrt(m_MoveVal[0] * m_MoveVal[0] + m_MoveVal[1] * m_MoveVal[1] + m_MoveVal[2] * m_MoveVal[2]), sqrt(m_MoveVal[3] * m_MoveVal[3] + m_MoveVal[4] * m_MoveVal[4] + m_MoveVal[5] * m_MoveVal[5])) / IMOV_SPEED;
	Sleep((sleepTime + 1) * 1000);
	//Disconnect();
	return TRUE;
}
