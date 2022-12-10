#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "serialLib/serialib.h"
#include <ctype.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#define CVPLOT_HEADER_ONLY
#include "CvPlot/cvplot.h"
#include <X11/Xlib.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <cstdio>

using namespace cv;
using namespace std;
using std::ifstream;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

// #define FIND_X_Y                                         /* To Find X pixel and Y pixel of window                          */
// #define DEMO                                             /* To Look at the demo of the outlook without coonnect to bluebee */

/*************************** CUSTOM AREA ***************************/
#define MAX_VARIABLE_PER_TRACKBAR 12
#define MAX_VARIABLE_PER_BOARD 28
#define MAX_NUMBER_OF_TRACKBAR 3
#define WIDTH_PER_TRACKBAR 320
#define OFFSET_DUE_TO_UBUNTU 0
#define BOARD_INT_WIDTH 200
#define BOARD_INT_HEIGHT 300
#define BOARD_INT_POS_X 900
#define BOARD_INT_POS_Y 0
#define BOARD_FLOAT_WIDTH 200
#define BOARD_FLOAT_HEIGHT 370
#define BOARD_FLOAT_POS_X 1800
#define BOARD_FLOAT_POS_Y 0

#define SAMPLE_TIME 0.02
#define CSV_PATH "./CSV/"
#define JPG_PATH "./jpg/"
/*************************** CUSTOM AREA ***************************/

typedef struct
{
	String varName;
	int current;
	int last;
	int offsetCur;
	int min;
	int max;
} Tune_Int_t;

typedef struct
{
	String varName;
	float currentF;
	float lastF;
	float minF;
	float maxF;
	int current;
	int offsetCur;
	int min;
	int max;
} Tune_Float_t;

vector<Tune_Int_t> listIntTune;
vector<Tune_Float_t> listFloatTune;
vector<String> fileName, imgName;
int imgIndex, fileNum;

#define PRECISION 1000.0
#define HEADER 0x12
#define TAIL 0x21
#define SERIAL_PORT "/dev/ttyUSB0"
#define BUFFER_SIZE 6
uint8_t sendBuf[BUFFER_SIZE];

void initTrackbars(int);
void requestUpdate(int s);
void sendEditedTune(int s);
void EepromWriteRequest(int s);
void findWindowXY(int key, int *X, int *Y);
void getScreenResolution(int &width, int &height);
void showRBCvideo(cv::VideoCapture &waitVideo, int x, int y, double scale, int delay);
void test(void);
void generate(void);
void toggleHelpWindow(int screenWidth, int screenHeight);
void log(int s);
void plotPID(int screenWidth, int screenHeight);
void analyse(void);
void backUp(cv::Mat board, cv::Mat board1, cv::Mat boardF, cv::Mat boardF1);
void clean(void);
int numTrackbar, numBoardInt, toggle;

#ifdef FIND_X_Y
int testX, testY;
#endif

cv::String Page;
Mat writeSuccess, writeFail;
int main(int argc, char **argv)
{
	// if (argc < 2)
	// {
	// 	printf("Wrong argument\n");
	// 	printf("Example usage: ./tune 98:D3:A1:FD:65:C1\n");
	// }
	struct sockaddr_rc addr = {0};
	int s, status;

	char dest[18] = "98:D3:51:FD:E3:FE";
	// memcpy(bluebeeAddress, argv[1], 17);
	// printf("%s\n", bluebeeAddress);

	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = 1;
	str2ba(dest, &addr.rc_bdaddr);

	// connect to server
#ifndef DEMO
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	while (status != 0)
	{
		status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

		cout << "Connecting to bluebee" << endl;
	}
#endif
	std::cout << "bluebee connected" << endl;

	cv::VideoCapture waitVideo;
	while (waitVideo.isOpened() != true)
	{
		waitVideo.open("rbctrim.mp4");
	}

	int screenWidth, screenHeight;
	getScreenResolution(screenWidth, screenHeight);
	printf("Screen resolution: %dx%d\n", screenWidth, screenHeight);

	numTrackbar = 0;

	Mat instruction = Mat(Size(680, 540), CV_8UC3, Scalar(0, 0, 0));
	putText(instruction, "  ************************* KEY FUNCTION *************************", cv::Point(10, 50), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "   0-4     - to request initialization tune variables from mainboard", cv::Point(10, 80), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    r      - to request update of tune variables from mainboard  ", cv::Point(10, 110), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    e     - to send tuned variables to mainboard  ", cv::Point(10, 140), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    w     - to write tune variables to EEPROM on mainboard  ", cv::Point(10, 170), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    s     - to save BoardInt and BoardFloat as image in PC  ", cv::Point(10, 200), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    g     - to generate default value to be saved in header file", cv::Point(10, 230), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    l      - to toggle this help window", cv::Point(10, 260), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    p     - to start logging data for plotting", cv::Point(10, 290), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "  <- -> - to switch between saved plotted graphs", cv::Point(10, 320), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    To tune PID, assign 0  1  2 as P I D of motorA", cv::Point(10, 350), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    in Tune_F_t  assign 3  4  5 as P I D of motorB", cv::Point(10, 380), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "                 assign 6  7  8 as P I D of motorC", cv::Point(10, 410), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "                 assign 9 10 11 as P I D of motorD", cv::Point(10, 440), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    Send the velocities to bluebee in csv format as below:", cv::Point(10, 470), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	putText(instruction, "    TargetVel, motorAvel, motorBvel, motorCvel, motorDvel", cv::Point(10, 500), 0, 0.6, Scalar(0, 255, 0), 1, 8, 0);
	cv::imshow("Instruction", instruction);

	Mat logging = Mat(Size(600, 180), CV_8UC3, Scalar(0, 0, 0));
	putText(logging, "Logging Data", cv::Point(40, 110), 0, 2.5, Scalar(0, 255, 0), 2, 8, 0);
	cv::imshow("Logging", logging);
	cv::moveWindow("Logging", 0, 0);
	cv::destroyWindow("Logging");

	std::cout << endl
			  << endl
			  << "	************************* KEY FUNCTION ************************  " << endl
			  << endl;
	std::cout << "0-4 - to request initialization tune variables from mainboard" << endl;
	std::cout << "	r - to request update of tune variables from mainboard  " << endl;
	std::cout << "	e - to send tuned variables to mainboard  " << endl;
	std::cout << "	w - to write tune variables to EEPROM on mainboard  " << endl;
	std::cout << "	s - to save Board as image in PC  " << endl
			  << endl;

	int first = 1, toggle = 2;

	// system("clear");
	cout << "Width = " << waitVideo.get(cv::CAP_PROP_FRAME_WIDTH) << " Height = " << waitVideo.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
	writeFail = cv::imread("fail.jpg", 1);
	writeSuccess = cv::imread("success.jpg", 1);
	cv::resize(writeFail, writeFail, cv::Size(), 0.25, 0.25, 1);
	cv::resize(writeSuccess, writeSuccess, cv::Size(), 0.25, 0.25, 1);

	while (true)
	{
		Mat boardF, boardF1;
		Mat board, board1;
		if (listFloatTune.size() == 0 && listIntTune.size() == 0)
		{
			cv::moveWindow("Instruction", screenWidth / 2 - 680, screenHeight / 2 - 270);
			showRBCvideo(waitVideo, screenWidth / 2, screenHeight / 2 - waitVideo.get(cv::CAP_PROP_FRAME_HEIGHT) / 2, 1.0, 7);
		}
		else
		{
			if (first)
			{
				// instruction.release();
				cv::destroyWindow("Instruction");
				first = 0;
			}
			showRBCvideo(waitVideo, screenWidth - (waitVideo.get(cv::CAP_PROP_FRAME_WIDTH) * 0.65), screenHeight - (waitVideo.get(cv::CAP_PROP_FRAME_HEIGHT) * 0.65), 0.65, 7);
		}

		int key = cv::waitKey(1);

		if (key == 'r')
			requestUpdate(s);
		if (key == 'e')
			sendEditedTune(s);
		if (key == 'w')
			EepromWriteRequest(s);
#ifdef DEMO
		if (key == 't')
			test();
#endif
		if (key == 'g')
			generate();
		if (key == 'l')
			toggle = !toggle;
		if (toggle == 0)
		{
			imshow("Instruction", instruction);
			cv::moveWindow("Instruction", screenWidth / 2 - 680, screenHeight / 2 - 270);
		}
		else if (toggle == 1)
		{
			cv::destroyWindow("Instruction");
			toggle = 2;
		}
#ifndef DEMO
		if (key == 'p')
		{
			imshow("Logging", logging);
			cv::waitKey(200);
			log(s);
			destroyWindow("Logging");
			plotPID(screenWidth, screenHeight);
		}
#else
		if (key == 'p')
		{
			static int flow = 0;
			if (flow == 0)
			{
				fileName.push_back("PID_M2-D6-H3-m9-s21.csv");
				imgName.push_back("PID_M2-D6-H3-m9-s21.jpg");
				flow = 1;
			}
			else if (flow == 1)
			{
				fileName.push_back("PID_M2-D9-H0-m41-s6.csv");
				imgName.push_back("PID_M2-D9-H0-m41-s6.jpg");
				flow = 2;
			}
			else if (flow == 2)
			{
				fileName.push_back("PID_M2-D19-H12-m45-s23.csv");
				imgName.push_back("PID_M2-D19-H12-m45-s23.jpg");
			}
			plotPID(screenWidth, screenHeight);
		}
#endif

		if (imgName.size() > 0)
		{
			if (key == 83)
			{ //left key
				imgIndex++;
				if (imgIndex > fileNum - 1)
					imgIndex = fileNum - 1;
				else
					destroyWindow(imgName.at(imgIndex - 1));
				cout << "img = " << imgIndex << " total = " << fileNum << endl;
				analyse();
			}
			if (key == 81)
			{ //right key
				imgIndex--;
				if (imgIndex < 0)
					imgIndex = 0;
				else
					destroyWindow(imgName.at(imgIndex + 1));
				cout << "img = " << imgIndex << " total = " << fileNum << endl;
				analyse();
			}
		}
		if (listIntTune.size() > 0)
		{

			if (listIntTune.size() > MAX_VARIABLE_PER_BOARD)
			{
				board = Mat(Size(BOARD_INT_WIDTH, MAX_VARIABLE_PER_BOARD * 21), CV_8UC3, Scalar(0, 0, 0));
				board1 = Mat(Size(BOARD_INT_WIDTH, (listIntTune.size() - MAX_VARIABLE_PER_BOARD) * 21), CV_8UC3, Scalar(0, 0, 0));
			}
			else
			{
				board = Mat(Size(BOARD_INT_WIDTH, listIntTune.size() * 21), CV_8UC3, Scalar(0, 0, 0));
			}
			for (int i = 0; i < listIntTune.size(); i++)
			{
				listIntTune.at(i).current = listIntTune.at(i).offsetCur + listIntTune.at(i).min;
				if (i < MAX_VARIABLE_PER_BOARD)
					putText(board, listIntTune.at(i).varName + string(" = ") + to_string(listIntTune.at(i).current), cv::Point(10, (i + 1) * 20), 0, 0.5, Scalar(0, 255, 0), 1, 8, 0);
				else
					putText(board1, listIntTune.at(i).varName + string(" = ") + to_string(listIntTune.at(i).current), cv::Point(10, (i - MAX_VARIABLE_PER_BOARD + 1) * 20), 0, 0.5, Scalar(0, 255, 0), 1, 8, 0);
			}
			cv::imshow("BoardInt", board);
			cv::moveWindow("BoardInt", numTrackbar * WIDTH_PER_TRACKBAR, BOARD_INT_POS_Y);
			numBoardInt = 1;
			if (!board1.empty())
			{
				numBoardInt = 2;
				cv::imshow("BoardInt1", board1);
				cv::moveWindow("BoardInt1", numTrackbar * WIDTH_PER_TRACKBAR + BOARD_INT_WIDTH, BOARD_INT_POS_Y);
			}
			if (key == 's')
			{
				cv::imwrite("BoardInt" + Page + ".jpg", board);
				std::cout << "BoardInt" + Page + ".jpg" << endl;
				if (!board1.empty())
				{
					cv::imwrite("BoardInt1" + Page + ".jpg", board);
					std::cout << "BoardInt1" + Page + ".jpg" << endl;
				}
			}
		}

		if (listFloatTune.size() > 0)
		{

			if (listFloatTune.size() > MAX_VARIABLE_PER_BOARD)
			{
				boardF = Mat(Size(BOARD_FLOAT_WIDTH, MAX_VARIABLE_PER_BOARD * 21), CV_8UC3, Scalar(0, 0, 0));
				boardF1 = Mat(Size(BOARD_FLOAT_WIDTH, (listFloatTune.size() - MAX_VARIABLE_PER_BOARD) * 21), CV_8UC3, Scalar(0, 0, 0));
			}
			else
			{
				boardF = Mat(Size(BOARD_FLOAT_WIDTH, listFloatTune.size() * 21), CV_8UC3, Scalar(0, 0, 0));
			}
			for (int i = 0; i < listFloatTune.size(); i++)
			{
				listFloatTune.at(i).current = listFloatTune.at(i).offsetCur + listFloatTune.at(i).min;
				listFloatTune.at(i).currentF = listFloatTune.at(i).current / PRECISION;
				if (i < MAX_VARIABLE_PER_BOARD)
					putText(boardF, string(listFloatTune.at(i).varName) + string(" = ") + to_string(listFloatTune.at(i).currentF), cv::Point(10, (i + 1) * 20), 0, 0.5, Scalar(0, 255, 0), 1, 8, 0);
				else
					putText(boardF1, string(listFloatTune.at(i).varName) + string(" = ") + to_string(listFloatTune.at(i).currentF), cv::Point(10, (i - MAX_VARIABLE_PER_BOARD + 1) * 20), 0, 0.5, Scalar(0, 255, 0), 1, 8, 0);
			}
			cv::imshow("BoardFloat", boardF);
			cv::moveWindow("BoardFloat", numTrackbar * WIDTH_PER_TRACKBAR + numBoardInt * BOARD_INT_WIDTH, BOARD_FLOAT_POS_Y);
			if (!boardF1.empty())
			{
				cv::imshow("BoardFloat1", boardF1);
				cv::moveWindow("BoardFloat1", numTrackbar * WIDTH_PER_TRACKBAR + numBoardInt * BOARD_INT_WIDTH + BOARD_FLOAT_WIDTH, BOARD_FLOAT_POS_Y);
			}
			if (key == 's')
			{
				cv::imwrite("BoardF" + Page + ".jpg", boardF);
				std::cout << "BoardF" + Page + ".jpg saved" << endl;
				if (!boardF1.empty())
				{
					cv::imwrite("BoardF1" + Page + ".jpg", boardF);
					std::cout << "BoardF1" + Page + ".jpg saved" << endl;
				}
			}
		}

		if (key == '0')
		{
			backUp(board, board1, boardF, boardF1);
			clean();
			listIntTune.clear();
			listFloatTune.clear();
			numTrackbar = 0;
			numBoardInt = 0;
			char command = '0';
			send(s, &command, 1, 0);
			initTrackbars(s);
			Page = "Page0";
		}
		if (key == '1')
		{
			backUp(board, board1, boardF, boardF1);
			clean();
			listIntTune.clear();
			listFloatTune.clear();
			numTrackbar = 0;
			numBoardInt = 0;
			char command = '1';
			send(s, &command, 1, 0);
			initTrackbars(s);
			Page = "Page1";
		}
		if (key == '2')
		{
			backUp(board, board1, boardF, boardF1);
			clean();
			listIntTune.clear();
			listFloatTune.clear();
			numTrackbar = 0;
			numBoardInt = 0;
			char command = '2';
			send(s, &command, 1, 0);
			initTrackbars(s);
			Page = "Page2";
		}
		if (key == '3')
		{
			backUp(board, board1, boardF, boardF1);
			clean();
			listIntTune.clear();
			listFloatTune.clear();
			numTrackbar = 0;
			numBoardInt = 0;
			char command = '3';
			send(s, &command, 1, 0);
			initTrackbars(s);
			Page = "Page3";
		}
		if (key == '4')
		{
			backUp(board, board1, boardF, boardF1);
			clean();
			listIntTune.clear();
			listFloatTune.clear();
			numTrackbar = 0;
			numBoardInt = 0;
			char command = '4';
			send(s, &command, 1, 0);
			initTrackbars(s);
			Page = "Page4";
		}

		if (key == 'q')
		{
			backUp(board, board1, boardF, boardF1);
			break;
		}
		board.release();
		board1.release();
		boardF.release();
		boardF1.release();
#ifdef FIND_X_Y
		findWindowXY(key, &testX, &testY);
#endif
	}
	close(s);
	waitVideo.release();
	cv::destroyAllWindows();
	return 0;
}

void initTrackbars(int s)
{
	//[NUM_INT_TUNE][NUM_FLOAT_TUNE]
	//[0x52][0x01][strlen][str...][*curValue...][min ...][max ...][0x31][0x40] format of INT
	//[0x14][0x01][strlen][str...][*curValue...][minF...][maxF...][0x20][0x00] format of FLOAT

	uint8_t numInt, numFloat, len;
	uint8_t recvBuf[100], strLen, header, header1;
	recv(s, recvBuf, 1, 0);
	numInt = recvBuf[0];
	recv(s, recvBuf, 1, 0);
	numFloat = recvBuf[0];

	for (uint8_t i = 0; i < numInt; i++)
	{
		recv(s, &header, 1, 0);
		recv(s, &header1, 1, 0);
		if (!(header == 0x52 && header1 == 0x01))
			continue;
		recv(s, &strLen, 1, 0);
		len = strLen + 4 + 4 + 4 + 1 + 1;
		for (int i = 0; i < len; i++)
			recv(s, &recvBuf[i], 1, 0);
		if (recvBuf[len - 2] == 0x31 && recvBuf[len - 1] == 0x40)
		{
			uint8_t varName[strLen];
			Tune_Int_t in;
			memcpy(varName, recvBuf, strLen);
			varName[strLen] = 0;
			in.varName = String((char *)varName);
			cout << in.varName << endl;
			if(!in.varName.empty()){
				in.current = *((int *)&recvBuf[strLen]);
				in.min = *((int *)&recvBuf[strLen + 4]);
				in.max = *((int *)&recvBuf[strLen + 8]);
				in.last = in.current;
				listIntTune.push_back(in);
			}
		}
	}

	for (uint8_t i = 0; i < numFloat; i++)
	{
		recv(s, &header, 1, 0);
		recv(s, &header1, 1, 0);
		if (!(header == 0x14 && header1 == 0x01))
			continue;
		recv(s, &strLen, 1, 0);
		len = strLen + 4 + 4 + 4 + 1 + 1;
		for (int i = 0; i < len; i++)
			recv(s, &recvBuf[i], 1, 0);

		if (recvBuf[len - 2] == 0x20 && recvBuf[len - 1] == 0x00)
		{
			uint8_t varName[strLen];
			Tune_Float_t in;
			memcpy(varName, recvBuf, strLen);
			varName[strLen] = 0;
			in.varName = String((char *)varName);
			if(!in.varName.empty()){
				in.currentF = *((float *)&recvBuf[strLen]);
				in.minF = *((float *)&recvBuf[strLen + 4]);
				in.maxF = *((float *)&recvBuf[strLen + 8]);
				in.lastF = in.currentF;
				listFloatTune.push_back(in);
			}
		}
	}
	int listNum = 0; String TrackbarName;
	for (int i = 0; i < listIntTune.size(); i++)
	{
		listIntTune.at(i).offsetCur = listIntTune.at(i).current - listIntTune.at(i).min;
		int range = listIntTune.at(i).max - listIntTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			cout << "created trackbar" << endl;
			TrackbarName = "TuneInt" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			// if (numTrackbar == 0)
			// 	cv::moveWindow(TrackbarName, 0, 0);
			// else if (numTrackbar == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			// else
			cv::moveWindow(TrackbarName, numTrackbar * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			numTrackbar++;
		}
		createTrackbar(listIntTune.at(i).varName, TrackbarName, &(listIntTune.at(i).offsetCur), range);
	}
	listNum = 0;
	for (int i = 0; i < listFloatTune.size(); i++)
	{
		listFloatTune.at(i).current = listFloatTune.at(i).currentF * PRECISION;
		listFloatTune.at(i).min = listFloatTune.at(i).minF * PRECISION;
		listFloatTune.at(i).max = listFloatTune.at(i).maxF * PRECISION;
		listFloatTune.at(i).offsetCur = listFloatTune.at(i).current - listFloatTune.at(i).min;
		int range = listFloatTune.at(i).max - listFloatTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			TrackbarName = "TuneFloat" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			// if (numTrackbar == 0)
			// 	cv::moveWindow(TrackbarName, 0, 0);
			// else if (numTrackbar == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			// else
			cv::moveWindow(TrackbarName, numTrackbar * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			numTrackbar++;
		}
		printf("fuckup2\n");
		createTrackbar(listFloatTune.at(i).varName, TrackbarName, &(listFloatTune.at(i).offsetCur), range);

		system("clear");
	}
}

void requestUpdate(int s)
{
	//[0x52][0x01][index][*curValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][*curValue...][0x20][0x00] format of FLOAT
	char command = 'r';
	send(s, &command, 1, 0);
	uint8_t recvBuf[7];
	for (int i = 0; i < listIntTune.size(); i++)
	{
		recv(s, recvBuf, 1, 0);
		if (!(recvBuf[0] == 0x52))
			continue;
		recv(s, recvBuf, 1, 0);
		if (!(recvBuf[0] == 0x01))
			continue;
		for (int j = 0; j < 7; j++)
		{
			recv(s, &recvBuf[j], 1, 0);
		}
		if (recvBuf[5] == 0x31 && recvBuf[6] == 0x40)
		{
			listIntTune.at(recvBuf[0]).current = *((int *)&recvBuf[1]);
			listIntTune.at(recvBuf[0]).last = listIntTune.at(recvBuf[0]).current;
			std::cout << listIntTune.at(i).varName << " = " << listIntTune.at(i).current << endl;
		}
	}

	for (int i = 0; i < listFloatTune.size(); i++)
	{
		recv(s, recvBuf, 1, 0);
		if (!(recvBuf[0] == 0x14))
			continue;
		recv(s, recvBuf, 1, 0);
		if (!(recvBuf[0] == 0x01))
			continue;
		for (int j = 0; j < 7; j++)
		{
			recv(s, &recvBuf[j], 1, 0);
		}
		if (recvBuf[5] == 0x20 && recvBuf[6] == 0x00)
		{
			listFloatTune.at(recvBuf[0]).currentF = *((float *)&recvBuf[1]);
			listFloatTune.at(recvBuf[0]).lastF = listFloatTune.at(recvBuf[0]).currentF;
			std::cout << listFloatTune.at(i).varName << " = " << listFloatTune.at(i).currentF << endl;
		}
	}
	if (listIntTune.size() > 0)
		destroyWindow("TuneInt0");
	if (listIntTune.size() > MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt1");
	if (listIntTune.size() > 2*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt2");
	if (listIntTune.size() > 3*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt3");
	if (listFloatTune.size() > 0)
		destroyWindow("TuneFloat0");
	if (listFloatTune.size() > MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat1");
	if (listFloatTune.size() > 2*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat2");
	if (listFloatTune.size() > 3*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat3");

	int trackbarCount = 0;
	int listNum = 0;
	for (int i = 0; i < listIntTune.size(); i++)
	{
		static String TrackbarName;
		listIntTune.at(i).offsetCur = listIntTune.at(i).current - listIntTune.at(i).min;
		int range = listIntTune.at(i).max - listIntTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			cout << "created trackbar" << endl;
			TrackbarName = "TuneInt" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			// if (trackbarCount == 0)
			// 	cv::moveWindow(TrackbarName, 0, 0);
			// else if (trackbarCount == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			// else
			cv::moveWindow(TrackbarName, trackbarCount * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			trackbarCount++;
		}
		createTrackbar(listIntTune.at(i).varName, TrackbarName, &(listIntTune.at(i).offsetCur), range);
	}
	listNum = 0;
	for (int i = 0; i < listFloatTune.size(); i++)
	{
		static String TrackbarName;
		listFloatTune.at(i).current = listFloatTune.at(i).currentF * PRECISION;
		listFloatTune.at(i).min = listFloatTune.at(i).minF * PRECISION;
		listFloatTune.at(i).max = listFloatTune.at(i).maxF * PRECISION;
		listFloatTune.at(i).offsetCur = listFloatTune.at(i).current - listFloatTune.at(i).min;
		int range = listFloatTune.at(i).max - listFloatTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			TrackbarName = "TuneFloat" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			// if (trackbarCount == 0)
			// 	cv::moveWindow(TrackbarName, 0, 0);
			// else if (trackbarCount == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			// else
			cv::moveWindow(TrackbarName, trackbarCount * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			trackbarCount++;
		}
		createTrackbar(listFloatTune.at(i).varName, TrackbarName, &(listFloatTune.at(i).offsetCur), range);
	}
	system("clear");
}

void sendEditedTune(int s)
{
	//[0x52][0x01][index][editValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][editValue...][0x20][0x00] format of FLOAT
	//[0x88][0x77] terminate, back to tune pending
	cv::imshow("WriteResponse", writeSuccess);
	char command = 'e';
	send(s, &command, 1, 0);
	uint8_t sendBuf[9];
	sendBuf[0] = 0x52;
	sendBuf[1] = 0x01;
	sendBuf[7] = 0x31;
	sendBuf[8] = 0x40;
	for (int i = 0; i < listIntTune.size(); i++)
	{
		if (listIntTune.at(i).current != listIntTune.at(i).last)
		{
			sendBuf[2] = i;
			memcpy(&sendBuf[3], &(listIntTune.at(i).current), 4);
			send(s, sendBuf, 9, 0);
		}
	}

	sendBuf[0] = 0x14;
	sendBuf[1] = 0x01;
	sendBuf[7] = 0x20;
	sendBuf[8] = 0x00;
	for (int i = 0; i < listFloatTune.size(); i++)
	{
		if (listFloatTune.at(i).currentF != listFloatTune.at(i).lastF)
		{
			sendBuf[2] = i;
			memcpy(&sendBuf[3], &(listFloatTune.at(i).currentF), 4);
			send(s, sendBuf, 9, 0);
		}
	}

	sendBuf[0] = 0x88;
	sendBuf[1] = 0x77;
	send(s, sendBuf, 2, 0);
	cv::destroyWindow("WriteResponse");
}

void EepromWriteRequest(int s)
{
	static int first = 0;

	if (first)
	{
		cv::destroyWindow("WriteResponse");
	}
	char c = 'w';
	send(s, &c, 1, 0);
	printf("                                                      ");
	char writeResponse[30];
	int index = 0;
	do
	{
		recv(s, &c, 1, 0);
		writeResponse[index++] = c;
		printf("%c", c);
	} while (c != '\n');
	if (strcmp("EEPROM WRITTEN\n", writeResponse) == 0)
	{
		cv::imshow("WriteResponse", writeSuccess);
		cv::waitKey(200);
	}
	else
	{
		cv::imshow("WriteResponse", writeFail);
		cv::waitKey(200);
	}
	first = 1;
}

void findWindowXY(int key, int *X, int *Y)
{
	if (key == 'u')
	{
		*Y -= 5;
		if (*Y < 0)
			*Y = 0;
	}
	if (key == 'j')
		*Y += 20;

	if (key == 'h')
	{
		*X -= 20;
		if (*X < 0)
			*X = 0;
	}

	if (key == 'k')
		*X += 20;
	printf("\t\t\t\t\t\tX = %d Y = %d\n", *X, *Y);
	// cout << "X = " << *X << "  Y = " << *Y << endl;
}

void getScreenResolution(int &width, int &height)
{
	Display *disp = XOpenDisplay(NULL);
	Screen *scrn = DefaultScreenOfDisplay(disp);
	width = scrn->width;
	height = scrn->height;
}

void showRBCvideo(cv::VideoCapture &waitVideo, int x, int y, double scale, int delay)
{
	static int slow = 0, frameCounter = 0;
	if (slow >= delay)
	{
		cv::Mat frame;
		frameCounter++;
		if (frameCounter >= waitVideo.get(cv::CAP_PROP_FRAME_COUNT) - 80)
		{
			frameCounter = 0;
			waitVideo.set(cv::CAP_PROP_POS_FRAMES, 0);
		}
		else
		{
			waitVideo.read(frame);
			cv::resize(frame, frame, cv::Size(), scale, scale, 1);
			cv::imshow("Wait", frame);
			cv::moveWindow("Wait", x, y);
		}
		slow = 0;
	}
	else
	{
		slow++;
	}
}

void test(void)
{
	// for (int i = 0; i < MAX_VARIABLE_PER_TRACKBAR*MAX_NUMBER_OF_TRACKBAR; i++)
	// {
	// 	Tune_Float_t a;
	// 	a.varName = String("IloveJooWei") + to_string(i);
	// 	a.currentF = 10000.0;
	// 	a.minF = -20000.0;
	// 	a.maxF = 20000.0;
	// 	listFloatTune.push_back(a);
	// }

	for(int i=0; i<MAX_VARIABLE_PER_TRACKBAR*MAX_NUMBER_OF_TRACKBAR; i++){
		Tune_Int_t a;
		a.varName = String("IloveJooWei") + to_string(i);
		a.current = 10000;
		a.min = -10000;
		a.max = 20000;
		listIntTune.push_back(a);
	}

	// for (int i = 0; i < MAX_VARIABLE_PER_TRACKBAR * MAX_NUMBER_OF_TRACKBAR / 2; i++)
	// {
	// 	Tune_Float_t a;
	// 	a.varName = String("IloveJooWei") + to_string(i);
	// 	a.currentF = 10000.0;
	// 	a.minF = -20000.0;
	// 	a.maxF = 20000.0;
	// 	listFloatTune.push_back(a);
	// }

	// for (int i = 0; i < MAX_VARIABLE_PER_TRACKBAR * MAX_NUMBER_OF_TRACKBAR / 2; i++)
	// {
	// 	Tune_Int_t a;
	// 	a.varName = String("IloveJooWei") + to_string(i);
	// 	a.current = 10000;
	// 	a.min = -10000;
	// 	a.max = 20000;
	// 	listIntTune.push_back(a);
	// }
	for (int i = 0; i < listIntTune.size(); i++)
	{
		static int listNum = 0;
		static String TrackbarName;
		listIntTune.at(i).offsetCur = listIntTune.at(i).current - listIntTune.at(i).min;
		int range = listIntTune.at(i).max - listIntTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			cout << "created trackbar" << endl;
			TrackbarName = "TuneInt" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			if (numTrackbar == 0)
				cv::moveWindow(TrackbarName, 0, 0);
			// else if (numTrackbar == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			else
				cv::moveWindow(TrackbarName, numTrackbar * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			numTrackbar++;
		}
		createTrackbar(listIntTune.at(i).varName, TrackbarName, &(listIntTune.at(i).offsetCur), range);
	}
	for (int i = 0; i < listFloatTune.size(); i++)
	{
		static int listNum = 0;
		static String TrackbarName;
		listFloatTune.at(i).current = listFloatTune.at(i).currentF * PRECISION;
		listFloatTune.at(i).min = listFloatTune.at(i).minF * PRECISION;
		listFloatTune.at(i).max = listFloatTune.at(i).maxF * PRECISION;
		listFloatTune.at(i).offsetCur = listFloatTune.at(i).current - listFloatTune.at(i).min;
		int range = listFloatTune.at(i).max - listFloatTune.at(i).min;
		if (i % MAX_VARIABLE_PER_TRACKBAR == 0)
		{
			TrackbarName = "TuneFloat" + to_string(listNum);
			cv::namedWindow(TrackbarName, WINDOW_AUTOSIZE);
			if (numTrackbar == 0)
				cv::moveWindow(TrackbarName, 0, 0);
			// else if (numTrackbar == 1)
			// 	cv::moveWindow(TrackbarName, OFFSET_DUE_TO_UBUNTU, 0);
			else
				cv::moveWindow(TrackbarName, numTrackbar * WIDTH_PER_TRACKBAR, 0);
			listNum++;
			numTrackbar++;
		}
		createTrackbar(listFloatTune.at(i).varName, TrackbarName, &(listFloatTune.at(i).offsetCur), range);
	}
	system("clear");
}

void generate(void)
{
	for (int i = 0; i < listIntTune.size(); i++)
	{
		std::for_each(listIntTune.at(i).varName.begin(), listIntTune.at(i).varName.end(), [](char &c)
					  { c = ::toupper(c); });
		std::cout << "#define " << listIntTune.at(i).varName << "   " << listIntTune.at(i).current << endl;
	}

	for (int i = 0; i < listFloatTune.size(); i++)
	{
		std::for_each(listFloatTune.at(i).varName.begin(), listFloatTune.at(i).varName.end(), [](char &c)
					  { c = ::toupper(c); });
		std::cout << "#define " << listFloatTune.at(i).varName << "   " << listFloatTune.at(i).current << endl;
	}
}

void log(int s)
{
	//Activate byte ['p']
	//terminate byte [0x15]
	//incoming format : char
	//targetVelocity, MotorAVelocity, MotorBVelocity, MotorCVelocity, MotorDVelocity
	char activate = 'p';
	uint8_t buffer, newline = 0;
	FILE *csv;
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	char filename[50];
	char path[] = CSV_PATH;
	sprintf(filename, "PID_M%d-D%d-H%d-m%d-s%d.csv", timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	String str(filename);
	fileName.push_back(str);
	strcat(path, filename);
	csv = fopen(path, "w");
	// csv.open(filename);
	printf("%s created\n", filename);
	sprintf(filename, "PID_M%d-D%d-H%d-m%d-s%d.jpg", timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	String stri(filename);
	imgName.push_back(stri);
	send(s, &activate, 1, 0);
	recv(s, &buffer, 1, 0);
	do
	{
		fprintf(csv, "%c", buffer);
		printf("%c", buffer);
		recv(s, &buffer, 1, 0);
	} while (buffer != 0x15);
	fclose(csv);
}

void plotPID(int screenWidth, int screenHeight)
{
	//targetVelocity, MotorAVelocity, MotorBVelocity, MotorCVelocity, MotorDVelocity
	//Assuming the listFloatTune
	// 0 AP		3 BP	6 CP	9  DP
	// 1 AI		4 BI	7 CI	10 DI
	// 2 AD		5 BD	8 CD	11 DD
	fstream file(String(CSV_PATH) + fileName.at(fileNum), ios::in);
	string line, word;
	vector<float> targetVelocity[4];
	vector<float> motor[4];
	vector<float> time;
	float secondS = 0.0;
	if (file.is_open())
	{
		while (getline(file, line))
		{
			stringstream str(line);
			//  0   1    2   3    4   5   6    7
			//tarA, A, tarB, B, tarC, C, tarD, D
			int i = 0;
			while (getline(str, word, ','))
			{
				if (i % 2 == 0)
					targetVelocity[i/2].push_back(stof(word));
				else
					motor[(i-1)/2].push_back(stof(word));
				i++;
			}
			time.push_back(secondS);
			secondS += SAMPLE_TIME;
		}
	}
	else
	{
		cout << fileName.at(fileNum) << "Could not open the file\n";
	}
	cout << "done data" << endl;
	CvPlot::Axes axes[4];
	cv::Mat mat[4];
	const char linetype[4][3] = {"-r", "-m", "-g", "-c"};
	String motorName[4] = {"A motor", "B motor", "C motor", "D motor"};
	for (int i = 0; i < 4; i++)
	{
		axes[i] = CvPlot::makePlotAxes();
		axes[i].create<CvPlot::Series>(time, targetVelocity[i], "-b");
		axes[i].create<CvPlot::Series>(time, motor[i], linetype[i]);
		mat[i] = axes[i].render(screenHeight / 3, screenWidth / 3);
		cv::putText(mat[i], motorName[i], cv::Point(40, 30), 1, 1.2, cv::Scalar(0, 0, 0), 2, 8, 0);
		if (motor[3].size() == 0 && i == 3)
			break;
#ifndef DEMO
		char fl3[7];
		sprintf(fl3, "%.3f", listFloatTune.at(0 + (i * 3)).currentF);
		String FL3(fl3);
		cv::putText(mat[i], listFloatTune.at(0 + (i * 3)).varName + "=" + FL3, cv::Point(160, 30), 1, 1.7, cv::Scalar(0, 0, 0), 2, 8, 0);
		sprintf(fl3, "%.3f", listFloatTune.at(1 + (i * 3)).currentF);

		FL3 = fl3;
		cv::putText(mat[i], listFloatTune.at(1 + (i * 3)).varName + "=" + FL3, cv::Point(300, 30), 1, 1.7, cv::Scalar(0, 0, 0), 2, 8, 0);
		sprintf(fl3, "%.3f", listFloatTune.at(2 + (i * 3)).currentF);
		FL3 = fl3;
		cv::putText(mat[i], listFloatTune.at(2 + (i * 3)).varName + "=" + FL3, cv::Point(440, 30), 1, 1.7, cv::Scalar(0, 0, 0), 2, 8, 0);
#else
		cv::putText(mat[i], "P=1.500", cv::Point(140, 30), 1, 1.2, cv::Scalar(0, 0, 0), 2, 8, 0);
		cv::putText(mat[i], "I=1.500", cv::Point(240, 30), 1, 1.2, cv::Scalar(0, 0, 0), 2, 8, 0);
		cv::putText(mat[i], "D=0.005", cv::Point(340, 30), 1, 1.2, cv::Scalar(0, 0, 0), 2, 8, 0);
#endif
	}

	cv::Mat first1, second, output;
	cv::hconcat(mat[0], mat[1], first1);
	cv::hconcat(mat[2], mat[3], second);
	cv::vconcat(first1, second, output);
	cv::imshow(imgName.at(fileNum), output);

	cv::moveWindow(imgName.at(fileNum), 0, 0);

	cv::imwrite(JPG_PATH + imgName.at(fileNum), output);
	if (fileNum > 0)
	{
		destroyWindow(imgName.at(fileNum - 1));
		imgIndex++;
	}
	fileNum += 1;
}

void analyse(void)
{
	cv::Mat pid = cv::imread(JPG_PATH + imgName.at(imgIndex), 1);
	cv::imshow(imgName.at(imgIndex), pid);
	cv::moveWindow(imgName.at(imgIndex), 0, 0);
}

void backUp(cv::Mat board, cv::Mat board1, cv::Mat boardF, cv::Mat boardF1)
{
	if (listIntTune.size() > 0)
	{
		cv::imwrite("BoardInt" + Page + ".jpg", board);
		std::cout << "BoardInt" + Page + ".jpg saved" << endl;
	}
	if (!board1.empty())
	{
		cv::imwrite("BoardInt1" + Page + ".jpg", board1);
		std::cout << "BoardInt1" + Page + ".jpg saved" << endl;
	}

	if (listFloatTune.size() > 0)
	{
		cv::imwrite("BoardF" + Page + ".jpg", boardF);
		std::cout << "BoardF" + Page + ".jpg saved" << endl;
	}
	if (!boardF1.empty())
	{
		cv::imwrite("BoardF1" + Page + ".jpg", boardF1);
		std::cout << "BoardF1" + Page + ".jpg saved" << endl;
	}
}

void clean(void){
	if (listIntTune.size() > 0){
		destroyWindow("TuneInt0");
		destroyWindow("BoardInt");
	}
	if (listIntTune.size() > MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt1");
	if (listIntTune.size() > 2*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt2");
	if (listIntTune.size() > 3*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneInt3");

	if(listIntTune.size() > MAX_VARIABLE_PER_BOARD)
		destroyWindow("BoardInt1");

	if (listFloatTune.size() > 0){
		destroyWindow("TuneFloat0");
		destroyWindow("BoardFloat");
	}
	if (listFloatTune.size() > MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat1");
	if (listFloatTune.size() > 2*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat2");
	if (listFloatTune.size() > 3*MAX_VARIABLE_PER_TRACKBAR)
		destroyWindow("TuneFloat3");

	if(listFloatTune.size() > MAX_VARIABLE_PER_BOARD)
		destroyWindow("BoardFloat1");	
}
