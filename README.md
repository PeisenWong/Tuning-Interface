# TuningInterface

## Dependencies
`sudo apt update`

`sudo apt install libbluetooth-dev`

`sudo apt install libopencv-dev`

`gnome-extensions disable ubuntu-dock@ubuntu.com`

## DEMO

Uncomment DEMO 
```
// #define FIND_X_Y                                         /* To Find X pixel and Y pixel of window                          */
#define DEMO                                             /* To Look at the demo of the outlook without coonnect to bluebee */
```
`rm tune`

`make`

`./tune 123`

### Welcoming Page
![Screenshot from 2022-03-19 12-39-14](https://user-images.githubusercontent.com/80484903/159107125-23f55bf4-b2c6-4f89-89e1-5d1a4be5b13e.png)

### Press t
![Screenshot from 2022-03-19 12-40-22](https://user-images.githubusercontent.com/80484903/159107220-412a2944-0305-430b-94c5-e798de04d5bf.png)

### Press p three times to plot three different graphs, Press left or right arrow key to switch between graphs
https://user-images.githubusercontent.com/80484903/159108603-cadf1eb9-01f6-4228-bd66-497994716b0b.mp4


## Custom Settings of the Interface 
```
/*************************** CUSTOM AREA ***************************/
#define MAX_VARIABLE_PER_TRACKBAR   18
#define MAX_VARIABLE_PER_BOARD      36
#define MAX_NUMBER_OF_TRACKBAR      4
#define WIDTH_PER_TRACKBAR          320
#define OFFSET_DUE_TO_UBUNTU        0
#define BOARD_INT_WIDTH             270
#define BOARD_INT_HEIGHT            370
#define BOARD_INT_POS_X             390
#define BOARD_INT_POS_Y             0
#define BOARD_FLOAT_WIDTH           300
#define BOARD_FLOAT_HEIGHT          370
#define BOARD_FLOAT_POS_X           1620
#define BOARD_FLOAT_POS_Y           0

#define SAMPLE_TIME                 0.02
#define CSV_PATH                    "./CSV/"
#define JPG_PATH                    "./jpg/"
/*************************** CUSTOM AREA ***************************/
```

## Protocol
### Mainboard send tune variables to PC through HC-06:
```
//[NUM_INT_TUNE][NUM_FLOAT_TUNE]
//[0x52][0x01][strlen][str...][*curValue...][min ...][max ...][0x31][0x40] format of INT
//[0x14][0x01][strlen][str...][*curValue...][minF...][maxF...][0x20][0x00] format of FLOAT
```  
### PC request variables update from mainboard:
```
//[0x52][0x01][index][*curValue...][0x31][0x40] format of INT
//[0x14][0x01][index][*curValue...][0x20][0x00] format of FLOAT
``` 
### PC send edited tune variables to mainboard:
```
//[0x52][0x01][index][editValue...][0x31][0x40] format of INT
//[0x14][0x01][index][editValue...][0x20][0x00] format of FLOAT
//[0x88][0x77] terminate, back to tune pending
```
### In PID tuning mode:
```
//Activate byte ['p']
//terminate byte [0x15]
//incoming format : char
//targetVelocity, MotorAVelocity, MotorBVelocity, MotorCVelocity, MotorDVelocity
  
//Assuming the listFloatTune
// 0 AP		3 BP	6 CP	9  DP
// 1 AI		4 BI	7 CI	10 DI
// 2 AD		5 BD	8 CD	11 DD
```
Current code requires user to set the variables in mainboard as shown below
```
typedef struct{
	char* varName;
	float* ptr;
	float min;
	float max;
}Tune_Float_t;

#define var(name) #name
#define varF(__NAME__, __MIN__, __MAX__) {var(__NAME__), &(__NAME__), __MIN__, __MAX__}

Tune_Float_t TuneFloatList0[NUM_FLOAT_TUNE_LIST0]={
		varF(AP, 0.5, 5.0),
		varF(AI, 0.5, 5.0),
		varF(AD, 0.5, 5.0),
		varF(BP, 0.5, 5.0),
		varF(BI, 0.5, 5.0),
		varF(BD, 0.5, 5.0),
		varF(CP, 0.5, 5.0),
		varF(CI, 0.5, 5.0),
		varF(CD, 0.5, 5.0),
		varF(DP, 0.5, 5.0),
		varF(DI, 0.5, 5.0),
		varF(DD, 0.5, 5.0),
};
```
