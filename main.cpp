//QMapControl libraries must be compiled and built with the same compiler as this app
//otherwise error will occur when compiling the app
#include "mainwindow.h"
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsWidget>
#include <QLabel>
#include <QLineSeries>
#include <QtMath>
#include <QPushButton>
#include <QString>
#include <QSignalMapper>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
#include <QWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QSlider>
#include <QSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QRadioButton>
#include <QMenu>
#include <QDial>
#include <QTabWidget>
#include <QMenuBar>
#include <QCheckBox>
#include <QTime>
#include <QDate>
#include <QDateTime>
#include <QTabWidget>
#include <QMenuBar>
#include <QColor>
#include <QApplication>
#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <QMediaMetaData>
#include <QGeoCodingManager>
 #include <QGeoCoordinate>
#include <QNmeaPositionInfoSource>
#include <QGeoServiceProvider>
#include <QGeoPositionInfo>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QAction>
#include <iso646.h>
#include "GetErrName.h"
#include "MPU9250.h"
#include "mcp2221_dll_um.h"

#include "QMapControl/QMapControl.h"
#include "QMapControl/LayerMapAdapter.h"
#include "QMapControl/MapAdapterOSM.h"

//Global variables
void *handle;
QString error_string;
QTextEdit *MMA8452_TextBox;

//MCP2221 variables
wchar_t SerNum = 0x0000075428;
wchar_t LibVer[6];
wchar_t MfrDescriptor[30];
wchar_t ProdDescrip[30];
int ver = 0;
int error = 0;
int flag = 0;

unsigned int delay = 0;
unsigned int ReqCurrent;
unsigned int PID = 0xDD;
unsigned int VID = 0x4D8;
unsigned int NumOfDev = 0;
unsigned char PowerAttrib;

//I/O variables
unsigned char pinFunc[4] = {MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO};
unsigned char pinDir[4] = {MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT};
unsigned char OutValues[4] = {MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW};
unsigned int ADCbuffer[3];
unsigned char DacVal = 0;
unsigned int ADCreading = 0;
unsigned int NegativeFlag = 0;

#define I2cAddr7bit 1
#define I2cAddr8bit 0

//File logging variables
FILE * logFile;		//Create file to save temperature log
int fileFlag = 0;
int MMA8452_fileFlag = 0, MPU9250_fileFalg = 0, AnalogIn_fileFlag = 0, GPS_fileFlag = 0;
QString fileName;
char FileName[80] = "log-";
FILE * MMA8452_logFile, * MPU9250_logFile, * AnalogIn_logFile, * GPS_logFile;
char MMA8452_FileName[80] = "MMA8452_log-";
char MPU9250_FileName[80] = "MPU9250_log-";
char AnalogIn_FileName[80] = "Analog_log-";
char GPS_FileName[80] = "GPS_log-";

//MMA8452
unsigned char MMA8452_I2C_ADDR = 0x1C;			//MMA8452 accelerometer, SA0=0 Add=0x1C, SA0=1 Add=0x1D
unsigned char MMA8452_WHO_AM_I = 0x0D;
unsigned char MMA8452_CTRL_REG1 = 0x2A;
unsigned char MMA8452_XYZ_CFG_REG = 0x0E;
unsigned char MMA8452_STATUS_REG = 0x00;

unsigned char setupTap[3] = {0};
enum MMA8452Q_Scale {SCALE_2G = 2, SCALE_4G = 4, SCALE_8G = 8}; // Possible full-scale settings
enum MMA8452Q_ODR {ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6, ODR_1}; // possible data rates
unsigned char config[2] = {0};
int FullScaleRange = SCALE_2G;  //default is +/-2g
unsigned char RxData[7];

//MMA8452 chart variables
using namespace QtCharts;
QChartView *MMA8452_chartViewAcc;
QLineSeries *MMA8452_seriesX;
QLineSeries *MMA8452_seriesY;
QLineSeries *MMA8452_seriesZ;
QChart *MMA8452_chartAcc;
QPointF MMA8452_numX, MMA8452_numY, MMA8452_numZ;
QComboBox *MMA8452_RangeDropList;
QValueAxis *MMA8452_axisY1;

//MMA8452 Plotting variables
int counter = 0;
int MMA8452_x1RangeMax = 180, MMA8452_x1RangeMin = 0, MMA8452_y1RangeMax = 2, MMA8452_y1RangeMin = -2;
double MMA8452_IMUval[3];

//checkboxes
QCheckBox *MMA8452_GPIO0chkBx;
QCheckBox *MMA8452_GPIO1chkBx;
QCheckBox *MMA8452_GPIO2chkBx;
QCheckBox *MMA8452_GPIO3chkBx;
QCheckBox *MMA8452_SaveTofilechkbx;
QTimer *MMA8452_timer;

//MPU9250 variables
//Create instance of MPU9250
extern unsigned char MPU9250_I2C_ADDR; 	//default MPU9250 I2C address
MPU9250 IMU(MPU9250_I2C_ADDR);

QTextEdit *MPU9250_TextBox;
QChartView *MPU9250_chartViewAcc;
QLineSeries *MPU9250_seriesX;
QLineSeries *MPU9250_seriesY;
QLineSeries *MPU9250_seriesZ;
QChart *MPU9250_chartAcc;
QPointF MPU9250_numX, MPU9250_numY, MPU9250_numZ;
QComboBox *MPU9250_AccFscaleDropList;
QComboBox *MPU9250_GyroFscaleDropList;
QValueAxis *MPU9250_axisYAcc;

QChartView *MPU9250_chartViewGyro;
QLineSeries *MPU9250_seriesYaw;
QLineSeries *MPU9250_seriesPitch;
QLineSeries *MPU9250_seriesRoll;
QChart *MPU9250_chartGyro;
QPointF MPU9250_numYaw, MPU9250_numPitch, MPU9250_numRoll;
QValueAxis *MPU9250_axisYGyro;

QChartView *MPU9250_chartViewMagneto;
QLineSeries *MPU9250_seriesMx;
QLineSeries *MPU9250_seriesMy;
QLineSeries *MPU9250_seriesMz;
QChart *MPU9250_chartMagneto;
QPointF MPU9250_numMx, MPU9250_numMy, MPU9250_numMz;
QValueAxis *MPU9250_axisYMagneto;

QPointF MPU9250_numIMU[9];
double MPU9250_IMUval[9];
qreal MPU9250_x1RangeMax = 180, MPU9250_x1RangeMin = 0, MPU9250_y1RangeMax = 16, MPU9250_y1RangeMin = -16;
qreal MPU9250_yGyroRangeMin = -20, MPU9250_yGyroRangeMax = 20, MPU9250_xGyroRangeMin = 0, MPU9250_xGyroRangeMax = 180;
qreal MPU9250_yMagnetoRangeMin = -480, MPU9250_yMagnetoRangeMax = 480, MPU9250_xMagnetoRangeMin = 0, MPU9250_xMagnetoRangeMax = 180;
int j = 0;

//checkboxes
QCheckBox *MPU9250_GPIO0chkBx;
QCheckBox *MPU9250_GPIO1chkBx;
QCheckBox *MPU9250_GPIO2chkBx;
QCheckBox *MPU9250_GPIO3chkBx;
QCheckBox *MPU9250_SaveTofilechkbx;
QTimer *MPU9250_timer;

//Neo-6 GPS
using namespace std;
using namespace qmapcontrol;
QGeoPositionInfo* location;
QDateTime timestamp;
QNmeaPositionInfoSource* GPSdevice;
QSerialPort* serial;
QGeoServiceProvider* ServiceProvider;
QMapControl* m_map_control;
QComboBox *serial_droplist;
double longitude = 0.0;
double latitude = 0.0;
double altitude = 0.0;
std::vector<PointWorldCoord> points;
QGeoCoordinate Coordinate(latitude, longitude);
QTextEdit *Neo6GPS_TextBox;

//Analog inputs variables
//checkboxes
QCheckBox *AnalogIn_AN0chkBx;
QCheckBox *AnalogIn_AN1chkBx;
QCheckBox *AnalogIn_AN2chkBx;
QCheckBox *AnalogIn_SaveTofilechkbx;
QTextEdit *AnalogIn_TextBox;
QChartView *AnalogIn_chartViewAcc;
QLineSeries *AnalogIn_seriesX;
QLineSeries *AnalogIn_seriesY;
QLineSeries *AnalogIn_seriesZ;
QChart *AnalogIn_chartAcc;
QPointF AnalogIn_numX, AnalogIn_numY, AnalogIn_numZ;
QComboBox *AnalogIn_VREF_DropList;
QValueAxis *AnalogIn_axisYAcc;
qreal AnalogIn_x1RangeMax = 1000, AnalogIn_x1RangeMin = 0;
qreal AnalogIn_y1RangeMax = 5, AnalogIn_y1RangeMin = 0;
QTimer *AnalogIn_timer;
unsigned int AnalogIn_flag[3] = {0,0,0}, AIN_counter = 0;
double AnalogIn_voltage[3];
float Vref = 5.0;

//Exit function
void ExitFunc()
{
    Mcp2221_Reset(handle);
}

//Configure MCP2221
void Mcp2221_config()
{
    ver = Mcp2221_GetLibraryVersion(LibVer);		//Get DLL version
    if(ver == 0)
    {
        error_string = "Library (DLL) version: "+QString::fromWCharArray(LibVer);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error = Mcp2221_GetLastError();
        error_string = "Cannot get version, error: " + QString::fromStdString(Mcp2221_GetErrorName(error));
        MMA8452_TextBox->append(error_string);
    }

    //Get number of connected devices with this VID & PID
    Mcp2221_GetConnectedDevices(VID, PID, &NumOfDev);
    if(NumOfDev == 0)
    {
        error_string = "No MCP2221 devices connected";
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Number of devices found: " + QString::number(NumOfDev);
        MMA8452_TextBox->append(error_string);
    }

    //Open device by index
    handle = Mcp2221_OpenByIndex(VID, PID, NumOfDev-1);
    if(error == NULL)
    {
        error_string = "Connection successful";
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error = Mcp2221_GetLastError();
        error_string = "Error message is "+ QString::number(error) + " " + QString(Mcp2221_GetErrorName(error));
        MMA8452_TextBox->append(error_string);
        _sleep(10000);
    }

    //Get manufacturer descriptor
    flag = Mcp2221_GetManufacturerDescriptor(handle, MfrDescriptor);
    if(flag == 0)
    {
        error_string = "Manufacturer descriptor: " + QString::fromWCharArray(MfrDescriptor);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error getting descriptor: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }

    //Get product descriptor
    flag = Mcp2221_GetProductDescriptor(handle, ProdDescrip);
    if(flag == 0)
    {
        error_string = "Product descriptor: " + QString::fromWCharArray(ProdDescrip);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error getting product descriptor:" + QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }

    //Get power attributes
    flag = Mcp2221_GetUsbPowerAttributes(handle, &PowerAttrib, &ReqCurrent);
    if(flag == 0)
    {
        error_string = "Power Attributes " + QString::number(PowerAttrib) + "\nRequested current units = " + QString::number(ReqCurrent) + "\nRequested current(mA) = " + QString::number(ReqCurrent*2);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error getting power attributes:"+ QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }

    //Set I2C bus
    flag = Mcp2221_SetSpeed(handle, 500000);    //set I2C speed to 400 KHz
    if(flag == 0)
    {
        error_string = "I2C is configured";
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error setting I2C bus:"+ QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }

    //Set I2C advanced parameters
    flag = Mcp2221_SetAdvancedCommParams(handle, 10, 100);  //10ms timeout, try 1000 times
    if(flag == 0)
    {
        error_string = "I2C advanced settings set";
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error setting I2C advanced settings:"+ QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }

    //Set GPIO
    /*flag = Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
    if(flag != 0)
    {
        //printf("Error setting GPIO, error: %d\n", flag);
        error_string = "Error setting GPIO, error: "+ QString::number(flag);
        MMA8452_TextBox->append(error_string);
    }*/
}

void MMA8452_Config()
{
    //Get MMA8452 device ID
    flag = Mcp2221_I2cWrite(handle, sizeof(MMA8452_WHO_AM_I), MMA8452_I2C_ADDR, I2cAddr7bit, &MMA8452_WHO_AM_I);
    if(flag == 0)
    {
        error_string = "Writing to device: " + QString::number(MMA8452_I2C_ADDR) + " successful";
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error requesting device ID: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    //Read response from MMA8452
    flag = Mcp2221_I2cRead(handle, 1, MMA8452_I2C_ADDR, I2cAddr7bit, RxData);	//sizeof(RxData)
    if(flag == 0)
    {
        error_string = "Device ID is: 0x" + QString::number(RxData[0], 16).toUpper();
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error reading device ID: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Active mode(0x03)
    //0b00000011: 50Hz auto-wake rate (00), 800 Hz ODR (000), reduced noise mode disabled (0), fast read mode enabled (1), active mode (1)
    //config[0] = 0x2A;
    config[0] = MMA8452_CTRL_REG1;
    config[1] = 0x01;//0x01;//0x03: fast read enables, data sample is limited to one byte (8-bits per sample)

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        error_string = "Setting MMA8452 rate: " + QString::number(MMA8452_I2C_ADDR);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error writing configuration to device: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Select configuration register(0x0E)
    // Set range to +/- 2g(0x00)
    config[0] = MMA8452_XYZ_CFG_REG;
    config[1] = 0x00;	//2g:0x00, 4g: 0x01, 8g: 0x02

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        error_string = "Setting MMA8452 range: " + QString::number(MMA8452_I2C_ADDR);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error writing to device: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Read 7 bytes of data(0x00)
    // staus, xAccl msb, xAccl lsb, yAccl msb, yAccl lsb, zAccl msb, zAccl lsb
    flag = Mcp2221_I2cWrite(handle, sizeof(MMA8452_STATUS_REG), MMA8452_I2C_ADDR, I2cAddr7bit, &MMA8452_STATUS_REG);    //issue start condition then address
    if(flag == 0)
    {
        error_string = "Requesting data from MMA8452 device: " + QString::number(MMA8452_I2C_ADDR);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error writing to device: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }
}

void MainWindow::MMA8452_updateChart()
{
    //Read response from MMA8452
    flag = Mcp2221_I2cRead(handle, sizeof(RxData), MMA8452_I2C_ADDR, I2cAddr7bit, RxData);
    if(flag == 0)
    {

    }
    else
    {
        error_string = "Error reading from device: " + QString::number(flag) + " " + QString(Mcp2221_GetErrorName(flag));;
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Convert the data to 12-bits
    int MMA8452_xAccl = ((RxData[1] * 256) + RxData[2]) / 16;
    if(MMA8452_xAccl > 2047)
    {
        MMA8452_xAccl -= 4096;
    }

    int MMA8452_yAccl = ((RxData[3] * 256) + RxData[4]) / 16;
    if(MMA8452_yAccl > 2047)
    {
        MMA8452_yAccl -= 4096;
    }

    int MMA8452_zAccl = ((RxData[5] * 256) + RxData[6]) / 16;
    if(MMA8452_zAccl > 2047)
    {
        MMA8452_zAccl -= 4096;
    }

    MMA8452_IMUval[0] = (float)MMA8452_xAccl / (float)(1 << 11) * (float)(FullScaleRange);
    MMA8452_IMUval[1] = (float)MMA8452_yAccl / (float)(1 << 11) * (float)(FullScaleRange);
    MMA8452_IMUval[2] = (float)MMA8452_zAccl / (float)(1 << 11) * (float)(FullScaleRange);

    counter++;
    if(counter == MMA8452_x1RangeMax)
    {
        counter = 0;
        MMA8452_seriesX->clear();
        MMA8452_seriesY->clear();
        MMA8452_seriesZ->clear();
    }
    MMA8452_numX.setX(counter);
    MMA8452_numX.setY(MMA8452_IMUval[0]);
    MMA8452_numY.setX(counter);
    MMA8452_numY.setY(MMA8452_IMUval[1]);
    MMA8452_numZ.setX(counter);
    MMA8452_numZ.setY(MMA8452_IMUval[2]);

    MMA8452_seriesX->append(MMA8452_numX);
    MMA8452_seriesY->append(MMA8452_numY);
    MMA8452_seriesZ->append(MMA8452_numZ);

    if(MMA8452_fileFlag == 1)
    {
        fprintf(logFile, "%.2f,%.2f,%.2f\n", MMA8452_IMUval[0],MMA8452_IMUval[1],MMA8452_IMUval[2]);
    }
}

void MainWindow::MMA8452_DropBoxCallback(int index)
{
    switch(MMA8452_RangeDropList->currentIndex())
    {
        case 0:
            qDebug() << "droplist index: " << MMA8452_RangeDropList->currentIndex();
            config[1] = 0x00;	//2g:0x00, 4g: 0x01, 8g: 0x02
            MMA8452_y1RangeMax = 2;
            MMA8452_y1RangeMin = -2;
            error_string = "Setting MMA8452 range to: +/- 2g ";
            FullScaleRange = SCALE_2G;
            qDebug() << "droplist index: " << MMA8452_RangeDropList->currentIndex() << "value: " << config[1];
            break;

        case 1:
            config[1] = 0x01;	//2g:0x00, 4g: 0x01, 8g: 0x02
            MMA8452_y1RangeMax = 4;
            MMA8452_y1RangeMin = -4;
            error_string = "Setting MMA8452 range to: +/- 4g ";
            FullScaleRange = SCALE_4G;
            qDebug() << "droplist index: " << MMA8452_RangeDropList->currentIndex() << "value: " << config[1];
            break;

        case 2:
            config[1] = 0x02;	//2g:0x00, 4g: 0x01, 8g: 0x02
            MMA8452_y1RangeMax = 8;
            MMA8452_y1RangeMin = -8;
            error_string = "Setting MMA8452 range to: +/- 8g ";
            FullScaleRange = SCALE_8G;
            qDebug() << "droplist index: " << MMA8452_RangeDropList->currentIndex() << "value: " << config[1];
            break;
    }
    config[0] = MMA8452_XYZ_CFG_REG;
    MMA8452_axisY1->setRange(MMA8452_y1RangeMin, MMA8452_y1RangeMax);

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        //error_string = "Setting MMA8452 range to: " + QString::number(MMA8452_I2C_ADDR);
        MMA8452_TextBox->append(error_string);
    }
    else
    {
        error_string = "Error writing to device: " + QString::number(flag);
        MMA8452_TextBox->append(error_string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }
}

void MainWindow::MMA8452_ChkBxCallback(int index)
{
    if(MMA8452_GPIO0chkBx->isChecked())
        OutValues[0] = 1;
    else
        OutValues[0] = 0;

    if(MMA8452_GPIO1chkBx->isChecked())
        OutValues[1] = 1;
    else
        OutValues[1] = 0;

    if(MMA8452_GPIO2chkBx->isChecked())
        OutValues[2] = 1;
    else
        OutValues[2] = 0;

    if(MMA8452_GPIO3chkBx->isChecked())
        OutValues[3] = 1;
    else
        OutValues[3] = 0;

    if(MMA8452_SaveTofilechkbx->isChecked())
    {
            MMA8452_fileFlag = 1;
            if(!MMA8452_logFile)
            {
                MMA8452_logFile = fopen(MMA8452_FileName, "a+");
                fprintf(MMA8452_logFile,"Ax,Ay,Az\n");
            }
    }
    else
        MMA8452_fileFlag = 0;

    Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
    Mcp2221_SetGpioValues(handle, OutValues);
}

void MainWindow::ConnectToCOM()
{
    serial->setPortName(serial_droplist->currentText());
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);
    GPSdevice->setDevice(serial);
}

void MainWindow::GetGPSinfo()
{
    error_string = "Latitude: " + QString::number(GPSdevice->lastKnownPosition(true).coordinate().latitude());
    Neo6GPS_TextBox->append(error_string);
    error_string = "Longitude: " + QString::number(GPSdevice->lastKnownPosition(true).coordinate().longitude());
    Neo6GPS_TextBox->append(error_string);
    error_string = "Altitude: " +  QString::number(GPSdevice->lastKnownPosition(true).coordinate().altitude());
    Neo6GPS_TextBox->append(error_string);
    error_string = "Time: " +  GPSdevice->lastKnownPosition(true).timestamp().toString();//.currentDateTime().toString();
    Neo6GPS_TextBox->append(error_string);

    latitude = GPSdevice->lastKnownPosition(true).coordinate().latitude();
    longitude = GPSdevice->lastKnownPosition(true).coordinate().longitude();
    points.emplace_back(longitude, latitude);
    m_map_control->setMapFocusPoint(points, true);

    Coordinate.setLatitude(latitude);
    Coordinate.setLongitude(longitude);
    timestamp = QDateTime::fromString(GPSdevice->lastKnownPosition(true).timestamp().toString());
}

//Digital output control checkbox
void MainWindow::MPU9250_ChkBxCallback(int index)
{
    if(MPU9250_GPIO0chkBx->isChecked())
        OutValues[0] = 1;
    else
        OutValues[0] = 0;

    if(MPU9250_GPIO1chkBx->isChecked())
        OutValues[1] = 1;
    else
        OutValues[1] = 0;

    if(MPU9250_GPIO2chkBx->isChecked())
        OutValues[2] = 1;
    else
        OutValues[2] = 0;

    if(MPU9250_GPIO3chkBx->isChecked())
        OutValues[3] = 1;
    else
        OutValues[3] = 0;

    if(MPU9250_SaveTofilechkbx->isChecked())
    {
            fileFlag = 1;
            if(!logFile)
            {
                logFile = fopen(MPU9250_FileName, "a+");
                fprintf(logFile,"Ax,Ay,Az,Gx,Gx,Gz,Mx,My,Mz,Temperature\n");
            }
    }
    else
        fileFlag = 0;

    Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
    Mcp2221_SetGpioValues(handle, OutValues);
}

//Chart plotting callback
void MainWindow::MPU9250_updateChart()
{

    IMU.readSensor();

    MPU9250_IMUval[0] = IMU.getAccelX_mss();
    MPU9250_IMUval[1] = IMU.getAccelY_mss();
    MPU9250_IMUval[2] = IMU.getAccelZ_mss();
    MPU9250_IMUval[3] = IMU.getGyroX_rads();
    MPU9250_IMUval[4] = IMU.getGyroY_rads();
    MPU9250_IMUval[5] = IMU.getGyroZ_rads();
    MPU9250_IMUval[6] = IMU.getMagX_uT();
    MPU9250_IMUval[7] = IMU.getMagY_uT();
    MPU9250_IMUval[8] = IMU.getMagZ_uT();

    counter++;
    if(counter == MPU9250_x1RangeMax)
    {
        counter = 0;
        MPU9250_seriesX->clear();
        MPU9250_seriesY->clear();
        MPU9250_seriesZ->clear();
        MPU9250_seriesYaw->clear();
        MPU9250_seriesPitch->clear();
        MPU9250_seriesRoll->clear();
        MPU9250_seriesMx->clear();
        MPU9250_seriesMy->clear();
        MPU9250_seriesMz->clear();
    }
    MPU9250_numX.setX(counter);
    MPU9250_numX.setY(MPU9250_IMUval[0]);

    MPU9250_numY.setX(counter);
    MPU9250_numY.setY(MPU9250_IMUval[1]);

    MPU9250_numZ.setX(counter);
    MPU9250_numZ.setY(MPU9250_IMUval[2]);

    MPU9250_numYaw.setX(counter);
    MPU9250_numYaw.setY(MPU9250_IMUval[3]);
    MPU9250_numPitch.setX(counter);
    MPU9250_numPitch.setY(MPU9250_IMUval[4]);
    MPU9250_numRoll.setX(counter);
    MPU9250_numRoll.setY(MPU9250_IMUval[5]);

    MPU9250_numMx.setX(counter);
    MPU9250_numMx.setY(MPU9250_IMUval[6]);
    MPU9250_numMy.setX(counter);
    MPU9250_numMy.setY(MPU9250_IMUval[7]);
    MPU9250_numMz.setX(counter);
    MPU9250_numMz.setY(MPU9250_IMUval[8]);


    MPU9250_seriesX->append(MPU9250_numX);
    MPU9250_seriesY->append(MPU9250_numY);
    MPU9250_seriesZ->append(MPU9250_numZ);
    MPU9250_seriesYaw->append(MPU9250_numYaw);
    MPU9250_seriesPitch->append(MPU9250_numPitch);
    MPU9250_seriesRoll->append(MPU9250_numRoll);
    MPU9250_seriesMx->append((MPU9250_numMx));
    MPU9250_seriesMy->append((MPU9250_numMy));
    MPU9250_seriesMz->append((MPU9250_numMz));

    if(fileFlag == 1)
    {
        fprintf(logFile, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                MPU9250_IMUval[0],MPU9250_IMUval[1],MPU9250_IMUval[2],MPU9250_IMUval[3],MPU9250_IMUval[4],
                MPU9250_IMUval[5],MPU9250_IMUval[6],MPU9250_IMUval[7],MPU9250_IMUval[8],IMU.getTemperature_C());
    }
}

void MainWindow::MPU9250_AccScale(int index)
{
    switch(MPU9250_AccFscaleDropList->currentIndex())
    {
        case 0:
            FullScaleRange = MPU9250::ACCEL_RANGE_2G;
            MPU9250_y1RangeMax = 2.0;
            MPU9250_y1RangeMin = -2.0;
            error_string = "Setting accelerometer range to: ±2g ";
            break;

        case 1:
            FullScaleRange = MPU9250::ACCEL_RANGE_4G;
            MPU9250_y1RangeMax = 4.0;
            MPU9250_y1RangeMin = -4.0;
            error_string = "Setting accelerometer range to: ±4g ";
            break;

        case 2:
            FullScaleRange = MPU9250::ACCEL_RANGE_8G;
            MPU9250_y1RangeMax = 8.0;
            MPU9250_y1RangeMin = -8.0;
            error_string = "Setting accelerometer range to: ±8g ";
            break;

        case 3:
            FullScaleRange = MPU9250::ACCEL_RANGE_16G;
            MPU9250_y1RangeMax = 16.0;
            MPU9250_y1RangeMin = -16.0;
            error_string = "Setting accelerometer range to: ±16g ";
            break;
    }
    IMU.setAccelRange(MPU9250::AccelRange(FullScaleRange));
    qDebug() << MPU9250_axisYAcc->max() << MPU9250_axisYAcc->min();
    MPU9250_axisYAcc->setRange(MPU9250_y1RangeMin, MPU9250_y1RangeMax);
    MPU9250_TextBox->append(error_string);
}

void MainWindow::MPU9250_GyroScale(int index)
{
    switch(MPU9250_GyroFscaleDropList->currentIndex())
    {
        case 0:
            FullScaleRange = MPU9250::GYRO_RANGE_250DPS;
            MPU9250_yGyroRangeMax = 250;
            MPU9250_yGyroRangeMin = -250;
            error_string = "Setting gyroscope range to: ±250°/sec";
            break;

        case 1:
            FullScaleRange = MPU9250::GYRO_RANGE_500DPS;
            MPU9250_yGyroRangeMax = 500;
            MPU9250_yGyroRangeMin = -500;
            error_string = "Setting gyroscope range to: ±500°/sec";
            break;

        case 2:
            FullScaleRange = MPU9250::GYRO_RANGE_1000DPS;
            MPU9250_yGyroRangeMax = 1000;
            MPU9250_yGyroRangeMin = -1000;
            error_string = "Setting gyroscope range to: ±1000°/sec";
            break;

        case 3:
            FullScaleRange = MPU9250::GYRO_RANGE_2000DPS;
            MPU9250_yGyroRangeMax = 2000;
            MPU9250_yGyroRangeMin = -2000;
            error_string = "Setting gyroscope range to: ±2000°/sec";
            break;
    }
    IMU.setGyroRange(MPU9250::GyroRange(FullScaleRange));
    MPU9250_axisYGyro->setRange(MPU9250_yGyroRangeMin, MPU9250_yGyroRangeMax);
    MPU9250_TextBox->append(error_string);
}

void MainWindow::AnalogIn_ChkBxCallback(int index)
{
    QString SettingMsg[3];
    if(AnalogIn_AN0chkBx->isChecked())
    {
        AnalogIn_flag[0] = 1;
        SettingMsg[0] = "AN0-GP1 selected";
    }
    else
    {
        AnalogIn_flag[0] = 0;
    }

    if(AnalogIn_AN1chkBx->isChecked())
    {
        AnalogIn_flag[1] = 1;
        SettingMsg[1] = "AN1-GP2 selected";
    }
    else
    {
        AnalogIn_flag[1] = 0;
    }
    if(AnalogIn_AN2chkBx->isChecked())
    {
        AnalogIn_flag[2] = 1;
        SettingMsg[2] = "AN2-GP3 selected";
    }
    else
    {
        AnalogIn_flag[2] = 0;
    }

    if(AnalogIn_SaveTofilechkbx->isChecked())
    {
        AnalogIn_fileFlag = 1;
        if(!AnalogIn_logFile)
        {
            AnalogIn_logFile = fopen(AnalogIn_FileName, "a+");
            fprintf(AnalogIn_logFile,"AN0, AN1, AN2\n");
        }
    }
    else
        AnalogIn_fileFlag = 0;

    for(j = 0; j < 3; j++)
    {
        AnalogIn_TextBox->append(SettingMsg[j]);
    }
}

void MainWindow::AnalogIn_updateChart()
{
    flag = Mcp2221_GetAdcData(handle, ADCbuffer);

    AnalogIn_voltage[0] = (float)ADCbuffer[0]*(Vref/1023.0);
    AnalogIn_voltage[1] = (float)ADCbuffer[1]*(Vref/1023.0);
    AnalogIn_voltage[2] = (float)ADCbuffer[2]*(Vref/1023.0);
    AIN_counter++;
    if(AIN_counter == AnalogIn_x1RangeMax)
    {
        AIN_counter = 0;
        AnalogIn_seriesX->clear();
        AnalogIn_seriesY->clear();
        AnalogIn_seriesZ->clear();
    }

    if(AnalogIn_flag[0] == 1)
    {
        AnalogIn_numX.setX(AIN_counter);
        AnalogIn_numX.setY(AnalogIn_voltage[0]);
    }
    else
    {
        AnalogIn_numX.setX(AIN_counter);
        AnalogIn_numX.setY(0);
    }

    if(AnalogIn_flag[1] == 1)
    {
        AnalogIn_numY.setX(AIN_counter);
        AnalogIn_numY.setY(AnalogIn_voltage[1]);
    }
    else
    {
        AnalogIn_numY.setX(AIN_counter);
        AnalogIn_numY.setY(0);
    }

    if(AnalogIn_flag[2] == 1)
    {
        AnalogIn_numZ.setX(AIN_counter);
        AnalogIn_numZ.setY(AnalogIn_voltage[2]);
    }
    else
    {
        AnalogIn_numZ.setX(AIN_counter);
        AnalogIn_numZ.setY(0);
    }

    AnalogIn_seriesX->append(AnalogIn_numX);
    AnalogIn_seriesY->append(AnalogIn_numY);
    AnalogIn_seriesZ->append(AnalogIn_numZ);

    if(AnalogIn_fileFlag == 1)
    {
        fprintf(AnalogIn_logFile, "%.2f,%.2f,%.2f\n", AnalogIn_voltage[0],AnalogIn_voltage[1],AnalogIn_voltage[2]);
    }
}

void MainWindow::ExitApp()
{
    exit(0);
}

void MainWindow::AboutApp()
{
    QMessageBox msgbox;
    msgbox.setText("MCP2221 Sensors Studio is a desktop app\n "
                   "that interfaces MCP2221 USB chip with\n"
                   "different sensors over I2C and UART.");
    msgbox.setWindowTitle("About MCP2221 Sensors Studio");
    msgbox.exec();
}

void EnableDigitalIO()
{
    pinFunc[0] = MCP2221_GPFUNC_IO;
    pinFunc[1] = MCP2221_GPFUNC_IO;
    pinFunc[2] = MCP2221_GPFUNC_IO;
    pinFunc[3] = MCP2221_GPFUNC_IO;
    pinDir[0] = MCP2221_GPDIR_OUTPUT;
    pinDir[1] = MCP2221_GPDIR_OUTPUT;
    pinDir[2] = MCP2221_GPDIR_OUTPUT;
    pinDir[3] = MCP2221_GPDIR_OUTPUT;
    Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
}

void EnableAnalogIn()
{
    pinFunc[1] = MCP2221_GP_ADC;
    pinFunc[2] = MCP2221_GP_ADC;
    pinFunc[3] = MCP2221_GP_ADC;
    pinDir[1] = NO_CHANGE;
    pinDir[2] = NO_CHANGE;
    pinDir[3] = NO_CHANGE;
    Mcp2221_SetAdcVref(handle, RUNTIME_SETTINGS, VREF_VDD);
    Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
}

//Enable/disable timers based on active tab
void MainWindow::ActiveTabs(int index)
{
    if(index == 0)
    {
        qDebug() << "MPU9250 window is active";
        MPU9250_timer->start(0);
        MMA8452_timer->stop();
        AnalogIn_timer->stop();
        EnableDigitalIO();
    }
    else if(index == 1)
    {
        qDebug() << "MMA8452 window is active";
        MPU9250_timer->stop();
        MMA8452_timer->start(0);
        AnalogIn_timer->stop();
        EnableDigitalIO();
    }
    else if(index == 2)
    {
        qDebug() << "Analog inputs window is active";
        MPU9250_timer->stop();
        MMA8452_timer->stop();
        AnalogIn_timer->start(0);
        EnableAnalogIn();
    }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setFixedSize(1350, 935);
    w.setWindowTitle("MCP2221 Motion Sensors Studio");   //Window title
    atexit(ExitFunc);	//register exit function

    QString BackGroundColor = w.palette().color(QPalette::Window).name();

    //Create Menu bar and menus
    QMenuBar *menu = new QMenuBar(&w);
    menu->setGeometry(15, 5, 200, 30);
    QString colorStr = QString("background-color:") + BackGroundColor + ";";    //get background color
    menu->setStyleSheet(colorStr);      //change toolbar menu background color

    QMenu* FileMenu = new QMenu("File");
    QMenu* HelpMenu = new QMenu("Help");

    //add menus to the menu bar
    menu->setFixedSize(200, 30);
    menu->addMenu(FileMenu);
    menu->addMenu(HelpMenu);
    FileMenu->setBackgroundRole(QPalette::Window);
    HelpMenu->setBackgroundRole(QPalette::Window);

    //add "exit" to "File" menu
    QAction* ExitAction = new QAction("Exit", &w);
    FileMenu->addAction(ExitAction);
    QObject::connect(ExitAction, SIGNAL(triggered()), &w, SLOT(ExitApp()));    //hovered() also is a good option

    //add "about" to "Help" menu
    QAction* AboutAction = new QAction("About", &w);
    HelpMenu->addAction(AboutAction);
    QObject::connect(AboutAction, SIGNAL(triggered()), &w, SLOT(AboutApp()));    //hovered() also is a good option

    //Create widgets
    QWidget* MPU9250_Tab1 = new QWidget();
    QWidget* MMA8452_Tab2 = new QWidget();
    QWidget* AnalogIn_Tab3 = new QWidget();

    //Create tabs, add widgets to each tab
    QTabWidget *tabs = new QTabWidget(&w);
    tabs->setGeometry(15, 35, 1315, 890);
    tabs->setTabsClosable(false);
    tabs->setStyleSheet(colorStr);
    tabs->addTab(MPU9250_Tab1, "MPU9250");
    tabs->addTab(MMA8452_Tab2, "MMA8452");
    tabs->addTab(AnalogIn_Tab3, "Analog Inputs");

    //MPU9250_Tab1->isActiveWindow();
    QObject::connect(tabs, SIGNAL(currentChanged(int)), &w, SLOT(ActiveTabs(int)));

    //Get time stamp for data-logging file name
    char *LocalTime = 0;
    char temp[24] = {0};
    time_t LogTime;	//Local time at the time of creation
    struct tm * timeinfo;
    time(&LogTime);
    LocalTime = ctime(&LogTime);
    timeinfo = localtime (&LogTime);
    strftime(temp, sizeof(temp), "%d%b%Y-%H-%M-%S", timeinfo);	//day-month-year-hour-minute-second
    strcat(FileName, temp);
    strcat(FileName, ".csv");
    strcat(MMA8452_FileName, temp);
    strcat(MMA8452_FileName, ".csv");
    strcat(MPU9250_FileName, temp);
    strcat(MPU9250_FileName, ".csv");
    strcat(AnalogIn_FileName, temp);
    strcat(AnalogIn_FileName, ".csv");

    //MMA8452 Chart tab
    //Label 1 - MCP2221 info
    QLabel *MMA8452_Label1 = new QLabel(MMA8452_Tab2);
    MMA8452_Label1->setGeometry(15, 460, 150, 25);
    MMA8452_Label1->setText("Device Configuration info");

    //MMA8452_TextBox for device information
    MMA8452_TextBox = new QTextEdit(MMA8452_Tab2);
    MMA8452_TextBox->setGeometry(15, 485, 380, 150);
    MMA8452_TextBox->acceptRichText();
    MMA8452_TextBox->setAcceptRichText(true);

    //MMA8452 range
    QLabel *MMA8452_RangeLabel = new QLabel(MMA8452_Tab2);
    MMA8452_RangeLabel->setGeometry(410, 460, 180, 25);
    MMA8452_RangeLabel->setText("Select MMA8452 Range:");

    //Drop-down list
    MMA8452_RangeDropList = new QComboBox(MMA8452_Tab2);
    MMA8452_RangeDropList->setGeometry(410, 485, 100, 35);
    MMA8452_RangeDropList->addItem("2g");
    MMA8452_RangeDropList->addItem("4g");
    MMA8452_RangeDropList->addItem("8g");

    //checkbox for GPIO - output only
    QSignalMapper *MMA8452_CheckBoxSignalMapper = new QSignalMapper(MMA8452_Tab2);
    MMA8452_GPIO0chkBx = new QCheckBox("GPIO0",MMA8452_Tab2);
    MMA8452_GPIO0chkBx->setGeometry(410, 535, 100, 20);

    MMA8452_GPIO1chkBx = new QCheckBox("GPIO1",MMA8452_Tab2);
    MMA8452_GPIO1chkBx->setGeometry(410, 565, 100, 20);

    MMA8452_GPIO2chkBx = new QCheckBox("GPIO2",MMA8452_Tab2);
    MMA8452_GPIO2chkBx->setGeometry(410, 595, 100, 20);

    MMA8452_GPIO3chkBx = new QCheckBox("GPIO3",MMA8452_Tab2);
    MMA8452_GPIO3chkBx->setGeometry(410, 625, 100, 20);

    MMA8452_SaveTofilechkbx = new QCheckBox("Save Data",MMA8452_Tab2);
    MMA8452_SaveTofilechkbx->setGeometry(410, 655, 100, 20);

    //Create line series for X,Y,Z
    MMA8452_seriesX = new QLineSeries();
    MMA8452_seriesX->setName("X-axis");
    MMA8452_seriesX->setColor(QColor("Red"));
    MMA8452_seriesY = new QLineSeries();
    MMA8452_seriesY->setName("Y-axis");
    MMA8452_seriesY->setColor(QColor("Blue"));
    MMA8452_seriesZ = new QLineSeries();
    MMA8452_seriesZ->setName("Z-axis");
    MMA8452_seriesZ->setColor(QColor("Green"));

    //Accelerometer Chart
    MMA8452_chartAcc = new QChart();
    MMA8452_chartAcc->setGeometry(15, 15, 620, 450);
    MMA8452_chartAcc->setPlotAreaBackgroundVisible(true);

    //Create Accelerometer Chart View Widget
    MMA8452_chartViewAcc = new QChartView(MMA8452_chartAcc, MMA8452_Tab2);
    MMA8452_chartViewAcc->setGeometry(15, 15, 620, 450);

    //Acelerometer char axes
    QValueAxis *MMA8452_axisX1 = new QValueAxis;
    MMA8452_axisX1->setRange(MMA8452_x1RangeMin, MMA8452_x1RangeMax);
    MMA8452_axisX1->setTitleText("Time");

    //QValueAxis *MMA8452_axisY1 = new QValueAxis;
    MMA8452_axisY1 = new QValueAxis;
    MMA8452_axisY1->setRange(MMA8452_y1RangeMin, MMA8452_y1RangeMax);
    MMA8452_axisY1->setTitleText("Acceleration (g)");

    //Add series to Accelerometer char
    MMA8452_chartAcc->addAxis(MMA8452_axisX1, Qt::AlignBottom);
    MMA8452_chartAcc->addAxis(MMA8452_axisY1, Qt::AlignLeft);
    MMA8452_chartAcc->addSeries(MMA8452_seriesX);
    MMA8452_chartAcc->addSeries(MMA8452_seriesY);
    MMA8452_chartAcc->addSeries(MMA8452_seriesZ);

    MMA8452_seriesX->attachAxis(MMA8452_axisX1);
    MMA8452_seriesX->attachAxis(MMA8452_axisY1);
    MMA8452_seriesY->attachAxis(MMA8452_axisX1);
    MMA8452_seriesY->attachAxis(MMA8452_axisY1);
    MMA8452_seriesZ->attachAxis(MMA8452_axisX1);
    MMA8452_seriesZ->attachAxis(MMA8452_axisY1);
    MMA8452_chartAcc->setPlotAreaBackgroundVisible(true);
    MMA8452_chartAcc->setBackgroundVisible(false);

    //MMA8452_chart->legend()->hide();
    MMA8452_chartAcc->legend()->show();
    MMA8452_chartAcc->setTitle("MMA8452 Accelerometer");

    //plotting timer and connect it to real-time slot
    MMA8452_timer = new QTimer(&w);
    QObject::connect(MMA8452_timer, SIGNAL(timeout()), &w, SLOT(MMA8452_updateChart()));
    /*******************************************************************************************/
    //MPU9250 Widgets and charts
    //Create line series for aX,aY,aZ
    MPU9250_seriesX = new QLineSeries();
    MPU9250_seriesX->setName("X-axis");
    MPU9250_seriesX->setColor(QColor("Red"));
    MPU9250_seriesY = new QLineSeries();
    MPU9250_seriesY->setName("Y-axis");
    MPU9250_seriesY->setColor(QColor("Blue"));
    MPU9250_seriesZ = new QLineSeries();
    MPU9250_seriesZ->setName("Z-axis");
    MPU9250_seriesZ->setColor(QColor("Green"));

    //Accelerometer Chart
    MPU9250_chartAcc = new QChart();
    MPU9250_chartAcc->legend()->show();

    //Acelerometer char axes
    QValueAxis *MPU9250_axisXAcc = new QValueAxis;
    MPU9250_axisXAcc->setRange(MPU9250_x1RangeMin, MPU9250_x1RangeMax);
    MPU9250_axisXAcc->setTitleText("Time");
    MPU9250_axisYAcc = new QValueAxis;
    MPU9250_axisYAcc->setRange(MPU9250_y1RangeMin, MPU9250_y1RangeMax);
    MPU9250_axisYAcc->setTitleText("Acceleration (g)");

    MPU9250_chartAcc->addAxis(MPU9250_axisXAcc, Qt::AlignBottom);
    MPU9250_chartAcc->addAxis(MPU9250_axisYAcc, Qt::AlignLeft);
    MPU9250_chartAcc->addSeries(MPU9250_seriesX);
    MPU9250_chartAcc->addSeries(MPU9250_seriesY);
    MPU9250_chartAcc->addSeries(MPU9250_seriesZ);

    MPU9250_seriesX->attachAxis(MPU9250_axisXAcc);
    MPU9250_seriesX->attachAxis(MPU9250_axisYAcc);
    MPU9250_seriesY->attachAxis(MPU9250_axisXAcc);
    MPU9250_seriesY->attachAxis(MPU9250_axisYAcc);
    MPU9250_seriesZ->attachAxis(MPU9250_axisXAcc);
    MPU9250_seriesZ->attachAxis(MPU9250_axisYAcc);

    //MPU9250_chartAcc->createDefaultAxes();         //comment this line for dynamic axes
    MPU9250_chartAcc->setTitle("MPU9250 Accelerometer Reading");
    MPU9250_chartAcc->setPlotAreaBackgroundVisible(true);
    MPU9250_chartAcc->setBackgroundVisible(false);

    //Create Accelerometer Chart View Widget
    MPU9250_chartViewAcc = new QChartView(MPU9250_chartAcc, MPU9250_Tab1);
    MPU9250_chartViewAcc->setGeometry(15, 15, 620, 450);
    /********************************************************************/
    //MPU9250 Gyroscope chart
    //Create line series for gX,gY,gZ
    MPU9250_seriesYaw = new QLineSeries();
    MPU9250_seriesYaw->setName("Yaw");
    MPU9250_seriesYaw->setColor(QColor("Red"));
    MPU9250_seriesPitch = new QLineSeries();
    MPU9250_seriesPitch->setName("Pitch");
    MPU9250_seriesPitch->setColor(QColor("Blue"));
    MPU9250_seriesRoll = new QLineSeries();
    MPU9250_seriesRoll->setName("Roll");
    MPU9250_seriesRoll->setColor(QColor("Green"));

    //Gyroscope Chart things
    MPU9250_chartGyro = new QChart();
    MPU9250_chartGyro->legend()->show();

    //gyroscope char axes
    QValueAxis *MPU9250_axisXGyro = new QValueAxis;
    MPU9250_axisXGyro->setRange(MPU9250_x1RangeMin, MPU9250_x1RangeMax);
    MPU9250_axisXGyro->setTitleText("Time");

    //QValueAxis *axisY1 = new QValueAxis;
    MPU9250_axisYGyro = new QValueAxis;
    MPU9250_axisYGyro->setRange(MPU9250_yGyroRangeMin, MPU9250_yGyroRangeMax);
    MPU9250_axisYGyro->setTitleText("Gyration (°/sec)");

    MPU9250_chartGyro->addAxis(MPU9250_axisXGyro, Qt::AlignBottom);
    MPU9250_chartGyro->addAxis(MPU9250_axisYGyro, Qt::AlignLeft);
    MPU9250_chartGyro->addSeries(MPU9250_seriesYaw);
    MPU9250_chartGyro->addSeries(MPU9250_seriesPitch);
    MPU9250_chartGyro->addSeries(MPU9250_seriesRoll);

    //Attach axis
    MPU9250_seriesYaw->attachAxis(MPU9250_axisXGyro);
    MPU9250_seriesYaw->attachAxis(MPU9250_axisYGyro);
    MPU9250_seriesPitch->attachAxis(MPU9250_axisXGyro);
    MPU9250_seriesPitch->attachAxis(MPU9250_axisYGyro);
    MPU9250_seriesRoll->attachAxis(MPU9250_axisXGyro);
    MPU9250_seriesRoll->attachAxis(MPU9250_axisYGyro);

    MPU9250_chartGyro->setTitle("MPU9250 Gyroscope Reading");
    MPU9250_chartGyro->setPlotAreaBackgroundVisible(true);
    MPU9250_chartGyro->setBackgroundVisible(false);

    //Create gyroscope Chart View Widget
    MPU9250_chartViewGyro = new QChartView(MPU9250_chartGyro, MPU9250_Tab1);
    MPU9250_chartViewGyro->setGeometry(650, 15, 620, 450);
    /*********************************************************************************/
    //Create line series for aX,aY,aZ
    MPU9250_seriesMx = new QLineSeries();
    MPU9250_seriesMx->setName("Mag. X");
    MPU9250_seriesMx->setColor(QColor("Red"));
    MPU9250_seriesMy = new QLineSeries();
    MPU9250_seriesMy->setName("Mag. Y");
    MPU9250_seriesMy->setColor(QColor("Blue"));
    MPU9250_seriesMz = new QLineSeries();
    MPU9250_seriesMz->setName("Mag. Z");
    MPU9250_seriesMz->setColor(QColor("Green"));

    //Magnetometer Chart
    MPU9250_chartMagneto = new QChart();
    MPU9250_chartMagneto->legend()->show();

    //magnetometer axes
    QValueAxis *MPU9250_axisXMagneto = new QValueAxis;
    MPU9250_axisXMagneto->setRange(MPU9250_xMagnetoRangeMin, MPU9250_xMagnetoRangeMax);
    MPU9250_axisXMagneto->setTitleText("Time");
    MPU9250_axisYMagneto = new QValueAxis;
    MPU9250_axisYMagneto->setRange(MPU9250_yMagnetoRangeMin, MPU9250_yMagnetoRangeMax);
    MPU9250_axisYMagneto->setTitleText("Magnetic Field (μT)");

    MPU9250_chartMagneto->addAxis(MPU9250_axisXMagneto, Qt::AlignBottom);
    MPU9250_chartMagneto->addAxis(MPU9250_axisYMagneto, Qt::AlignLeft);
    MPU9250_chartMagneto->addSeries(MPU9250_seriesMx);
    MPU9250_chartMagneto->addSeries(MPU9250_seriesMy);
    MPU9250_chartMagneto->addSeries(MPU9250_seriesMz);

    MPU9250_seriesMx->attachAxis(MPU9250_axisXMagneto);
    MPU9250_seriesMx->attachAxis(MPU9250_axisYMagneto);
    MPU9250_seriesMy->attachAxis(MPU9250_axisXMagneto);
    MPU9250_seriesMy->attachAxis(MPU9250_axisYMagneto);
    MPU9250_seriesMz->attachAxis(MPU9250_axisXMagneto);
    MPU9250_seriesMz->attachAxis(MPU9250_axisYMagneto);

    MPU9250_chartMagneto->setTitle("MPU9250 Magnetometer Reading");
    MPU9250_chartMagneto->setPlotAreaBackgroundVisible(true);
    MPU9250_chartMagneto->setBackgroundVisible(false);

    //Create Accelerometer Chart View Widget
    MPU9250_chartViewMagneto = new QChartView(MPU9250_chartMagneto, MPU9250_Tab1);
    MPU9250_chartViewMagneto->setGeometry(660, 435, 620, 450);

    //MPU9250 widgets
    //Label 1 - MCP2221 info
    QLabel *MPU9250_Label1 = new QLabel(MPU9250_Tab1);
    MPU9250_Label1->setGeometry(15, 460, 150, 25);
    MPU9250_Label1->setText("Device Configuration info");

    //MPU9250_TextBox for device information
    MPU9250_TextBox = new QTextEdit(MPU9250_Tab1);
    MPU9250_TextBox->setGeometry(15, 485, 380, 150);
    MPU9250_TextBox->acceptRichText();
    MPU9250_TextBox->setAcceptRichText(true);

    //MPU9250 checkbox for GPIO - output only
    QSignalMapper *MPU9250_CheckBoxSignalMapper = new QSignalMapper(MPU9250_Tab1);
    MPU9250_GPIO0chkBx = new QCheckBox("GPIO0",MPU9250_Tab1);
    MPU9250_GPIO0chkBx->setGeometry(410, 485, 100, 20);
    MPU9250_GPIO1chkBx = new QCheckBox("GPIO1",MPU9250_Tab1);
    MPU9250_GPIO1chkBx->setGeometry(410, 515, 100, 20);
    MPU9250_GPIO2chkBx = new QCheckBox("GPIO2",MPU9250_Tab1);
    MPU9250_GPIO2chkBx->setGeometry(410, 545, 100, 20);
    MPU9250_GPIO3chkBx = new QCheckBox("GPIO3",MPU9250_Tab1);
    MPU9250_GPIO3chkBx->setGeometry(410, 575, 100, 20);
    MPU9250_SaveTofilechkbx = new QCheckBox("Save Data to File", MPU9250_Tab1);
    MPU9250_SaveTofilechkbx->setGeometry(410, 605, 100, 20);
    /********************************************************************/
    //MPU9250 Accelerometer fullscale range
    QLabel *MPU9250_AccRangeLabel = new QLabel(MPU9250_Tab1);
    MPU9250_AccRangeLabel->setGeometry(15, 650, 275, 35);
    MPU9250_AccRangeLabel->setText("Select Accelerometer Fullscale Range:");

    //Accelerometer Drop-down list
    MPU9250_AccFscaleDropList = new QComboBox(MPU9250_Tab1);
    MPU9250_AccFscaleDropList->setGeometry(15, 680, 125, 35);
    MPU9250_AccFscaleDropList->addItem("±2g");
    MPU9250_AccFscaleDropList->addItem("±4g");
    MPU9250_AccFscaleDropList->addItem("±8g");
    MPU9250_AccFscaleDropList->addItem("±16g");

    //MPU9250 Gyroscope fullscale range
    QLabel *MPU9250_GyroRangeLabel = new QLabel(MPU9250_Tab1);
    MPU9250_GyroRangeLabel->setGeometry(15, 725, 250, 35);
    MPU9250_GyroRangeLabel->setText("Select Gyroscope Fullscale Range:");

    //MPU9250 Gyroscope Drop-down list
    MPU9250_GyroFscaleDropList = new QComboBox(MPU9250_Tab1);
    MPU9250_GyroFscaleDropList->setGeometry(15, 755, 125, 35);
    MPU9250_GyroFscaleDropList->addItem("±250°/sec");
    MPU9250_GyroFscaleDropList->addItem("±500°/sec");
    MPU9250_GyroFscaleDropList->addItem("±1000°/sec");
    MPU9250_GyroFscaleDropList->addItem("±2000°/sec");

    //plotting timer and connect it to real-time slot
    MPU9250_timer = new QTimer(&w);
    QObject::connect(MPU9250_timer, SIGNAL(timeout()), &w, SLOT(MPU9250_updateChart()));
    /************************************************************************************************/
    //GPS widgets
    //Label 1 - MCP2221 info
    QLabel *Neo6GPS_Label1 = new QLabel(MMA8452_Tab2);
    Neo6GPS_Label1->setGeometry(700, 670, 150, 25);
    Neo6GPS_Label1->setText("Location Information");

    Neo6GPS_TextBox = new QTextEdit(MMA8452_Tab2);
    Neo6GPS_TextBox->setGeometry(700, 700, 380, 150);
    Neo6GPS_TextBox->acceptRichText();
    Neo6GPS_TextBox->setAcceptRichText(true);

    QPushButton *GPS_Button = new QPushButton(MMA8452_Tab2);
    GPS_Button->setGeometry(1100, 800, 100, 35);
    GPS_Button->setText("Read GPS");

    QPushButton *GPS_ConnectButton = new QPushButton(MMA8452_Tab2);
    GPS_ConnectButton->setGeometry(1100, 750, 100, 35);
    GPS_ConnectButton->setText("Connect");

    serial_droplist = new QComboBox(MMA8452_Tab2);
    serial_droplist->setGeometry(1100, 700, 100, 35);

    //List all available serial ports and add them to a drop-down list
    QList<QSerialPortInfo> list;
    list = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : list)
    {
        serial_droplist->addItem(info.portName());
    }
    m_map_control = new QMapControl(QSizeF(480.0, 640.0), MMA8452_Tab2);
    m_map_control->setGeometry(700, 30, 480, 640);

    // Create/add a layer with the default OSM map adapter.
    m_map_control->addLayer(std::make_shared<LayerMapAdapter>("Custom Layer", std::make_shared<MapAdapterOSM>()));

    serial = new QSerialPort(MMA8452_Tab2);
    serial->setPortName(serial_droplist->currentText());
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadOnly);

    //location
    location = new QGeoPositionInfo(Coordinate, timestamp);

    //Coordinate = new QGeoCoordinate(longitude, latitude);
    GPSdevice = new QNmeaPositionInfoSource(QNmeaPositionInfoSource::RealTimeMode); //(QIODevice::ReadOnly);
    GPSdevice->setPreferredPositioningMethods(QGeoPositionInfoSource::AllPositioningMethods);
    GPSdevice->setDevice(serial);

    //GPSDevice must be set in the main with a default device before changing it
    //to the desired serial device from the drop-down list, follow the callback functions
    //to understand the flow.
    GPSdevice->requestUpdate(500);
    GPSdevice->startUpdates();
    /***************************************************************************/
    //Analog Inputs widgets
    //Analog inputs textbox Label
    QLabel *AnalogIn_Label1 = new QLabel(AnalogIn_Tab3);
    AnalogIn_Label1->setGeometry(15, 550, 150, 25);
    AnalogIn_Label1->setText("Analog Reading");

    //AnalogIn_TextBox for device information and analog readings
    AnalogIn_TextBox = new QTextEdit(AnalogIn_Tab3);
    AnalogIn_TextBox->setGeometry(15, 575, 380, 150);
    AnalogIn_TextBox->acceptRichText();
    AnalogIn_TextBox->setAcceptRichText(true);

    //Analog Inputs checkboxes for AN channels - input only
    QSignalMapper *AnalogIn_CheckBoxSignalMapper = new QSignalMapper(AnalogIn_Tab3);
    AnalogIn_AN0chkBx = new QCheckBox("AN0",AnalogIn_Tab3);
    AnalogIn_AN0chkBx->setGeometry(410, 575, 100, 20);
    AnalogIn_AN1chkBx = new QCheckBox("AN1",AnalogIn_Tab3);
    AnalogIn_AN1chkBx->setGeometry(410, 605, 100, 20);
    AnalogIn_AN2chkBx = new QCheckBox("AN2",AnalogIn_Tab3);
    AnalogIn_AN2chkBx->setGeometry(410, 635, 100, 20);
    AnalogIn_SaveTofilechkbx = new QCheckBox("Save Data to File", AnalogIn_Tab3);
    AnalogIn_SaveTofilechkbx->setGeometry(410, 665, 100, 20);

    //Analog Inputs Widgets and charts
    //Create line series for AN0, AN1 and AN2
    AnalogIn_seriesX = new QLineSeries();
    AnalogIn_seriesX->setName("AN0");
    AnalogIn_seriesX->setColor(QColor("Red"));
    AnalogIn_seriesY = new QLineSeries();
    AnalogIn_seriesY->setName("AN1");
    AnalogIn_seriesY->setColor(QColor("Blue"));
    AnalogIn_seriesZ = new QLineSeries();
    AnalogIn_seriesZ->setName("AN2");
    AnalogIn_seriesZ->setColor(QColor("Green"));

    //Analog inputs Chart
    AnalogIn_chartAcc = new QChart();
    AnalogIn_chartAcc->legend()->show();

    //Acelerometer char axes
    QValueAxis *AnalogIn_axisXAcc = new QValueAxis;
    AnalogIn_axisXAcc->setRange(AnalogIn_x1RangeMin, AnalogIn_x1RangeMax);
    AnalogIn_axisXAcc->setTitleText("Time");
    AnalogIn_axisYAcc = new QValueAxis;
    AnalogIn_axisYAcc->setRange(AnalogIn_y1RangeMin, AnalogIn_y1RangeMax);
    AnalogIn_axisYAcc->setTitleText("Voltage (v)");

    AnalogIn_chartAcc->addAxis(AnalogIn_axisXAcc, Qt::AlignBottom);
    AnalogIn_chartAcc->addAxis(AnalogIn_axisYAcc, Qt::AlignLeft);
    AnalogIn_chartAcc->addSeries(AnalogIn_seriesX);
    AnalogIn_chartAcc->addSeries(AnalogIn_seriesY);
    AnalogIn_chartAcc->addSeries(AnalogIn_seriesZ);

    AnalogIn_seriesX->attachAxis(AnalogIn_axisXAcc);
    AnalogIn_seriesX->attachAxis(AnalogIn_axisYAcc);
    AnalogIn_seriesY->attachAxis(AnalogIn_axisXAcc);
    AnalogIn_seriesY->attachAxis(AnalogIn_axisYAcc);
    AnalogIn_seriesZ->attachAxis(AnalogIn_axisXAcc);
    AnalogIn_seriesZ->attachAxis(AnalogIn_axisYAcc);

    //AnalogIn_chartAcc->createDefaultAxes();         //comment this line for dynamic axes
    AnalogIn_chartAcc->setTitle("Analog Inputs Reading");
    AnalogIn_chartAcc->setPlotAreaBackgroundVisible(true);
    AnalogIn_chartAcc->setBackgroundVisible(false);

    //Create Accelerometer Chart View Widget
    AnalogIn_chartViewAcc = new QChartView(AnalogIn_chartAcc, AnalogIn_Tab3);
    AnalogIn_chartViewAcc->setGeometry(15, 15, 1250, 475);

    //plotting timer and connect it to real-time slot
    AnalogIn_timer = new QTimer(&w);
    QObject::connect(AnalogIn_timer, SIGNAL(timeout()), &w, SLOT(AnalogIn_updateChart()));
    /****************************************************************************/
    Mcp2221_config();        //Configure any connected MCP2221
    MMA8452_Config();
    int status = IMU.begin();       //start communication with MPU9250

    w.show();
    QObject::connect(MMA8452_RangeDropList, SIGNAL(currentIndexChanged(int)), &w, SLOT(MMA8452_DropBoxCallback(int)));
    QObject::connect(MMA8452_CheckBoxSignalMapper, SIGNAL(mapped(int)), &w, SLOT(MMA8452_ChkBxCallback(int)));
    MMA8452_CheckBoxSignalMapper->setMapping(MMA8452_GPIO0chkBx, 0);
    MMA8452_CheckBoxSignalMapper->setMapping(MMA8452_GPIO1chkBx, 1);
    MMA8452_CheckBoxSignalMapper->setMapping(MMA8452_GPIO2chkBx, 2);
    MMA8452_CheckBoxSignalMapper->setMapping(MMA8452_GPIO3chkBx, 3);
    MMA8452_CheckBoxSignalMapper->setMapping(MMA8452_SaveTofilechkbx, 4);
    QObject::connect(MMA8452_GPIO0chkBx, SIGNAL(stateChanged(int)), MMA8452_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MMA8452_GPIO1chkBx, SIGNAL(stateChanged(int)), MMA8452_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MMA8452_GPIO2chkBx, SIGNAL(stateChanged(int)), MMA8452_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MMA8452_GPIO3chkBx, SIGNAL(stateChanged(int)), MMA8452_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MMA8452_SaveTofilechkbx, SIGNAL(stateChanged(int)), MMA8452_CheckBoxSignalMapper, SLOT(map()));

    QObject::connect(MPU9250_AccFscaleDropList, SIGNAL(currentIndexChanged(int)), &w, SLOT(MPU9250_AccScale(int)));
    QObject::connect(MPU9250_GyroFscaleDropList, SIGNAL(currentIndexChanged(int)), &w, SLOT(MPU9250_GyroScale(int)));
    QObject::connect(MPU9250_CheckBoxSignalMapper, SIGNAL(mapped(int)), &w, SLOT(MPU9250_ChkBxCallback(int)));
    MPU9250_CheckBoxSignalMapper->setMapping(MPU9250_GPIO0chkBx, 0);
    MPU9250_CheckBoxSignalMapper->setMapping(MPU9250_GPIO1chkBx, 1);
    MPU9250_CheckBoxSignalMapper->setMapping(MPU9250_GPIO2chkBx, 2);
    MPU9250_CheckBoxSignalMapper->setMapping(MPU9250_GPIO3chkBx, 3);
    MPU9250_CheckBoxSignalMapper->setMapping(MPU9250_SaveTofilechkbx, 4);
    QObject::connect(MPU9250_GPIO0chkBx, SIGNAL(stateChanged(int)), MPU9250_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MPU9250_GPIO1chkBx, SIGNAL(stateChanged(int)), MPU9250_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MPU9250_GPIO2chkBx, SIGNAL(stateChanged(int)), MPU9250_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MPU9250_GPIO3chkBx, SIGNAL(stateChanged(int)), MPU9250_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(MPU9250_SaveTofilechkbx, SIGNAL(stateChanged(int)), MPU9250_CheckBoxSignalMapper, SLOT(map()));

    QObject::connect(GPS_Button, SIGNAL(clicked()), &w, SLOT(GetGPSinfo()));
    QObject::connect(GPS_ConnectButton, SIGNAL(clicked()), &w, SLOT(ConnectToCOM()));

    QObject::connect(AnalogIn_CheckBoxSignalMapper, SIGNAL(mapped(int)), &w, SLOT(AnalogIn_ChkBxCallback(int)));
    AnalogIn_CheckBoxSignalMapper->setMapping(AnalogIn_AN0chkBx, 0);
    AnalogIn_CheckBoxSignalMapper->setMapping(AnalogIn_AN1chkBx, 1);
    AnalogIn_CheckBoxSignalMapper->setMapping(AnalogIn_AN2chkBx, 2);
    AnalogIn_CheckBoxSignalMapper->setMapping(AnalogIn_SaveTofilechkbx, 3);
    QObject::connect(AnalogIn_AN0chkBx, SIGNAL(stateChanged(int)), AnalogIn_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(AnalogIn_AN1chkBx, SIGNAL(stateChanged(int)), AnalogIn_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(AnalogIn_AN2chkBx, SIGNAL(stateChanged(int)), AnalogIn_CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(AnalogIn_SaveTofilechkbx, SIGNAL(stateChanged(int)), AnalogIn_CheckBoxSignalMapper, SLOT(map()));
    return a.exec();
}
