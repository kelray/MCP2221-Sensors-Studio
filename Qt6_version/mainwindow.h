#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGeoPositionInfo>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void MMA8452_updateChart();
    void MMA8452_DropBoxCallback(int index);
    void MMA8452_ChkBxCallback(int index);
    void ConnectToCOM();
    void GetGPSinfo();
    void MPU9250_ChkBxCallback(int index);
    void MPU9250_updateChart();
    void MPU9250_GyroScale(int index);
    void MPU9250_AccScale(int index);
    void AnalogIn_ChkBxCallback(int index);
    void AnalogIn_updateChart();
    void ExitApp();
    void AboutApp();
    void ActiveTabs(int index);
};
#endif // MAINWINDOW_H
