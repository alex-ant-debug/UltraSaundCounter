/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <fstream>

#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QFileDialog>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class QLabel;

namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class Console;
class SettingsDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QVector<double> x1[2], x3, y1[4], ySignal[2]; //Массивы координат точек
    #if numberOfPulses > 1
        bool flagSignal;
    #else
        unsigned int flagSignal;
    #endif
    QVector<double> flightTimeDifference[2];
    QVector<double> averageConsumption;
    bool switchBetweenPoints;
    double Vs = 0;
    bool averagingFlag = 0;
    unsigned int rangeLine[3];
    int startMeasurement = 0;
    QString pathFile = "C://Qt//Examples//Qt-5.10.0//serialport//terminal//settings";
    bool stopFlag = 0;
    QCPGraph *graph1;
    QCPGraph *graph2;
    QCPGraph *graph3;
    QCPGraph *graph4;
    QCPItemCurve *arrow;
    double oneSamplingStep;
    QCPItemTracer *firstPointTracer;
    QCPItemTracer *secondPointTracer;
    double differenceBetweenPoints[2];
    double delta[2];


    void thirdPeakTimeSearch(QVector <double> ADCData, double outputData[2]);
    unsigned int *zeroLineSearch(QVector <double> ADCData, unsigned int size, unsigned int* points);
    void plotting(void);
    void passTimeAndZero(void);
    void colorPoint(double indexP, Qt::GlobalColor color, QCPGraph *graph1, QCPGraph *graph2);
    void addZeroLine(unsigned int zero);
    void printPointA_B(unsigned int index, QVector <double> ADCData);
    double calculationTimeThirdPeak(float pointA, float pointB, unsigned int index, unsigned int average);// QVector <double> ADCData);
    double calculationTimeThirdPeakOther(float pointA, float pointB, unsigned int index, unsigned int average);
    void samplingTimeArray(void);
    QVector<double> convertChatToShort(QByteArray data);
    void replotGraf(void);


private slots:
    void openSerialPort();
    void closeSerialPort();
    void about();
    void writeData(const QByteArray &data);
    void readData();

    void handleError(QSerialPort::SerialPortError error);

    void on_Start_clicked();


    void on_show_clicked();

    void on_SaveFile_clicked();

    void  calculationOfAverageConsumption(void);

    void on_noFlowAveraging_clicked();

    void clear(void);


private:
    void initActionsConnections();

private:
    void showStatusMessage(const QString &message);

    Ui::MainWindow *m_ui = nullptr;
    QLabel *m_status = nullptr;
    Console *m_console = nullptr;
    SettingsDialog *m_settings = nullptr;
    QSerialPort *m_serial = nullptr;
protected:
    void resizeEvent(QResizeEvent *event) override;
};

#endif // MAINWINDOW_H
