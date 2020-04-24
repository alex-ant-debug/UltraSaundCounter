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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "console.h"
#include "settingsdialog.h"

#include <QLabel>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <string>
#include <cmath>
#include "QStandardItemModel"
#include "QStandardItem"

#define sizeADC_Result  251
#define numberOfPulses  1
#if numberOfPulses == 2
#define samplingStep    3.76// мкс.
#else
#define samplingStep    2.575// мкс.
#endif

bool flagClear = true;

//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_status(new QLabel),
    m_console(new Console),
    m_settings(new SettingsDialog),
//! [1]
    m_serial(new QSerialPort(this))
//! [1]
{
//! [0]
    flagSignal = 0;
    m_ui->setupUi(this);
    m_console->setEnabled(false);
    //setCentralWidget(m_console);

    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionQuit->setEnabled(true);
    m_ui->actionConfigure->setEnabled(true);

    m_ui->statusBar->addWidget(m_status);

    initActionsConnections();

    //------------------------------Variable Input------------------------------------------------------------------------------------

    m_ui->distanceBetweenSensors->setRange(100, 140); // растояние между датчиками 11-13 см
    m_ui->pipeDiameter->setRange(8.0, 12.0);             //диапазон расхождения диаметра трубы
    m_ui->soundSpeed->setRange(250, 500);               //скорость звука в среде
    m_ui->sensorDelay->setRange(0, 100);
    m_ui->noisefilter->setRange(0.0, 2.0);              //диапвзон фильтра

    QFile file(pathFile);
    if (!file.open(QIODevice::ReadOnly ))
    {
         m_ui->pipeDiameter->setValue(10.00);      //диаметр трубы
         m_ui->soundSpeed->setValue(330);
         m_ui->elementDelay->setValue(59.00);
         m_ui->distanceBetweenSensors->setValue(122);
        // error message
    }
    else
    {

        QTextStream stream(&file);
        QString dataWindow[5] = {"Delta_t = ", "tz = ", "d = ", "C = ", "L = "};
        QString line;

         do {
             unsigned int n = 0;
             line = stream.readLine();
             for(unsigned int index = 0; index < 5; index++)
             {
                 if((n = (unsigned int)line.indexOf(dataWindow[index])) != -1)
                 {
                     switch (index) {
                     case 0: {
                         double sensorDelay = std::stof(line.right(5).toStdString());
                         m_ui->sensorDelay->setValue(sensorDelay);
                         break;}
                     case 1: {
                         double elementDelay = std::stof(line.right(5).toStdString());
                         m_ui->elementDelay->setValue(elementDelay);
                         break;}
                     case 2: {
                         double pipe_diameter = std::stof(line.right(5).toStdString());
                         m_ui->pipeDiameter->setValue(pipe_diameter);
                         break;}
                     case 3: {
                         double soundSpeed = std::stof(line.right(6).toStdString());
                         m_ui->soundSpeed->setValue(soundSpeed);
                         break;}
                     case 4: {
                         double distanceBetweenSensors = std::stof(line.right(6).toStdString());
                         m_ui->distanceBetweenSensors->setValue(distanceBetweenSensors);
                         break;}


                     default:
                         break;
                     }
                 }
             }

         } while (!line.isNull());

        stream.flush();
        file.close();
    }

    //-----------------------------------------------------------------------------------------------------------------------------

    m_ui->spinBox_X1->setRange(0,  1290);
    m_ui->spinBox_X2->setRange(10, 1300);
    m_ui->spinBox_Y1->setRange(0,   900);
    m_ui->spinBox_Y2->setRange(10, 1023);

    m_ui->spinBox_X1->setValue(340);
    m_ui->spinBox_X2->setValue(440);
    m_ui->spinBox_Y1->setValue(300);
    m_ui->spinBox_Y2->setValue(800);


    m_ui->spinBox_X3->setRange(0, 390000);
    m_ui->spinBox_X4->setRange(0, 500);

    m_ui->doubleSpinBox_Y3->setRange(0, 2000);
    m_ui->doubleSpinBox_Y4->setRange(0, 2000);


    m_ui->doubleSpinBox_Y3->setValue(-2.5);
    m_ui->doubleSpinBox_Y4->setValue(2.5);

    //m_ui->spinBox_X3->setValue(0);
    m_ui->spinBox_X4->setValue(30);

    m_ui->noisefilter->setValue(0.25);  //значение фильтра по умолчанию

    plotting();
    //colorPoint(0, Qt::darkMagenta, graph1, graph2);//точки слежения за переходом через ноль
    passTimeAndZero();
    //-----------------------------------------------------------------------------------------------------------------------------
    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
//! [2]
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::readData);
//! [2]
    connect(m_console, &Console::getData, this, &MainWindow::writeData);
//! [3]
}
//! [3]

MainWindow::~MainWindow()
{
    QFile file(pathFile);
    if (!file.open(QIODevice::WriteOnly ))
    {
        // error message
    }
    else
    {

        QTextStream stream(&file);
        QString newLine = {'\n'};
        QString tabLine = {'\t'};
        QString Delta_t = QString::number((m_ui->sensorDelay->value()), 'f', 2);
        QString tz =  QString::number((m_ui->elementDelay->value()), 'f', 2);
        QString d =  QString::number((m_ui->pipeDiameter->value()), 'f', 2);
        QString C =  QString::number((m_ui->soundSpeed->value()), 'f', 2);
        QString L =  QString::number((m_ui->distanceBetweenSensors->value()), 'f', 2);

        QString writeToFile = "Delta_t = " + Delta_t + newLine + "tz = " + tz + newLine + "d = " + d + newLine + "C = " + C + newLine + "L = " + L + newLine;
        stream << writeToFile;

        stream.flush();
        file.close();
    }
    delete m_settings;
    delete m_ui;
}

//! [4]
void MainWindow::openSerialPort()
{
    const SettingsDialog::Settings p = m_settings->settings();
    m_serial->setPortName(p.name);
    m_serial->setBaudRate(p.baudRate);
    m_serial->setDataBits(p.dataBits);
    m_serial->setParity(p.parity);
    m_serial->setStopBits(p.stopBits);
    m_serial->setFlowControl(p.flowControl);
    if (m_serial->open(QIODevice::ReadWrite)) {
        m_console->setEnabled(true);
        m_console->setLocalEchoEnabled(p.localEchoEnabled);
        m_ui->actionConnect->setEnabled(false);
        m_ui->actionDisconnect->setEnabled(true);
        m_ui->actionConfigure->setEnabled(false);
        showStatusMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    }
    else
    {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());

        showStatusMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort()
{
    if (m_serial->isOpen())
    {
        m_serial->close();
    }
    m_console->setEnabled(false);
    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}
//! [5]

void MainWindow::about()
{
    QMessageBox::about(this, tr("About Simple Terminal"),
                       tr("The <b>Simple Terminal</b> example demonstrates how to "
                          "use the Qt Serial Port module in modern GUI applications "
                          "using Qt, with a menu bar, toolbars, and a status bar."));
}

//! [6]
void MainWindow::writeData(const QByteArray &data)
{
    m_serial->write(data);
}
//! [6]

//! [7]
void MainWindow::readData()
{

#if numberOfPulses == 2
    if((y1[0].size() >= sizeADC_Result)&&\
       (y1[1].size() >= sizeADC_Result)&&\
       (y1[2].size() >= sizeADC_Result)&&\
       (y1[3].size() >= sizeADC_Result)&&\
       (x1[0].size() >= sizeADC_Result)&&\
       (x1[1].size() >= sizeADC_Result))
    {
            y1[0].clear();
            y1[1].clear();
            y1[2].clear();
            y1[3].clear();
            x1[0].clear();
            x1[1].clear();
    }
#else
    if((y1[0].size() >= sizeADC_Result)&&(y1[1].size() >= sizeADC_Result))
    {
            y1[0].clear();
            y1[1].clear();
            x1[0].clear();
            x1[1].clear();
    }
#endif

    QByteArray data = m_serial->readAll();
    while(m_serial->waitForReadyRead(1000))
    {
        data += m_serial->readAll();
    }


    if(flagSignal <= (numberOfPulses+1))//3  заполняем y1 принятыми данными
    {
        QVector<double> y;
        y = convertChatToShort(data);
        if(numberOfPulses > 1)
        {
            if((y[0] < 20)&&((flagSignal == 0)||(flagSignal == 2)))//время выборки
            {
                y1[flagSignal] = y;
            }
            if((y[0] > 20)&&((flagSignal == 1)||(flagSignal == 3)))//значение АЦП
            {
                y1[flagSignal] = y;
            }
        }
        else
        {
            y1[flagSignal] = y;
        }


       if(y1[flagSignal].size() >= sizeADC_Result)
       {
        #if numberOfPulses == 2
           if(flagSignal < 4)
           {
               flagSignal++;
           }
           if(flagSignal >= 4)
           {
               flagSignal = 0;
           }
        #else
          if(flagSignal == 0)
          {
              flagSignal = 1;
          }
          else if(flagSignal == 1)
          {
              flagSignal = 0;
          }
        #endif
       }
    }

    #if numberOfPulses == 2
        if((y1[0].size() >= sizeADC_Result)&&\
           (y1[1].size() >= sizeADC_Result)&&\
           (y1[2].size() >= sizeADC_Result)&&\
           (y1[3].size() >= sizeADC_Result))
    #else
        if((y1[0].size() >= sizeADC_Result)&&\
           (y1[1].size() >= sizeADC_Result))
    #endif
    {
        samplingTimeArray();
        switchBetweenPoints = 0;
        double Time1Value[2];
        thirdPeakTimeSearch((numberOfPulses>1)?y1[1]:y1[0], Time1Value);


        QString Time1 = QString::number(Time1Value[0], 'f', 6);    //Время проходжения по потоку
        m_ui->Period_1->setText("t1_изм = " + Time1 + "(мкс)");

        QString Time1_1 = QString::number(Time1Value[1], 'f', 6);    //Время проходжения по потоку
        m_ui->Period_3->setText("t1_изм = " + Time1_1 + "(мкс)");
//------------------------------------------------------------------------------------------------------------------------
        switchBetweenPoints = 1;
        double Time2Value[2];
        thirdPeakTimeSearch((numberOfPulses>1)?y1[3]:y1[1], Time2Value);


        QString Time2 = QString::number(Time2Value[0], 'f', 6);    //Время проходжения против потока
        m_ui->Period_2->setText("t2_изм = " + Time2 + "(мкс)");

        QString Time2_2 = QString::number(Time2Value[1], 'f', 6);    //Время проходжения против потока
        m_ui->Period_4->setText("t2_изм = " + Time2_2 + "(мкс)");
//------------------------------------------------------------------------------------------------------------------------
        double t1p = Time1Value[0] - (m_ui->elementDelay->value()) - (m_ui->sensorDelay->value());//время распространение сигнала  по потоку

        QString Time_1P = QString::number(t1p, 'f', 6);
        m_ui->signalPropagationTime1->setText("t1p = " + Time_1P + "(мкс)");

        double t1p1 = Time1Value[1] - (m_ui->elementDelay->value()) - (m_ui->sensorDelay->value());//время распространение сигнала  по потоку

        QString Time_1P1 = QString::number(t1p1, 'f', 6);
        m_ui->signalPropagationTime1_2->setText("t1p = " + Time_1P1 + "(мкс)");
//------------------------------------------------------------------------------------------------------------------------
        double t2p = Time2Value[0] - (m_ui->elementDelay->value());                               //время распространение сигнала против потока
        QString Time_2P = QString::number(t2p, 'f', 6);
        m_ui->signalPropagationTime2->setText("t2p = " + Time_2P + "(мкс)");


        double t2p1 = Time2Value[1] - (m_ui->elementDelay->value());                               //время распространение сигнала против потока
        QString Time_2P1 = QString::number(t2p1, 'f', 6);
        m_ui->signalPropagationTime2_2->setText("t2p = " + Time_2P1 + "(мкс)");
//------------------------------------------------------------------------------------------------------------------------

        double Delta_tp = (t1p != 0)&&(t2p != 0) ? (t2p - t1p) : 0;                                                            //разность между времинами пролета сигнала
        QString DeltaTime = QString::number((Delta_tp), 'f', 9);
        m_ui->Time_difference->setText("Δtp = " + DeltaTime + "(мкс)");
        flightTimeDifference[0].push_back(Delta_tp);


        double noisefilter = m_ui->noisefilter->value();

        double timeDifference = (abs(t1p1 - t2p1) < 31) ? t2p1 - t1p1 : 0;
        double alpha = ((differenceBetweenPoints[0] - differenceBetweenPoints[1])*3.76)/((differenceBetweenPoints[0] > differenceBetweenPoints[1])?differenceBetweenPoints[0]:differenceBetweenPoints[1]);//timeDifference/abs(differenceBetweenPoints[0] - differenceBetweenPoints[1]);

        QString indexPoint1 = m_ui->Index_1->text();
        std::string str1 = indexPoint1.toStdString();
        char *indexString1 = &str1[10];
        double index1 = std::stod(indexString1);

        QString indexPoint2 = m_ui->Index_2->text();
        std::string str2 = indexPoint2.toStdString();
        char *indexString2 = &str2[10];
        double index2 = std::stod(indexString2);

        //(413.49 + alpha*delta)
        //timeDifference = (x1[0][index1] + alpha*delta[0]) - (x1[1][index2] + alpha*delta[1]);
        timeDifference = (x1[1][index2] - x1[0][index1]) + (delta[1] - delta[0]);// - alpha;

                                            //
        double Delta_tp1 = ((timeDifference>noisefilter)) ?  timeDifference: 0;        //&&((t2p1 - t1p1)<61)                                                    //разность между времинами пролета сигнала
        QString DeltaTime1 = QString::number((Delta_tp1), 'f', 9);
        m_ui->Time_difference_2->setText("Δtp = " + DeltaTime1 + "(мкс)");
        flightTimeDifference[1].push_back(Delta_tp1);
//------------------------------------------------------------------------------------------------------------------------

        double speed = (pow(m_ui->soundSpeed->value(), 2)*(Delta_tp*pow(10, (-6))))/(2*((m_ui->distanceBetweenSensors->value())*0.001));
        QString gasVolume = QString::number(speed, 'f', 9);
        m_ui->speed->setText("ν = " + gasVolume + "(м/с)");

        double speed1 = (pow(m_ui->soundSpeed->value(), 2)*(Delta_tp1*pow(10, (-6))))/(2*((m_ui->distanceBetweenSensors->value())*0.001));
        QString gasVolume1 = QString::number(speed1, 'f', 9);
        m_ui->speed_2->setText("ν = " + gasVolume1 + "(м/с)");

//------------------------------------------------------------------------------------------------------------------------

        double consumption = (speed * M_PI * pow((m_ui->pipeDiameter->value())*pow(10, (-3)), 2)*3600)/4;
        QString gasConsumption = QString::number(consumption, 'f', 9);
        m_ui->gas_consumption->setText("Q = " + gasConsumption + "(м³/ч)");

        double consumption1 = (speed1 * M_PI * pow((m_ui->pipeDiameter->value())*pow(10, (-3)), 2)*3600)/4;
        QString gasConsumption1 = QString::number(consumption1, 'f', 9);
        m_ui->gas_consumption_2->setText("Q = " + gasConsumption1 + "(м³/ч)");


        averageConsumption.push_back(consumption);
        char gasConsumptionSend[30] = {0};
        strcpy(gasConsumptionSend, "----Q = ");

        const char* ch = gasConsumption.toStdString().c_str();
        unsigned int indentation = 8;
        unsigned int sizeGasConsumptionSend = gasConsumption.size() + indentation;

        if(stopFlag)
        {
            std::strncpy(gasConsumptionSend, "Stop", 4);
        }

        for(unsigned int j = indentation; j < sizeGasConsumptionSend; j++)
        {
            gasConsumptionSend[j] = *(ch + (j-indentation));
        }

        gasConsumptionSend[sizeGasConsumptionSend] = '\n';
        gasConsumptionSend[sizeGasConsumptionSend+1] = '\r';

        m_serial->write(gasConsumptionSend, sizeGasConsumptionSend+2);

        for(unsigned int ii=0; ii<=(sizeGasConsumptionSend+1); ii++)
        {
            gasConsumptionSend[ii] = 0;
        }

//------------------------------------------------------------------------------------------------------------------------

        double distance = ((m_ui->distanceBetweenSensors->value())*0.001);//
        double speedInTheEnvironment = (distance/2)*(((t2p+t1p)*pow(10, (-6)))/((t1p*pow(10, (-6))*t2p*pow(10, (-6)))));
        QString soundSpeed = QString::number(speedInTheEnvironment, 'f', 3);
        m_ui->sound_speed->setText("C = " + soundSpeed + "(м/с)");

//------------------------------------------------------------------------------------------------------------------------

        double deltaV = consumption*2;          // ΔV = Q*t =(м³/ч)*ч = (м³)
        QString  deltaConsumption = QString::number(deltaV, 'f', 9);
        m_ui->DeltaV->setText("ΔV = " + deltaConsumption + "(м³)");
//------------------------------------------------------------------------------------------------------------------------


        Vs += deltaV;
        QString volumeSummation = QString::number(Vs, 'f', 9);
        m_ui->Vs->setText("Vs = " + volumeSummation + "(м³)");

        replotGraf();
        //passTimeAndZero();
        if(averagingFlag == 1)
        {
            calculationOfAverageConsumption();
        }

//        if(Delta_tp < -1)
//        {
//            stopFlag = true;//m_serial->write("Stop", 6);
//            m_ui->Start->setText("Start");
//            m_ui->Start->setStyleSheet(QString("color: black;"));
//        }
    }
}
//! [7]

//! [8]
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serial->errorString());
        closeSerialPort();
    }
}
//! [8]

void MainWindow::initActionsConnections()
{
    connect(m_ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(m_ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(m_ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(m_ui->actionConfigure, &QAction::triggered, m_settings, &SettingsDialog::show);
    connect(m_ui->actionClear, &QAction::triggered, this, &MainWindow::clear);
    connect(m_ui->actionAbout, &QAction::triggered, this, &MainWindow::about);
    connect(m_ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::showStatusMessage(const QString &message)
{
    m_status->setText(message);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
//    m_ui->signal->setGeometry(QRect(9,9, MainWindow::width()/2 - 2, MainWindow::height()/2 - 2));
//    m_ui->travelTime->setGeometry(QRect(MainWindow::width()/2 + 4,9, MainWindow::width()/2 - 2, MainWindow::height()/2 - 2));

//    QWidget::resizeEvent(event);
}


void MainWindow::on_Start_clicked()
{
    if (!m_serial->isOpen())
    {
        return;
    }

    flagClear = true;
    bool isTextStart = ((m_ui->Start->text()) == "Start");
    QString buttonColor = "color: ";

    const char *command = isTextStart ? "Start" : " "; //"Stop"  -> stopFlag
    QString buttonText = isTextStart ? "Stop": "Start";
    buttonColor += isTextStart ? "green;" : "black;";
    stopFlag = !isTextStart;

    m_serial->write(command, 6);
    m_ui->Start->setText(buttonText);
    m_ui->Start->setStyleSheet(QString(buttonColor));
}

void MainWindow::on_show_clicked()
{
//    if((y1[0].size() >= sizeADC_Result)&&(y1[1].size() >= sizeADC_Result))
//    {
//            plotting();
//    }
}

void MainWindow::on_SaveFile_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "Signal", tr("Text Files (*.in);;Text Files (*.txt)"));

        if (fileName != "")
        {
            QFile file(fileName);
            if (!file.open(QIODevice::WriteOnly))
            {
                // error message
            }
            else
            {

                QTextStream stream(&file);


                for(unsigned int j=0;j<sizeADC_Result;j++)
                {
                    QString newLine = {'\n'};
                    QString tabLine = {'\t'};
                    #if numberOfPulses == 2
                    QString decY1 = QString::number(y1[0][j]);
                    QString decY2 = QString::number(y1[1][j]);
                    QString decY3 = QString::number(y1[2][j]);
                    QString decY4 = QString::number(y1[3][j]);

                    decY1 += tabLine;
                    decY1 += decY2;
                    decY1 += tabLine;
                    decY1 += decY3;
                    decY1 += tabLine;
                    decY1 += decY4;
                    decY1 += newLine;
                    stream << decY1;
                    #else
                    QString decY1 = QString::number(y1[0][j]);
                    QString decY2 = QString::number(y1[1][j]);

                    decY1 += tabLine;
                    decY1 += decY2;
                    decY1 += newLine;
                    stream << decY1;
                    #endif
                }
                stream.flush();
                file.close();
            }
        }
}

unsigned int* MainWindow::zeroLineSearch(QVector <double> ADCData, unsigned int size, unsigned int* points)
{
  unsigned int min = 1024, max = 0;
  double valueAndQuantityPoints[20] = {0};
  unsigned short rangePoints = 0;                                                       //range points
  unsigned int j;
  unsigned int count = 0, threshold = 10;
  unsigned int midpoint = 0;
  unsigned sumPoints = 0;
  unsigned int straightRun = 5;            //прямой ход сигнала по металлу

  if(size > ADCData.size())
  {
      qDebug() << "zeroLineSearch";
      return 0;
  }

  //----------------------------search min and max point-----------------------------
  for(j=straightRun;j<size;j++)
  {
    if(ADCData.at(j) > max)
    {
      max = ADCData.at(j);
    }
    if(ADCData.at(j) < min)
    {
      min = ADCData.at(j);
    }
  }
  *(points+0) = max;
  *(points+2) = min;

  //------------------------count the quantity of identical points-------------------
  rangePoints = max - min;

  if(rangePoints < 20)
  {
    for(j=straightRun;j<size;j++)
    {
      valueAndQuantityPoints[(int)(max - ADCData.at(j))] += 1;
    }
    //-------------------------------------------------------------------------------
    for(j=0;j<20;j++)
    {
      if(*(valueAndQuantityPoints+j) > threshold)
      {
        sumPoints += ((max - j)*valueAndQuantityPoints[j]);
        count += valueAndQuantityPoints[j];
      }
    }
    midpoint = sumPoints/count;
  }
  else
  {
    midpoint = min + rangePoints/2;
  }
  //---------------------------------------------------------------------------------
  QString zeroSignal = QString::number(midpoint, 'f', 0);
  m_ui->zero->setText("zero = " + zeroSignal);
  *(points+1) = midpoint;

  return points;
}

void MainWindow::thirdPeakTimeSearch(QVector <double> ADCData,  double outputData[2])
{
    unsigned int max, min, countPeak = 2;
    unsigned int thirdPeakIndex[3] = {0}, average = 0;
    const unsigned short noiseFigure = 10;
    const int memorySize = ADCData.size();
    int index=0;

    if(!switchBetweenPoints)
    {
        zeroLineSearch(ADCData, 80, rangeLine);
    }
    max = rangeLine[0] + noiseFigure;                           //max zero interference
    average = rangeLine[1];                                     //zero
    min = rangeLine[2];                                         //min zero interference

    bool flagStartPeak = 0;
    bool flagEndingPeak = 0;

    //-----------------------------find three peak-------------------------------------------
    for(index = 50; index < memorySize-50; )
    {
        if(ADCData.at(index) >=  max)                       //find max
        {
            max = ADCData.at(index);
            flagStartPeak = 1;                           	//up
            flagEndingPeak = 0;
        }

        if((flagStartPeak == 1)&&(ADCData.at(index) < max))
        {
            flagStartPeak = 0;                           	//doun
            flagEndingPeak = 1;
            thirdPeakIndex[countPeak] = index;
            countPeak--;
        }

        if((flagStartPeak == 0)&&(flagEndingPeak == 1)&&(ADCData.at(index) < average))
        {
            flagStartPeak = 0;                           	//below zero
            flagEndingPeak = 0;
        }

        if(countPeak < 3)
        {
            index++;
        }
        else
        {
            break;
        }
    }

    //qDebug() <<"index = "<< index;
    index--; //((thirdPeakIndex[0] - 1)<0)? thirdPeakIndex[0]: thirdPeakIndex[0]-1;
    //qDebug() <<"index = "<< index;


    while(ADCData.at(index) > average)            // find the transition through zero 3 peak
    {
        index--;
    }

    //qDebug() <<"index = "<< index;

    float pointA = (index < memorySize)&&(index > 0)? ADCData.at(index)   : 0;                //pointA ниже нулевой линии
    float pointB = (index < memorySize)&&(index > 0)? ADCData.at(index+1) : 0;              //pointB выше нулевой линии

    printPointA_B(index, ADCData);

    outputData[0] = calculationTimeThirdPeak(pointA, pointB, index, average);
    outputData[1] = calculationTimeThirdPeakOther(pointA, pointB, index, average);
}


void MainWindow::plotting(void)
{
    int N=sizeADC_Result; //Вычисляем количество точек, которые будем отрисовывать
    QVector<double> x(N), y(N);

    for (int i=0; i<N; ++i)
    {
      x[i] = 0;
      y[i] = 0;
    }

    m_ui->signal->clearGraphs();//Если нужно, но очищаем все графики
    //Добавляем один график в signal
    m_ui->signal->addGraph();
    m_ui->signal->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    m_ui->signal->graph(0)->setPen(QPen(Qt::red)); // красный цвет линии для первого графика
    graph1 = m_ui->signal->graph(0);


    m_ui->signal->addGraph();
    m_ui->signal->graph(1)->setPen(QPen(Qt::blue)); // синий цвет линии для второго графика
    graph2 = m_ui->signal->graph(1);

    //Говорим, что отрисовать нужно график по нашим двум массивам x и y
    graph1->setData(x, y, 1);
    graph1->setLineStyle(QCPGraph::lsLine);
    graph1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    graph2->setData(x, y, 1);
    graph2->setLineStyle(QCPGraph::lsLine);
    graph2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));

    m_ui->signal->setInteraction(QCP::iRangeZoom, true);
    m_ui->signal->setInteraction(QCP::iRangeDrag, true);
    m_ui->signal->setInteraction(QCP::iSelectPlottables, true);
    //Подписываем оси Ox и Oy
    m_ui->signal->xAxis->setLabel("Время пролета, мкс");
    m_ui->signal->yAxis->setLabel("Данные АЦП");

    m_ui->signal->xAxis2->setVisible(true);
    m_ui->signal->xAxis2->setTickLabels(false);
    m_ui->signal->yAxis2->setVisible(true);
    m_ui->signal->yAxis2->setTickLabels(false);

    // make left and bottom axes always transfer their ranges to right and top axes:
    connect(m_ui->signal->xAxis, SIGNAL(rangeChanged(QCPRange)), m_ui->signal->xAxis2, SLOT(setRange(QCPRange)));
    connect(m_ui->signal->yAxis, SIGNAL(rangeChanged(QCPRange)), m_ui->signal->yAxis2, SLOT(setRange(QCPRange)));


    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    graph1->rescaleAxes(true);
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    graph2->rescaleAxes(true);

    arrow = new QCPItemCurve(m_ui->signal);

    //Установим область, которая будет показываться на графике
    m_ui->signal->xAxis->setRange(m_ui->spinBox_X1->value(), m_ui->spinBox_X2->value());//Для оси Ox
    m_ui->signal->yAxis->setRange(m_ui->spinBox_Y1->value(), m_ui->spinBox_Y2->value());//Для оси Oy//minY, maxY

    //И перерисуем график на нашем signal
    m_ui->signal->replot();
    //colorPoint(0, Qt::green, graph1, graph2);//точки слежения за переходом через ноль
}

void MainWindow::passTimeAndZero(void)
{
    unsigned int N = flightTimeDifference[0].size();
    QVector<double> x(N), y(N);

    for (unsigned int i=0; i<N; ++i)
    {
      x[i] = 0;
      y[i] = 0;
    }

    m_ui->travelTime->clearGraphs();//Если нужно, но очищаем все графики
    //Добавляем один график в signal
    m_ui->travelTime->addGraph();
    m_ui->travelTime->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
    m_ui->travelTime->graph(0)->setPen(QPen(Qt::red)); // line color red for second graph
    graph3 = m_ui->travelTime->graph(0);

    m_ui->travelTime->addGraph();
    m_ui->travelTime->graph(1)->setPen(QPen(Qt::blue)); // line color red for second graph
    graph4 = m_ui->travelTime->graph(1);

    //Говорим, что отрисовать нужно график по нашим двум массивам x и y
    graph3->setData(x, y, 1);//flightTimeDifference[0]
    graph3->setLineStyle(QCPGraph::lsLine);
    graph3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));

    graph4->setData(x, y, 1);//flightTimeDifference[1]
    graph4->setLineStyle(QCPGraph::lsLine);
    graph4->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));

    m_ui->travelTime->setInteraction(QCP::iRangeZoom, true);
    m_ui->travelTime->setInteraction(QCP::iRangeDrag, true);
    m_ui->travelTime->setInteraction(QCP::iSelectPlottables, true);
    //Подписываем оси Ox и Oy
    m_ui->travelTime->xAxis->setLabel("Полное время измерения, с.");
    m_ui->travelTime->yAxis->setLabel("Разность времени пролетов Δtp, мкс.");

    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)

    m_ui->travelTime->xAxis2->setVisible(true);
    m_ui->travelTime->xAxis2->setTickLabels(false);
    m_ui->travelTime->yAxis2->setVisible(true);
    m_ui->travelTime->yAxis2->setTickLabels(false);

    // make left and bottom axes always transfer their ranges to right and top axes:
    connect(m_ui->travelTime->xAxis, SIGNAL(rangeChanged(QCPRange)), m_ui->travelTime->xAxis2, SLOT(setRange(QCPRange)));
    connect(m_ui->travelTime->yAxis, SIGNAL(rangeChanged(QCPRange)), m_ui->travelTime->yAxis2, SLOT(setRange(QCPRange)));

    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    graph3->rescaleAxes(true);
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    graph4->rescaleAxes(true);


    //Установим область, которая будет показываться на графике
    m_ui->travelTime->xAxis2->setRange(m_ui->spinBox_X3->value(), m_ui->spinBox_X4->value());//Для оси Ox
    m_ui->travelTime->yAxis2->setRange(m_ui->doubleSpinBox_Y3->value(), m_ui->doubleSpinBox_Y4->value());//Для оси Oy//minY, maxY

    //И перерисуем график на нашем signal
    m_ui->travelTime->replot();
}

void MainWindow::on_noFlowAveraging_clicked()
{
    bool isTextStart = ((m_ui->noFlowAveraging->text()) == "Усреднение\nбез потока");

    startMeasurement = averageConsumption.size();//среднее потребление
    averagingFlag = (isTextStart) ? true : false ;

    QString textText = (isTextStart) ? "Завершить\n усреднение" : "Усреднение\nбез потока";
    QString textColor = (isTextStart)? "color: green;" : "color: black;";


    m_ui->noFlowAveraging->setText(textText);
    m_ui->noFlowAveraging->setStyleSheet(textColor);
}

void  MainWindow::calculationOfAverageConsumption(void)
{
    double averagingNoFlow = 0;
    unsigned int sizeAveraging =  averageConsumption.size();

    for(unsigned int n = startMeasurement; n < sizeAveraging; n++)
    {
        averagingNoFlow += averageConsumption[n];
    }

    averagingNoFlow = (averagingNoFlow)/(sizeAveraging - startMeasurement);

    QString averaging  = QString::number((averagingNoFlow *1000), 'f', 6);
    m_ui->averaging->setText("Qcp = " + averaging + "(л)");
}

void MainWindow::clear(void)
{
    if(flagClear)
    {
        x1[0].clear();
        x1[1].clear();
        x3.clear();
        y1[0].clear();
        y1[1].clear();
        y1[2].clear();
        y1[3].clear();
        flightTimeDifference[0].clear();
        flightTimeDifference[1].clear();
        flagSignal = 0;
    //    m_ui->travelTime->clearGraphs();
    //    m_ui->signal->clearGraphs();
        //m_ui->signal->replot();
        //m_ui->travelTime->replot();
        //m_ui->signal->clearGraphs();//graph1 = 0;
        plotting();
        passTimeAndZero();
        m_serial->write("Clean\n\r", 7);//
        flagClear = false;
    }
}


void MainWindow::colorPoint(double indexP, Qt::GlobalColor color, QCPGraph *graph1, QCPGraph *graph2)
{
    firstPointTracer = new QCPItemTracer(m_ui->signal);
    firstPointTracer->setGraph(0);
    firstPointTracer->setGraph(graph1);
    firstPointTracer->setGraphKey(indexP);
    firstPointTracer->updatePosition();
    firstPointTracer->setInterpolating(true);
    firstPointTracer->setStyle(QCPItemTracer::tsPlus);//tsCircle
    firstPointTracer->setPen(QPen(QBrush(color), 2));////color
    firstPointTracer->setBrush(color);
    firstPointTracer->setSize(75);
    //----------------------------------------------------
    secondPointTracer = new QCPItemTracer(m_ui->signal);
    secondPointTracer->setGraph(0);
    secondPointTracer->setGraph(graph2);
    secondPointTracer->setGraphKey(indexP);
    secondPointTracer->updatePosition();
    secondPointTracer->setInterpolating(true);
    secondPointTracer->setStyle(QCPItemTracer::tsPlus);//tsCircle
    secondPointTracer->setPen(QPen(QBrush(Qt::yellow), 2));//Qt::darkBlue
    secondPointTracer->setBrush(color);
    secondPointTracer->setSize(75);
}

void MainWindow::addZeroLine(unsigned int zero)
{
    arrow->start->setCoords(0, zero);
    arrow->startDir->setCoords(0, zero);
    arrow->endDir->setCoords(x1[0].size()*samplingStep+5, zero);
    arrow->end->setCoords(x1[0].size()*samplingStep+5, zero);
}

void MainWindow::printPointA_B(unsigned int index, QVector <double> ADCData)
{
    if((unsigned int)ADCData.size() > index)
    {
        float pointA = ADCData[index];                //pointA ниже нулевой линии
        float pointB = ADCData[index+1];              //pointB выше нулевой линии

        QString amplitudePointA = QString::number(pointA, 'f', 0);
        QString amplitudePointB = QString::number(pointB, 'f', 0);

        differenceBetweenPoints[switchBetweenPoints] = pointB -pointA;

        if(!switchBetweenPoints)
        {
            QString Index_1 = QString::number(index, 'f', 0);
            m_ui->Index_1->setText("Index 1 = " + Index_1);
            m_ui->Point_A1->setText("Point_A1 = " + amplitudePointA);
            m_ui->Point_B1->setText("Point_B1 = " + amplitudePointB);
            QString adcData1 = QString::number(x1[switchBetweenPoints][index]);
            m_ui->label_3->setText("t1 = " + adcData1);
        }
        else
        {
            QString Index_2 = QString::number(index, 'f', 0);
            m_ui->Index_2->setText("Index 2 = " + Index_2);
            m_ui->Point_A2->setText("Point_A2 = " + amplitudePointA);
            m_ui->Point_B2->setText("Point_B2 = " + amplitudePointB);
            QString adcData2 = QString::number(x1[switchBetweenPoints][index]);
            m_ui->label_23->setText("t2 = " + adcData2);
        }
    }
}

double MainWindow::calculationTimeThirdPeak(float pointA, float pointB, unsigned int index, unsigned int average)// QVector <double> ADCData)
{
    unsigned int firstIndex = 0;

    double timeCrossingZerouLine;

    timeCrossingZerouLine = (average == pointA) ? ((pointA - firstIndex)*samplingStep) : (samplingStep*((index - firstIndex) + (average-pointA)/(pointB-pointA)));

    timeCrossingZerouLine = (average == pointB) ? ((pointB - firstIndex)*samplingStep) : (samplingStep*((index - firstIndex) + (average-pointA)/(pointB-pointA)));

    return timeCrossingZerouLine;
}

double MainWindow::calculationTimeThirdPeakOther(float pointA, float pointB, unsigned int index, unsigned int average)// QVector <double> ADCData)
{
    unsigned int firstIndex = 0;
    double timeCrossingZerouLine;

    oneSamplingStep = (x1[switchBetweenPoints][index+1] - x1[switchBetweenPoints][index]);//pointB - pointA

    delta[switchBetweenPoints] = oneSamplingStep*(average-pointA)/(pointB-pointA);

    timeCrossingZerouLine = (average == pointA) ? (x1[switchBetweenPoints][index] - firstIndex) : ((x1[switchBetweenPoints][index] - firstIndex) + (delta[switchBetweenPoints]));

    timeCrossingZerouLine = (average == pointB) ? (x1[switchBetweenPoints][index] - firstIndex) : ((x1[switchBetweenPoints][index] - firstIndex) + (delta[switchBetweenPoints]));

    return timeCrossingZerouLine;
}

void MainWindow::samplingTimeArray(void)
{
 //-----------------------------------Время выборки  сигнала-------------------------------------------
    if(numberOfPulses > 1)
    {
        for(unsigned int n = 0; n < 2; n++)
        {
            int sizeY1 = y1[n*2].size();

            for(int x = 0; x < sizeY1; x++)
            {
                x1[n].push_back( ((y1[n*2].at(x)<10)&&(y1[n*2].at(x) != 0)&&(y1[n*2].at(x)<20)) ? (20-y1[n*2].at(x)) : y1[n*2].at(x));
            }

            double data = 0;
            int sizeX1 = x1[n].size();
            for(int x = 0; x < sizeX1; x++)
            {
                data += x1[n][x];
                x1[n][x] = data*0.2685;                 //сложение и умножение на коэффициент
            }
        }
    }
    else
    {
        for(unsigned int n = 0; n < 2; n++)
        {
            int sizeY1 = y1[n].size();

            for(int x = 0; x < sizeY1; x++)
            {
                x1[n].push_back(x*2.575);
            }
        }
    }
}

QVector<double> MainWindow::convertChatToShort(QByteArray data)
{
    QVector<double> y;
    unsigned int sizeData =  data.size();

    for(unsigned int j=0;j<sizeData;j+=2)
    {
        unsigned short dataConvert = (unsigned char)data.at(j)*0x100 + (unsigned char)data.at(j+1);//преобразование char to short
        unsigned short simbol = (dataConvert < 1024) ? dataConvert : 0 ;//если приняли что-то не то
        y.push_back(simbol);
    }

    return y;
}

void MainWindow::replotGraf(void)
{
    //--------------------------------- Обновляем данные о сигнале ---------------------------------
    graph1->setData(x1[0], (numberOfPulses>1)? y1[1]: y1[0], 1);
    graph2->setData(x1[1], (numberOfPulses>1)? y1[3]: y1[1], 1);
    m_ui->signal->yAxis->setRange(m_ui->spinBox_Y1->value(), m_ui->spinBox_Y2->value());
    m_ui->signal->xAxis->setRange(m_ui->spinBox_X1->value(), m_ui->spinBox_X2->value());
    m_ui->signal->replot();


    unsigned int N = flightTimeDifference[0].size();
    if(N >= 10)
    {
        unsigned int X = x3.size();

        for (; X<N; X++)//Пробегаем по всем точкам
        {
            x3.push_back(X*2);
        }
        //Для показа границ по оси Oy сложнее, так как надо по правильному
        //вычислить минимальное и максимальное значение в векторах
        double minY = 0, maxY = 0;
        for (unsigned int i=1; i<N; i++)
        {
            if(flightTimeDifference[0][i]<minY) minY = flightTimeDifference[0][i];
            if(flightTimeDifference[0][i]>maxY) maxY = flightTimeDifference[0][i];
        }

        graph3->setData(x3, flightTimeDifference[0], 1);
        graph4->setData(x3, flightTimeDifference[1], 1);
        m_ui->travelTime->xAxis->setRange(0, N*2);
        m_ui->travelTime->yAxis->setRange(minY-5, maxY+5);
        m_ui->travelTime->replot();
    }
    //--------------------------------- Первая точка перехода через ноль ---------------------------------
//    QString indexPoint1 = m_ui->Period_3->text();
//    indexPoint1.truncate(indexPoint1.lastIndexOf(QChar('(')));
//    std::string str1 = indexPoint1.toStdString();
//    char *indexString1 = &str1[11];
//    double index1 = std::stod(indexString1);

//    firstPointTracer->setGraphKey(index1);
//    firstPointTracer->updatePosition();

//    //--------------------------------- Вторая точка перехода через ноль ---------------------------------
//    QString indexPoint2 = m_ui->Period_4->text();
//    indexPoint2.truncate(indexPoint2.lastIndexOf(QChar('(')));
//    std::string str2 = indexPoint2.toStdString();
//    char *indexString2 = &str2[11];
//    double index2 = std::stod(indexString2);

//    secondPointTracer->setGraphKey(index2);
//    secondPointTracer->updatePosition();

    //--------------------------------- Добавляем линию "нуля" ---------------------------------
//    QString zeroText = m_ui->zero->text();
//    str1 = zeroText.toStdString();
//    char *zeroString = &str1[7];
//    unsigned int zeroVolue = std::stoi(zeroString);
    addZeroLine(536);//zeroVolue
    //----------------------------------------------------------------------------------
    //И перерисуем график
    m_ui->signal->replot();
}
