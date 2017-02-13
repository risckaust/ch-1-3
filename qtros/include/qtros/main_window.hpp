/**
 * @file /include/qtros/main_window.hpp
 *
 * @brief Qt based gui for qtros.
 *
 * @date November 2010
 **/
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QGraphicsView>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QApplication>
#include <QtGui>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QTransform>
#include <QGraphicsEllipseItem>
#include <QWidget>
#include <QString>
#include <QProcess>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class QuadItem;
class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

//  void ReadSettings(); // Load up qt program settings at startup
//	void WriteSettings(); // Save qt program settings when closing
	void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_interruptbutton1_clicked();
  void on_resumebutton1_clicked();
//  void on_checkbox_use_environment_stateChanged(int state);
//  void on_button_test_clicked(bool check);
//  void showButtonTestMessages();
//  void on_left_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically
  void MoveObject(float,float);
  void MoveObject2(float,float);
  void AltitudeSlot1(float);
  void AltitudeSlot2(float);
  void AltitudeSlot3(float);
  void BatterySlot1(float,float);
  void BatterySlot2(float,float);
  void BatterySlot3(float,float);
  void PositionSlot1(float,float,float);
  void PositionSlot2(float,float,float);
  void PositionSlot3(float,float,float);
  void VelocitySlot1(float,float,float);
  void VelocitySlot2(float,float,float);
  void VelocitySlot3(float,float,float);
  void StateMachineSlot1(string,string);
  void on_Button_cvisionstart_clicked();
  void on_Button_cvisionstop_clicked();

protected:
  virtual void wheelEvent(QWheelEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseMoveEvent(QMouseEvent *event);
  bool eventFilter(QObject *target, QEvent *event);

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  QGraphicsScene * scene;
  QuadItem *Quad1, *Quad2, *Quad3;
  QProcess cvisionprocess;
  int m_originX;
  int m_originY;
};

class QuadItem : public QGraphicsItem
{
public:
    QuadItem(QColor);
protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;
private:
    QColor quadcolor;
};

}

#endif // qtros_MAIN_WINDOW_H
