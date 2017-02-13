/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "ui_main_window.h"
#include "../include/qtros/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace qtros {
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  ui.view->viewport()->installEventFilter(this);
  scene=new QGraphicsScene(this);
  ui.view->setScene(scene);
  qRegisterMetaType<string>("string");
  Quad1=new QuadItem(Qt::blue);
  Quad2=new QuadItem(Qt::red);
  Quad3=new QuadItem(Qt::black);
  scene->addItem(Quad1);
  scene->addItem(Quad2);
  scene->addItem(Quad3);

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode,SIGNAL(CallBackTrigger(float,float)),this,SLOT(MoveObject(float,float)));
  QObject::connect(&qnode,SIGNAL(CallBackTrigger2(float,float)),this,SLOT(MoveObject2(float,float)));
  QObject::connect(&qnode,SIGNAL(AltitudeSignal1(float)),this,SLOT(AltitudeSlot1(float)));
  QObject::connect(&qnode,SIGNAL(AltitudeSignal2(float)),this,SLOT(AltitudeSlot2(float)));
  QObject::connect(&qnode,SIGNAL(AltitudeSignal3(float)),this,SLOT(AltitudeSlot3(float)));
  QObject::connect(&qnode,SIGNAL(BatterySignal1(float,float)),this,SLOT(BatterySlot1(float,float)));
  QObject::connect(&qnode,SIGNAL(BatterySignal2(float,float)),this,SLOT(BatterySlot2(float,float)));
  QObject::connect(&qnode,SIGNAL(BatterySignal3(float,float)),this,SLOT(BatterySlot3(float,float)));
  QObject::connect(&qnode,SIGNAL(PositionSignal1(float,float,float)),this,SLOT(PositionSlot1(float,float,float)));
  QObject::connect(&qnode,SIGNAL(PositionSignal2(float,float,float)),this,SLOT(PositionSlot2(float,float,float)));
  QObject::connect(&qnode,SIGNAL(PositionSignal3(float,float,float)),this,SLOT(PositionSlot3(float,float,float)));
  QObject::connect(&qnode,SIGNAL(VelocitySignal1(float,float,float)),this,SLOT(VelocitySlot1(float,float,float)));
  QObject::connect(&qnode,SIGNAL(VelocitySignal2(float,float,float)),this,SLOT(VelocitySlot2(float,float,float)));
  QObject::connect(&qnode,SIGNAL(VelocitySignal3(float,float,float)),this,SLOT(VelocitySlot3(float,float,float)));
  QObject::connect(&qnode,SIGNAL(StateMachineSignal1(string,string)),this,SLOT(StateMachineSlot1(string,string)));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
//    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

//void MainWindow::showNoMasterMessage() {
//	QMessageBox msgBox;
//	msgBox.setText("Couldn't find the ros master.");
//	msgBox.exec();
//    close();
//}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check )
{
//  if ( ui.checkbox_use_environment->isChecked() )
//  {
//    if ( !qnode.init() )
//    {
//      showNoMasterMessage();
//    }
//    else
    {
      qnode.init();
			ui.button_connect->setEnabled(false);
		}
//  }
//  else
//  {
//		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//           ui.line_edit_host->text().toStdString()) )
//    {
//			showNoMasterMessage();
//    }
//    else
//    {
//			ui.button_connect->setEnabled(false);
//			ui.line_edit_master->setReadOnly(true);
//			ui.line_edit_host->setReadOnly(true);
//			ui.line_edit_topic->setReadOnly(true);
//		}
//	}
}


//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//	bool enabled;
//	if ( state == 0 ) {
//		enabled = true;
//	} else {
//		enabled = false;
//	}
//	ui.line_edit_master->setEnabled(enabled);
//	ui.line_edit_host->setEnabled(enabled);
//	//ui.line_edit_topic->setEnabled(enabled);
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

//void MainWindow::showButtonTestMessages()
//{
//  QMessageBox msgBox;
//  msgBox.setText("Hello ROS!!");
//  msgBox.exec();
//  close();
//}

//void MainWindow::on_button_test_clicked(bool check)
//{
//  showButtonTestMessages();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

//void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "qtros");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
//}

//void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "qtros");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

//}

void MainWindow::closeEvent(QCloseEvent *event)
{
//	WriteSettings();
	QMainWindow::closeEvent(event);
}

//void MainWindow::on_left_clicked()
//{
//  QStringList txt;
//  txt<<QString("moving left by 1 step<<<<<<\n");
//  QStringListModel *model = new QStringListModel(txt);
//  ui.view_logging->setModel(model);
// // ui.view->rotate(10);
//  super->moveBy(10,0);
//}

void MainWindow::MoveObject(float msg2,float msg3)
{
  static float buffer=msg3;
  Quad1->translate(8*msg2,0);
  Quad1->translate(15,0);
  Quad1->translate(0,15);
  Quad1->rotate(-msg3*180/(8*PI));
  Quad1->translate(-15,0);
  Quad1->translate(0,-15);
  scene->update();
}
void MainWindow::MoveObject2(float msg2,float msg3)
{
  static float buffer=msg2;
  Quad2->translate(8*msg2,0);
  Quad2->translate(15,0);
  Quad2->translate(0,15);
  Quad2->rotate(-msg3*180/(8*PI));
  Quad2->translate(-15,0);
  Quad2->translate(0,-15);
  scene->update();
}
void MainWindow::AltitudeSlot1(float alti)
{
  QString buffer;
  ui.AltitudeProgressBar1->setValue(alti);
  buffer.setNum(alti);
  ui.AltitudeLabel1->setText(buffer);
}
void MainWindow::AltitudeSlot2(float alti)
{
  QString buffer;
  ui.AltitudeProgressBar2->setValue(alti);
  buffer.setNum(alti);
  ui.AltitudeLabel2->setText(buffer);
}
void MainWindow::AltitudeSlot3(float alti)
{
  QString buffer;
  ui.AltitudeProgressBar3->setValue(alti);
  buffer.setNum(alti);
  ui.AltitudeLabel3->setText(buffer);
}
void MainWindow::BatterySlot1(float perc,float volt)
{
  QString buffer;
  buffer.setNum(volt);
  if(perc<0)
  {
    perc=-perc;
  }
  else;
  ui.BatteryProgressBar1->setValue(perc);
  ui.VoltageLabel1->setText(buffer);
}
void MainWindow::BatterySlot2(float perc,float volt)
{
  QString buffer;
  buffer.setNum(volt);
  if(perc<0)
  {
    perc=-perc;
  }
  else;
  ui.BatteryProgressBar2->setValue(perc);
  ui.VoltageLabel2->setText(buffer);
}
void MainWindow::BatterySlot3(float perc,float volt)
{
  QString buffer;
  buffer.setNum(volt);
  if(perc<0)
  {
    perc=-perc;
  }
  else;
  ui.BatteryProgressBar3->setValue(perc);
  ui.VoltageLabel3->setText(buffer);
}
void MainWindow::PositionSlot1(float x, float y, float z)
{
  QString bufferx,buffery,bufferz;
  bufferx.setNum(x);
  buffery.setNum(y);
  bufferz.setNum(z);
  ui.PositionLabelx1->setText(bufferx);
  ui.PositionLabely1->setText(buffery);
  ui.PositionLabelz1->setText(bufferz);
}
void MainWindow::PositionSlot2(float x, float y, float z)
{
  QString bufferx,buffery,bufferz;
  bufferx.setNum(x);
  buffery.setNum(y);
  bufferz.setNum(z);
  ui.PositionLabelx2->setText(bufferx);
  ui.PositionLabely2->setText(buffery);
  ui.PositionLabelz2->setText(bufferz);
}
void MainWindow::PositionSlot3(float x, float y, float z)
{
  QString bufferx,buffery,bufferz;
  bufferx.setNum(x);
  buffery.setNum(y);
  bufferz.setNum(z);
  ui.PositionLabelx3->setText(bufferx);
  ui.PositionLabely3->setText(buffery);
  ui.PositionLabelz3->setText(bufferz);
}
void MainWindow::VelocitySlot1(float vx,float vy,float vz)
{
  QString buffervx1,buffervy1,buffervz1;
  buffervx1.setNum(vx);
  buffervy1.setNum(vy);
  buffervz1.setNum(vz);
  ui.VelocityLabelx1->setText(buffervx1);
  ui.VelocityLabely1->setText(buffervy1);
  ui.VelocityLabelz1->setText(buffervz1);
}
void MainWindow::VelocitySlot2(float vx,float vy,float vz)
{
  QString buffervx2,buffervy2,buffervz2;
  buffervx2.setNum(vx);
  buffervy2.setNum(vy);
  buffervz2.setNum(vz);
  ui.VelocityLabelx2->setText(buffervx2);
  ui.VelocityLabely2->setText(buffervy2);
  ui.VelocityLabelz2->setText(buffervz2);
}
void MainWindow::VelocitySlot3(float vx,float vy,float vz)
{
  QString buffervx3,buffervy3,buffervz3;
  buffervx3.setNum(vx);
  buffervy3.setNum(vy);
  buffervz3.setNum(vz);
  ui.VelocityLabelx3->setText(buffervx3);
  ui.VelocityLabely3->setText(buffervy3);
  ui.VelocityLabelz3->setText(buffervz3);
}
void MainWindow::StateMachineSlot1(string state,string signal)
{
  QString qstate = QString::fromStdString(state);
  QString qsignal = QString::fromStdString(signal);
  ui.state_label1->setText(qstate);
  ui.signal_label1->setText(qsignal);
}
void qtros::MainWindow::on_Button_cvisionstart_clicked()
{
  QString buffer;
  float buffer1;
  cvisionprocess.start("rosrun turtlesim turtlesim_node");
  ros::NodeHandle nh("~");
  nh.getParam("/background_b",buffer1);
  buffer=buffer.setNum(buffer1);
  ui.cvisionoutput->setText(buffer);
}
void qtros::MainWindow::on_Button_cvisionstop_clicked()
{
  ui.cvisionoutput->clear();
  cvisionprocess.close();
}

void MainWindow::on_interruptbutton1_clicked()
{
  qnode.InterruptSlot1();
}
void MainWindow::on_resumebutton1_clicked()
{
  qnode.ResumeSlot1();
}

void MainWindow::wheelEvent(QWheelEvent *event)
{
  ui.view->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  float scaleFactor=1.15;

  if(event->delta()>0)
  {
    ui.view->scale(scaleFactor,scaleFactor);
//    super->scale(1/scaleFactor,1/scaleFactor);
//    super2->scale(1/scaleFactor,1/scaleFactor);
  }
  else
  {
    ui.view->scale(1/scaleFactor,1/scaleFactor);
//    super->scale(scaleFactor,scaleFactor);
//    super2->scale(scaleFactor,scaleFactor);
  }
}

//void MainWindow::mouseMoveEvent(QMouseEvent *event)
//{
//  ui.view->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
//  if(event->buttons() == Qt::MidButton)
//  {
//    super->translate(event->x()/100,event->y()/100);
//    super2->translate(event->x()/100,event->y()/100);
//    ui.view->translate(20,0);//translate(20,20);//event->x(),event->y());
//    ui.view->centerOn(event->x(),event->y());
//    super2->translate(-20,0);
//    ui.VelocityLabelx1->setText("123");
//  }
//}

void MainWindow::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton)
    {
        // Store original position.
        m_originX = event->x();
        m_originY = event->y();
        scene->update();
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent* event)
{
  if (event->buttons() == Qt::MiddleButton)
  {
    QPointF oldp = ui.view->mapToScene(m_originX, m_originY);
    QPointF newp = ui.view->mapToScene(event->pos());
    QPointF translation = newp - oldp;

    Quad1->moveBy(translation.x(), translation.y());
    Quad2->moveBy(translation.x(), translation.y());
    Quad3->moveBy(translation.x(), translation.y());

    m_originX = event->x();
    m_originY = event->y();
    scene->update();
  }
}

bool MainWindow::eventFilter(QObject *target, QEvent *event)
{
  if(target=ui.view->viewport())
  {
    switch (event->type())
    {
    case QEvent::MouseButtonPress:
    {
      QMouseEvent *mouseevent=static_cast<QMouseEvent*>(event);
      if(mouseevent->button()==Qt::MidButton)
      {
        m_originX = mouseevent->x();
        m_originY = mouseevent->y();
        scene->update();
        return true;
      }
    }
      break;
    case QEvent::MouseMove:
    {
      QMouseEvent *mouseevent=static_cast<QMouseEvent*>(event);
      if(mouseevent->buttons()==Qt::MidButton)
      {
        QPointF oldp = ui.view->mapToScene(m_originX, m_originY);
        QPointF newp = ui.view->mapToScene(mouseevent->pos());
        QPointF translation = newp - oldp;

        Quad1->moveBy(translation.x(), translation.y());
        Quad2->moveBy(translation.x(), translation.y());
        Quad3->moveBy(translation.x(), translation.y());

        m_originX = mouseevent->x();
        m_originY = mouseevent->y();
        scene->update();
        return true;
      }
    }
      break;
    default:
      break;
    }
  }
  return QObject::eventFilter(target, event);
}

QuadItem::QuadItem(QColor qc)//QGraphicsItem *parent)
{
  QGraphicsItem *parent =NULL;
  QPainter *painter;
  quadcolor=qc;
}
QRectF QuadItem::boundingRect() const
{
    return QRectF(0,0,30,30);
}

void QuadItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setBrush(Qt::transparent);
  painter->setPen(quadcolor);
  painter->drawEllipse(QPoint(-15,-15),12,12);
  painter->drawEllipse(QPoint(-15,15),12,12);
  painter->drawEllipse(QPoint(15,-15),12,12);
  painter->drawEllipse(QPoint(15,15),12,12);
  painter->drawLine(QPoint(-15,-15),QPoint(15,15));
  painter->drawLine(QPoint(-15,15),QPoint(15,-15));
}

}
