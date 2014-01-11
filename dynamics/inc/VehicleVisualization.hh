#ifndef VEHICLEVISUALIZATION
#define VEHICLEVISUALIZATION

#include <QPushButton>
#include <QPainter>
#include <QBrush>
#include <QPalette>
#include <QLineEdit>
#include <QString>
#include <QLineF>
#include <QKeyEvent>
#include <iostream>
#include "Vehicle.hh"

class VehicleVisualization: public QWidget {
Q_OBJECT
  VehicleParameters param;
  Vehicle *S;
  QKeyEvent *Wp;
  QKeyEvent *Sp;
  QKeyEvent *Ap;
  QKeyEvent *Dp;
  QKeyEvent *Wr;
  QKeyEvent *Sr;
  QKeyEvent *Ar;
  QKeyEvent *Dr;


public:

  VehicleVisualization( QWidget *wRodzic = 0 );
  void paintEvent( QPaintEvent *event );

protected:

  void keyPressEvent( QKeyEvent *event );
  void keyReleaseEvent( QKeyEvent *event );

public slots:

  void fresh( VehicleParameters p );

signals:

  void ApplyTorque();
  void ReleaseTorque();
  void ApplyLeft();
  void ReleaseLeft();
  void ApplyRight();
  void ReleaseRight();
  void ApplyBreak();
  void ReleaseBreak();

};

#endif
