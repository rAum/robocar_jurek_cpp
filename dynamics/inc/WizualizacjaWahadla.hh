#ifndef WIZUALIZACJAWAHADLA
#define WIZUALIZACJAWAHADLA

#include <QPushButton>
#include <QPainter>
#include <QBrush>
#include <QPalette>
#include <QLineEdit>
#include <QString>
#include <QLineF>
#include <iostream>
#include "wahadlo.hh"

class WizualizacjaWahadla: public QWidget {
Q_OBJECT
  PendulumParameters param;
  QPushButton *B;
  Wahadlo *S;

public:

  WizualizacjaWahadla( QWidget *wRodzic = 0 );
  void paintEvent( QPaintEvent *event );

public slots:

  void fresh( PendulumParameters p );

};

#endif
