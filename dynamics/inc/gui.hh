#ifndef GUI
#define GUI


#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include "WizualizacjaWahadla.hh"
#include "VehicleVisualization.hh"


class OknoGlowne: public QMainWindow{
Q_OBJECT

  WizualizacjaWahadla *W;
  VehicleVisualization *V;
  QPushButton *Op1;
  QPushButton *Op2;

public:

  OknoGlowne( QWidget *wRodzic = 0 );

public slots:

  void InitWahadlo();
  void InitVehicle();

};



#endif
