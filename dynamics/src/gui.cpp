#include "gui.hh"

OknoGlowne::OknoGlowne( QWidget *wRodzic ): QMainWindow( wRodzic ){

  resize(600,400);
  Op1 = new QPushButton( this );
  Op2 = new QPushButton( this );
  Op1->resize( 200,60 );
  Op2->resize( 200,60 );
  Op1->move( 100, 200 );
  Op2->move( 300, 200 );
  Op1->setText("Wahadlo");
  Op2->setText("Samochod");

  connect(Op1, SIGNAL(pressed()), this, SLOT(InitWahadlo()));
  connect(Op2, SIGNAL(pressed()), this, SLOT(InitVehicle()));

}

void OknoGlowne::InitWahadlo(){

  Op1->disconnect();
  Op2->disconnect();
  delete Op1;
  delete Op2;

  W = new WizualizacjaWahadla( this );

  setCentralWidget( W );

}

void OknoGlowne::InitVehicle(){

  Op1->disconnect();
  Op2->disconnect();
  delete Op1;
  delete Op2;

  V = new VehicleVisualization( this );

  setCentralWidget( V );

}

