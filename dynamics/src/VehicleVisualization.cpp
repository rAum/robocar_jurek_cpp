#include"VehicleVisualization.hh"

#define STATES 7

using namespace std;

VehicleVisualization::VehicleVisualization( QWidget *wRodzic ): QWidget( wRodzic ){

  param.q[0] = 0; param.q[1] = 0; param.q[2] = 0; param.q[3] = 0;
  param.q[4] = 0; param.q[5] = 0; param.q[6] = 0;
  param.params[0] = 1; param.params[1] = 0.1; param.params[2] = 100;
  param.params[3] = 1; param.params[4] = 10; param.params[5] = 10;

  S = new Vehicle( this );

  setFocusPolicy(Qt::StrongFocus);

  Wp = new QKeyEvent( QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, tr("Up") );
  Wp->accept();
  Sp = new QKeyEvent( QEvent::KeyPress, Qt::Key_S, Qt::NoModifier, tr("Down") );
  Sp->accept();
  Ap = new QKeyEvent( QEvent::KeyPress, Qt::Key_A, Qt::NoModifier, tr("Left") );
  Ap->accept();
  Dp = new QKeyEvent( QEvent::KeyPress, Qt::Key_D, Qt::NoModifier, tr("Right") );
  Dp->accept();

  Wr = new QKeyEvent( QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier, tr("Up") );
  Wr->accept();
  Sr = new QKeyEvent( QEvent::KeyRelease, Qt::Key_S, Qt::NoModifier, tr("Down") );
  Sr->accept();
  Ar = new QKeyEvent( QEvent::KeyRelease, Qt::Key_A, Qt::NoModifier, tr("Left") );
  Ar->accept();
  Dr = new QKeyEvent( QEvent::KeyRelease, Qt::Key_D, Qt::NoModifier, tr("Right") );
  Dr->accept();


  connect(S, SIGNAL(UpdateStates(VehicleParameters)), this, SLOT(fresh(VehicleParameters)));
  connect(this, SIGNAL(ApplyTorque()), S, SLOT(ApplyTorque()));
  connect(this, SIGNAL(ReleaseTorque()), S, SLOT(ReleaseTorque()));
  connect(this, SIGNAL(ApplyLeft()), S, SLOT(ApplyLeft()));
  connect(this, SIGNAL(ReleaseLeft()), S, SLOT(ReleaseLeft()));
  connect(this, SIGNAL(ApplyRight()), S, SLOT(ApplyRight()));
  connect(this, SIGNAL(ReleaseRight()), S, SLOT(ReleaseRight()));
  connect(this, SIGNAL(ApplyBreak()), S, SLOT(ApplyBreak()));
  connect(this, SIGNAL(ReleaseBreak()), S, SLOT(ReleaseBreak()));

  setPalette( QPalette(Qt::white) );
  setAutoFillBackground( true );
  update();

}

void VehicleVisualization::paintEvent( QPaintEvent * ){

  QPainter Rys( this );
  Rys.setBackground( Qt::white );


  Rys.rotate(param.q[4]*180/PI);
  Rys.drawRect(param.q[2],param.q[3],10,15);
  cout << param.q[2] << " , " << param.q[3] << endl;



}

void VehicleVisualization::fresh( VehicleParameters p ){

  for(int i=0; i < STATES; i++){

    param.q[i] = p.q[i];

  }

  update();

}

void VehicleVisualization::keyPressEvent( QKeyEvent *event ){

  if( event->isAutoRepeat() ){
    QWidget::keyPressEvent( event );
    return;
  }

  switch( event->key() ){
  case Qt::Key_W: 
    emit ApplyTorque();
    break;
  case Qt::Key_S:
    emit ApplyBreak();
    break;
  case Qt::Key_A:
    emit ApplyLeft();
    break;
  case Qt::Key_D:
    emit ApplyRight();
    break;
  default: QWidget::keyPressEvent( event );

  }

}


void VehicleVisualization::keyReleaseEvent( QKeyEvent *event ){

  if( event->isAutoRepeat() ){
    QWidget::keyPressEvent( event );
    return;
  }

  switch( event->key() ){
  case Qt::Key_W: 
    emit ReleaseTorque();
    break;
  case Qt::Key_S:
    emit ReleaseBreak();
    break;
  case Qt::Key_A:
    emit ReleaseLeft();
    break;
  case Qt::Key_D:
    emit ReleaseRight();
    break;
  default: QWidget::keyReleaseEvent( event );

  }

}
