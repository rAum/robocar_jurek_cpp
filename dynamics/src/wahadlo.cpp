#include "wahadlo.hh"


Wahadlo::Wahadlo( QObject *wRodzic ): QObject( wRodzic ){

  param.q[0] = PI/2; param.q[1] = 0; param.q[2] = PI/2; param.q[3] = 0;
  param.l[0] = 6; param.l[1] = 3;
  param.m[0] = 5; param.m[1] = 4;



  SetDimX(4); // 4 states: q1,q2,q'1,q'2
  SetDimU(2); // u1, u2 - control inputs
  double *q0 = new double[4];
  double *u = new double[2];
  for(int i=0;i<4;i++) q0[i] = param.q[i];
  u[0] = 0; u[1] = 0;
  SetInitialConditions(q0);
  SetControlInput(u);
  delete[] q0;
  delete[] u;
  SetStep(0.01); // 10msec

  Timer = new QTimer( this );
  connect(Timer, SIGNAL(timeout()), this, SLOT(DoTheStep()));
  Timer->start(10); // 10msec

}

Wahadlo::~Wahadlo(){}

/* Euler-Lagrange equations */

void Wahadlo::ODEFun(double *x, double *u, double *xp){

  double g = 9.81;
  double l1 = param.l[0];
  double l2 = param.l[1];
  double m1 = param.m[0];
  double m2 = param.m[1];
  double u1 = u[0];
  double u2 = u[1];
  double Tarc = 50; // friction force

  xp[0] = x[2];
  xp[1] = x[3];
  xp[2] = (g*l1*l2*((2*m1 + m2)*cos(x[0]) - m2*cos(x[0] + 2*x[1])) + pow(l1,2)*l2*m2*sin(2*x[1])*pow(x[2],2) + 2*l1*pow(l2,2)*m2*sin(x[1])*pow(x[2] + x[3],2) - 2*l1*cos(x[1])*(u2 - Tarc*x[3]) + 2*l2*(u1 - u2 + Tarc*(-x[2] + x[3])))/(2.*pow(l1,2)*l2*(m1 + m2 - m2*pow(cos(x[1]),2)));
  xp[3] = ((g*l2*m2*cos(x[0] + x[1]) + u2 - l1*l2*m2*sin(x[1])*pow(x[2],2) - Tarc*x[3])/m2 - ((l2 + l1*cos(x[1]))*(g*l1*l2*((2*m1 + m2)*cos(x[0]) - m2*cos(x[0] + 2*x[1])) + pow(l1,2)*l2*m2*sin(2*x[1])*pow(x[2],2) + 2*l1*pow(l2,2)*m2*sin(x[1])*pow(x[2] + x[3],2) - 2*l1*cos(x[1])*(u2 - Tarc*x[3]) + 2*l2*(u1 - u2 + Tarc*(-x[2] + x[3]))))/(2.*pow(l1,2)*(m1 + m2 - m2*pow(cos(x[1]),2))))/pow(l2,2);

}

void Wahadlo::DoTheStep(){

  double *q;
  q = X();
  //cout << "q1="<<q[0]<<", q2="<<q[1]<<", q3="<<q[2]<<", q4="<<q[3]<<endl;
  for(int i=0; i<4; i++) param.q[i] = q[i];
  emit UpdateAngles( param );
  EulerMethodStep();

}

void Wahadlo::ApplyForce(){

  double *u = new double[2];
  u[0] = 1000;
  u[1] = 0;
  SetControlInput(u);
  delete[] u;

}

void Wahadlo::ReleaseForce(){

  double *u = new double[2];
  u[0] = 0;
  u[1] = 0;
  SetControlInput(u);
  delete[] u;

}
