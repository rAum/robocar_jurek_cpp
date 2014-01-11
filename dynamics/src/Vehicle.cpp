#include "Vehicle.hh"

#define STATES 7
#define CONTROLS 2

using namespace std;

Vehicle::Vehicle( QObject *wRodzic ): QObject( wRodzic ){

  param.q[0] = 0; param.q[1] = 0; param.q[2] = 0; param.q[3] = 0;
  param.q[4] = 0; param.q[5] = 0; param.q[6] = 0;
  param.params[0] = 1; param.params[1] = 0.1; param.params[2] = 100;
  param.params[3] = 1; param.params[4] = 10; param.params[5] = 10;


  SetDimX(STATES); // 7 states: eta1,eta2,x,y,phi,delta,theta
  SetDimU(CONTROLS); // u1, u2 - control inputs
  double *q0 = new double[STATES];
  double *u = new double[CONTROLS];
  for(int i=0;i<STATES;i++) q0[i] = param.q[i];
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

Vehicle::~Vehicle(){}

/* Euler-Lagrange equations */

void Vehicle::ODEFun(double *x, double *u, double *xp){

  double a = param.params[0];
  double R = param.params[1];
  double mc = param.params[2];
  double Ic = param.params[3];
  double Ikier = param.params[4];
  double Ikol = param.params[5];
  double u1 = u[0];
  double u2 = u[1];


  xp[0] = (8*pow(a,2)*pow(cos(x[5]),2)*(u1 - ((4*Ic + pow(a,2)*mc)*pow(R,2)*pow(1/cos(x[5]),2)*tan(x[5])*x[0]*x[1])/(4.*pow(a,2))))/(4*Ic*pow(R,2) + pow(a,2)*(4*Ikol + 5*mc*pow(R,2)) + (-4*Ic*pow(R,2) + pow(a,2)*(4*Ikol + 3*mc*pow(R,2)))*cos(2*x[5]));
  xp[1] == u2/Ikier;
  xp[2] = (R*cos(x[4]) - (R*sin(x[4])*tan(x[5]))/2.)*x[0];
  xp[3] = (R*sin(x[4]) + (R*cos(x[4])*tan(x[5]))/2.)*x[0];
  xp[4] = (R*tan(x[5])*x[0])/a;
  xp[5] = x[1];
  xp[6] = x[0];

}

void Vehicle::DoTheStep(){

  double *q;
  q = X();
  for(int i=0; i<STATES; i++) param.q[i] = q[i];
  emit UpdateStates( param );
  EulerMethodStep();

}

void Vehicle::ApplyTorque(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = 10;
  u[1] = old_u[1];
  SetControlInput(u);
  cout << "Press W \n";
  delete[] u;

}

void Vehicle::ReleaseTorque(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = 0;
  u[1] = old_u[1];
  SetControlInput(u);
  cout << "Release W \n";
  delete[] u;

}

void Vehicle::ApplyLeft(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = old_u[0];
  u[1] = -1;
  SetControlInput(u);
  delete[] u;

}

void Vehicle::ReleaseLeft(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = old_u[0];
  u[1] = 0;
  SetControlInput(u);
  delete[] u;

}

void Vehicle::ApplyRight(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = old_u[0];
  u[1] = 1;
  SetControlInput(u);
  delete[] u;

}

void Vehicle::ReleaseRight(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = old_u[0];
  u[1] = 0;
  SetControlInput(u);
  delete[] u;

}

void Vehicle::ApplyBreak(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = -10;
  u[1] = old_u[1];
  SetControlInput(u);
  delete[] u;

}

void Vehicle::ReleaseBreak(){

  double *u = new double[2];
  double *old_u = U();
  u[0] = 0;
  u[1] = old_u[1];
  SetControlInput(u);
  delete[] u;

}
