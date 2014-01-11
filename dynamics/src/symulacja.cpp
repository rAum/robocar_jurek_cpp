#include "symulacja.hh"

using namespace std;
///////////////////////////////////////////////////////////////////////////


ODESolver::ODESolver(): step(0), x0(NULL), u(NULL), dimx(0), dimu(0){}
ODESolver::~ODESolver(){

  if(x0 != NULL) delete[] x0;
  if(u != NULL) delete[] u;

}

void ODESolver::SetDimX(int dim){ dimx = dim; }
void ODESolver::SetDimU(int dim){ dimu = dim; }
void ODESolver::SetStep(double s){ step = s; }

void ODESolver::SetInitialConditions(double *x){

  if( dimx == 0 ){
    cout << "Dimx = 0\n";
    return;
  }

  if(x0 == NULL) x0 = new double[dimx];

  for(int i=0; i<dimx; i++)  x0[i] = x[i];

}

void ODESolver::SetControlInput(double *uu){

  if( dimu == 0 ){
    cout << "Dimu = 0\n";
    return;
  }

  if(u == NULL) u = new double[dimu];

  for(int i=0; i<dimu; i++)  u[i] = uu[i];

}

int ODESolver::DimX(){ return dimx; }
int ODESolver::DimU(){ return dimu; }
double* ODESolver::X(){ return x0; }
double* ODESolver::U(){ return u; }
/* Example: x' = x; */
void ODESolver::ODEFun(double *x, double *u, double *xp){ 

  for(int i=0; i<dimx; i++) xp[i] = x[i]+u[0];

 }

void ODESolver::EulerMethodStep(){

  double *x_temp = new double[dimx];
  double *xp = new double[dimx];

  this->ODEFun(x0,u,xp);

  for(int i=0; i<dimx; i++) x_temp[i] = x0[i] + step*xp[i];
  for(int i=0; i<dimx; i++) x0[i] = x_temp[i];

  delete[] x_temp;
  delete[] xp;

}

/////////////////////////////////////////////////////////////////////////////



Przyklad::Przyklad(){

  /* Rownanie oscylatora x'' + 4x == 0, x(0)=1, x'(0)=0 */
  SetDimX(2);
  double *x0 = new double[2];
  x0[0]=1;x0[1]=0;
  SetInitialConditions(x0);
  delete[] x0;
  SetStep(0.01);

}
Przyklad::~Przyklad(){}

void Przyklad::ODEFun(double *x, double *u, double *xp){

  xp[0] = x[1];
  xp[1] = -4*x[0]+u[0];
  u;

}


