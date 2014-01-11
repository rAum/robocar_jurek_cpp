#ifndef VEHICLE
#define VEHICLE

#include <QTimer>
#include <cmath>
#include <iostream>
#include "symulacja.hh"

#define PI 3.141592

/*! \struct VehicleParameters Vehicle.hh "inc/Vehicle.hh"
 * \brief Structure used for exchanging data between solver and graphical
 * engine
 */
struct VehicleParameters{

  /*! \brief q1 - eta1, q2 - eta2, q3 = x, q4 = y, q5 = phi, q6 = delta, q7 = theta */
  double q[7];
  /*! \brief [0] - a, [1] - R, [2] - mc, [3] - Ic, [4] - Ikier, [5] - Ikol*/
  double params[6];

};

Q_DECLARE_METATYPE( VehicleParameters )

class Vehicle: public QObject, public ODESolver{
Q_OBJECT
  QTimer *Timer;
  VehicleParameters param;


public:

  Vehicle( QObject *wRodzic = 0 );
  ~Vehicle();
  void  ODEFun(double *x, double *u, double *xp);

public slots:

  void DoTheStep();
  void ApplyTorque();
  void ReleaseTorque();
  void ApplyLeft();
  void ReleaseLeft();
  void ApplyRight();
  void ReleaseRight();
  void ApplyBreak();
  void ReleaseBreak();

signals:

  void UpdateStates( VehicleParameters p );

};

#endif
