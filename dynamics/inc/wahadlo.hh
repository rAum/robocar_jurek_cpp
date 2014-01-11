#ifndef WAHADLO
#define WAHADLO

#include <QTimer>
#include <cmath>
#include "symulacja.hh"

#define PI 3.141592

/*! \struct PendulumParameters wahadlo.hh "inc/wahadlo.hh"
 * \brief Structure used for exchanging data between solver and graphical
 * engine
 */
struct PendulumParameters{

  /*! \brief q1 - joint1, q2 - joint2, q3 = dq1/dt, q4 = dq2/dt */
  double q[4];
  /*! \brief l1 - joint1 length, l2 - joint2 length */
  double l[2];
  /*! \brief m1 - joint1 mass, m2 - joint2 mass */
  double m[2];

};

Q_DECLARE_METATYPE( PendulumParameters )

class Wahadlo: public QObject, public ODESolver{
Q_OBJECT
  QTimer *Timer;
  PendulumParameters param;


public:

  Wahadlo( QObject *wRodzic = 0 );
  ~Wahadlo();
  void  ODEFun(double *x, double *u, double *xp);

public slots:

  void DoTheStep();
  void ApplyForce();
  void ReleaseForce();

signals:

  void UpdateAngles( PendulumParameters p );

};

#endif
