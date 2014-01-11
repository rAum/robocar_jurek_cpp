#include "gui.hh"

int main( int argc, char * argv[] ) 
{
  QApplication App(argc,argv);
  OknoGlowne   Okno;
  Okno.show();

  return App.exec();
}
