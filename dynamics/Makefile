NAZWA_APLIKACJI=Dynamics
URUCHOMIENIE_APLIKACJI=./Dynamics
ZRODLA_PROJEKTU=src/*.cpp
PODSTAWIENIA=OBJECTS_DIR=${KATALOG_OBJ} QMAKE_CXXFLAGS+=-g
KATALOG_OBJ=./obj
INC=inc/*.hh
MOC_DIR=${KATALOG_MOC}
KATALOG_MOC=./moc



__start__: __sprawdz_Qt__ ${NAZWA_APLIKACJI}
	rm -f core*; ${URUCHOMIENIE_APLIKACJI}

__sprawdz_Qt__: __sprawdz_qmake__
	@if qmake -v | grep 'ver.*4\.[0-9]*\.[0-9]*' > /dev/null;\
         then exit 0;\
         else echo; echo " Brak biblioteki Qt w wersji 4.x.x";\
              echo;  exit 1;\
        fi

__sprawdz_qmake__:
	@if which qmake > /dev/null; then exit 0;\
        else\
          echo; echo " Brak programu qmake."\
               " Prawdopodobnie biblioteka Qt nie zostala zainstalowana.";\
          echo; exit 1;\
        fi


${NAZWA_APLIKACJI}: Makefile.app __sprawdz__

__sprawdz__:
	make -f Makefile.app

Makefile.app: ${NAZWA_APLIKACJI}.pro
	qmake -o Makefile.app ${NAZWA_APLIKACJI}.pro

${NAZWA_APLIKACJI}.pro:
	rm -f ${NAZWA_APLIKACJI}
	qmake -project -nopwd -o ${NAZWA_APLIKACJI}.pro\
               ${PODSTAWIENIA} ${ZRODLA_PROJEKTU} ${INC}

project: __usun_pro__ ${NAZWA_APLIKACJI}.pro

__usun_pro__:
	rm -f ${NAZWA_APLIKACJI}.pro

clean:
	make -f Makefile.app clean

cleanall: clean
	rm -f ${NAZWA_APLIKACJI}

cleantotally: cleanall
	rm -f ${NAZWA_APLIKACJI}.pro Makefile.app
	rm -fr ${KATALOG_MOC} ${KATALOG_OBJ}
	find . -name \*~ -exec rm {} \;
	find . -name \*.tex -exec rm {} \;

help:
	@echo "Podcele:"
	@echo 
	@echo " project  - wymusza utworzenie nowego projektu"
	@echo " clean    - usuwa wszystkie produkty kompilacji i konsolidacji"
	@echo " cleanall - usuwa produkty kompilacji wraz z aplikacja"
	@echo " cleantotally - usuwa wszystko oprocz zrodel i pliku Makefile"
	@echo " help     - wyswietla niniejszy pomoc"
	@echo
