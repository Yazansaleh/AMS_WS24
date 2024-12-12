/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "main.hpp"
#include <newmat/newmatap.h>   // Lineare Algebra
#include <fstream>              // Für Datei-Zugriff in Aufgabe 4

using namespace AMS;
using namespace std;

AMS_Robot robot;               // Roboterobjekt

int main(int argc, char **argv) {

    ColumnVector P0(2);        // Startpunkt der Bahnkurve
    ColumnVector P1(2);        // Scheitelpunkt des krümmungsstetigen Übergangs
    ColumnVector P2(2);        // Endpunkt der Bahnkurve
    ColumnVector V01(2);       // Vektor vom Punkt 0 zum Punkt 1
    ColumnVector V12(2);       // Vektor vom Punkt 1 zum Punkt 2
    ColumnVector Pc0(2);       // Startpunkt des krümmungsstetigen Übergangs
    ColumnVector Pc1(2);       // Endpunkt des krümmungsstetigen Übergangs
    double Rmin;               // Kurvenradius im Scheitelpunkt der Bahnkurve (=Kehrwert der max. Krümmung)
    double L01, L12;           // Längen der Vektoren V01 und V12
    double theta;              // Winkel der Tangente an die Klothoide
    double thetaL;             // Tangentenwinkel der Klothoiden am Scheitelpunkt
    double L;                  // Länge der Klothoide
    double k;                  // Krümmungsparameter für Klothoide
    double xL, yL;             // Parameter für Klothoide
    double delta;              // Abstand zwischen dem Beginn der Klothoiden und Punkt P1
    double L1, L2;             // Länge der Geradenstücke vor und hinter der Klothoiden (einschließlich delta)
    double phi1, phi2;         // Richtungswinkel der Geradenstücke vor und hinter der Klothoiden
    double s;                  // Bahnlänge
    double xs=0, ys=0;         // Lokale Koordinaten der Klothoiden vom Startpunkt Pc0 aus
    ColumnVector P(2);         // Vektor mit globalen Koordinaten
    const double ds = 0.005;   // Abstand der Punkte beim Zeichnen der Trajektorie

    // Zusätzliche Variablen für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren)
   const double T = 0.2;            // Abtastzeit zwischen aufeinander folgenden Punkten
    double Delta_s = 0;                   // Abstand benachbarter Abtastwerte (abhängig von T und v)
    double vakt = 0;                 // aktuelle Bahngeschwindigkeit
    string file = "AMS_Trajekt.txt"; // Datei zum Speichern der Punkte der abgetasteten Trajektorie
    double vmax;                      // Maximale Bahngeschwindigkeit
    double vacc;                     // Bahnbeschleunigung
    double tb; 	                     // Beschleunigungszeit
    double sb;                       // Beschleunigungsweg
    ColumnVector Ps(2);              // Vektor mit zuletzt gespeicherten Koordinaten (x,y) eines Bahnpunktes

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.init_push_mode();

    // Koordinaten der Eckpunkte vorgeben
    P0(1) = -2.0;     // Startpunkt x-Koordinate
    P0(2) =  4.0;     // Startpunkt y-Koordinate
    P1(1) =  3.0;     // Scheitelpunkt x-Koordinate
    P1(2) =  0.0;     // Scheitelpunkt y-Koordinate
    P2(1) =  2.5;     // Endpunkt x-Koordinate
    P2(2) = -3.0;     // Endpunkt y-Koordinate

    /******* Fügen Sie ab hier eigenen Quellcode ein ********/

	Rmin = robot.get_vmax()/robot.get_wmax();

    V01 = P1-P0;
    V12 = P2-P1;
    L01 = V01.NormFrobenius();
    L12 = V12.NormFrobenius();

    double skalar_produkt = DotProduct(V01, V12);
    //ColumnVector vektor_produkt = crossproduct(V01, V12);
    double vektor_produkt = V01(1)*V12(2) - V01(2) * V12(1);
    //signed vorzeichen = 1;
    double alpha = M_PI - acos(skalar_produkt/(L01*L12));//acos(skalar_produkt/(L01*L12));
    //thetaL = M_PI - 2*thetaL;
    thetaL = (M_PI - alpha)/2;
    if(vektor_produkt < 0)
        thetaL = -thetaL;

    double kappa_L = 1 / Rmin;
    L = abs(2*thetaL /kappa_L);
    xL = L *(1- (pow(thetaL,2)/10) +  (pow(thetaL,4)/216));
    yL = L *((thetaL/3) -(pow(thetaL,3)/42) +  (pow(thetaL,5)/1320));
    double delta_x = yL * tan(thetaL);//yL / tan(alpha);
    delta = xL + delta_x;

    ColumnVector v_temp1 = V01* (L01-delta)/L01;
    Pc0 = P0 + v_temp1;
    ColumnVector v_temp2 = V12*(L12 -delta)/L12;
    Pc1 = P2 - v_temp2;

    /******* Ende des zusätzlich eingefügten Quellcodes *******/

    // Abbruch, falls delta_x die Länge des Vektors V01 oder die des Vektors V12 übersteigt.
    // In diesem Fall kann der krümmungsstetige Übergang mit dem vorgegebenem Radius nicht berechnet werden
    if( delta > L01 || delta > L12 ) {
        robot.log.errorStream() << "Fehler: Gewählter minimaler Radius nicht möglich.\n";
        return -1;
        }

    // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
    ofstream outfile(file.c_str()); // Datei mit Schreibzugriff öffnen
    vacc = robot.get_vacc();
    vmax = robot.get_vmax();
    Ps = P0;
    tb = vmax/vacc;
    sb = vmax*vmax/(2*vacc);

    // Zeichnen des ersten Geradenstücks bis zum Beginn der Klothoide
    L1 = (Pc0-P0).NormFrobenius();
    phi1 = atan2(V01(2),V01(1)); // Orientierung des Geradenstücks, d.h. des Vektors V01 berechnen
    for( s=0; s<L1; s+=ds) {
        P(1) = P0(1) + s*cos(phi1);
        P(2) = P0(2) + s*sin(phi1);
        robot.draw_point(P(1), P(2), 0, 255, 0, 'm');  // Punkt zeichnen

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
       if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            if(s < sb) vakt += vacc*T;
            else vakt = vmax;
            Delta_s = vakt *T;
        }
    }

    ColumnVector letzterK_Punkt;
    // Zeichnen der Klothoiden
    k = 0.5/(thetaL*Rmin*Rmin);    // Parameter der Klothoiden (Vorzeichen von thetaL bestimmt Krümmungsrichtung)
    for( s=0; s<=2*L; s+=ds ) {    // Schleife über die Länge der zwei Klothoiden

        /******* Fügen Sie ab hier eigenen Quellcode ein ********/

        if( s<=L )
            theta = 0.5*k*pow(s, 2);      // 1. Hälfte: Zunahme der Krümmung
        else{
            theta = 2*thetaL - 0.5*k*pow(2*L-s, 2);      // 2. Hälfte: Abnahme der Krümmung
            // phi = phi2;
        }
        xs += ds*cos(theta);
        ys += ds*sin(theta);
        P(1) = Pc0(1) + (cos(phi1)*xs-sin(phi1)*ys);
        P(2) = Pc0(2) + (sin(phi1)*xs+cos(phi1)*ys);
        letzterK_Punkt = P;

        /******* Ende des zusätzlich eingefügten Quellcodes *******/

        robot.draw_point(P(1), P(2), 255, 0, 255, 'm'); // Punkt zeichnen

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
        if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            Delta_s = vakt *T;
        }
   }

    // Zeichnen des zweiten Geradenstücks hinter der Klothoide
    phi2 = atan2(V12(2),V12(1));
    L2 = (P2-Pc1).NormFrobenius();
    for( s=0; s<L2; s+=ds) {
        P(1) = Pc1(1) + s*cos(phi2);
        P(2) = Pc1(2) + s*sin(phi2);
        robot.draw_point(P(1), P(2), 0, 255, 0, 'm');

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
       if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            if((L12 -s) < sb) vakt = vakt - vacc*T;
            else if((L12-s)== sb) vakt = 0;
            Delta_s = vakt*T;
        }
    }

    // Ausgabe der berechneten Teillängen
	robot.log.info("Length of 1st straight line: %.2lf m.", L1);
	robot.log.info("Length of clothoide: %.2lf m.", L*2);
	robot.log.info("Length of 2nd straight line: %.2lf m.", L2);
	robot.log.info("V01: %.2lf %.2lf", V01(1), V01(2));
	robot.log.info("V12: %.2lf %.2lf", V12(1), V12(2));
	robot.log.info("Vtemp1: %.2lf %.2lf", v_temp1(1), v_temp1(2));
	robot.log.info("Vtemp2: %.2lf %.2lf", v_temp2(1), v_temp2(2));
	robot.log.info("delta: %.2lf", delta);
	robot.log.info("L01: %.2lf", L01);
	robot.log.info("L12: %.2lf", L12);
	robot.log.info("skalar_produkt: %.2lf", skalar_produkt);
	robot.log.info("alpha: %.2lf", alpha);
	robot.log.info("thetaL: %.2lf", thetaL);
	robot.log.info("k: %.2lf", k);
	robot.log.info("letzterK_Punkt: %.2lf, %.2lf", letzterK_Punkt(1), letzterK_Punkt(2));
	robot.log.info("Pc1: %.2lf, %.2lf", Pc1(1), Pc1(2));

	int turndir = 1;
	if (thetaL < 0) turndir = -1;
	robot.set_sim_pos(P0(1), P0(2), phi1);
	robot.set_slip_const(0);
	DriveRobot(&robot, L1, L, L2, turndir);

    while(1) // Endlosschleife
    {
    }
}
