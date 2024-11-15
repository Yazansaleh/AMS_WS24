/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include <newmat/newmatap.h>   // Lineare Algebra
#include <fstream>              // Für Datei-Zugriff in Aufgabe 4

using namespace AMS;

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
    double px, py;             // Koordinaten für Zeichnung
    ColumnVector P(2);         // Vektor mit globalen Koordinaten
    const double ds = 0.005;   // Abstand der Punkte beim Zeichnen der Trajektorie

    // Zusätzliche Variablen für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren)
/*    const double T = 0.2;            // Abtastzeit zwischen aufeinander folgenden Punkten
    double Delta_s                   // Abstand benachbarter Abtastwerte (abhängig von T und v)
    double vakt = 0;                 // aktuelle Bahngeschwindigkeit
    string file = "AMS_Trajekt.txt"; // Datei zum Speichern der Punkte der abgetasteten Trajektorie
    double vmax                      // Maximale Bahngeschwindigkeit
    double vacc;                     // Bahnbeschleunigung
    double tb; 	                     // Beschleunigungszeit
    double sb;                       // Beschleunigungsweg
    ColumnVector Ps(2);              // Vektor mit zuletzt gespeicherten Koordinaten (x,y) eines Bahnpunktes
*/
    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.init_push_mode();

    // Koordinaten der Eckpunkte vorgeben
/*    P0(1) = -2.0;     // Startpunkt x-Koordinate
    P0(2) =  4.0;     // Startpunkt y-Koordinate
    P1(1) =  3.0;     // Scheitelpunkt x-Koordinate
    P1(2) =  0.0;     // Scheitelpunkt y-Koordinate
    P2(1) =  2.5;     // Endpunkt x-Koordinate
    P2(2) = -3.0;     // Endpunkt y-Koordinate
*/
    P0(1) = -2.0;     // Startpunkt x-Koordinate
    P0(2) =  0.0;     // Startpunkt y-Koordinate
    P1(1) =  3.0;     // Scheitelpunkt x-Koordinate
    P1(2) =  2.0;     // Scheitelpunkt y-Koordinate
    P2(1) =  2.5;     // Endpunkt x-Koordinate
    P2(2) = -3.0;     // Endpunkt y-Koordinate


    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
// a)
	Rmin = robot.get_vmax()/robot.get_wmax();

// b)
    V01 = P1 - P0;
    V12 = P2 - P1;
    L01 = V01.NormFrobenius();
    L12 = V12.NormFrobenius();
// c)
    thetaL = (M_PI - (M_PI - acos(DotProduct(V01, V12)/(L01*L12))))/2;
    double V01xV12 = V01(1)*V12(2)-V01(2)*V12(1);
    thetaL = V01xV12 >= 0 ? thetaL : -thetaL;

// d)
    L = abs(2*thetaL*Rmin);
    xL = L*(1 - pow(thetaL,2)/10 + pow(thetaL,4)/216);
    yL = L*(thetaL/3 - pow(thetaL,3)/42 + pow(thetaL,5)/1320);
    delta = xL +yL*tan(thetaL);                      //xL+deltax
// e)
    Pc0 =P0+ V01* (L01-delta)/L01;             // P0->P1 mit länge delta                        
    Pc1 =P1+ V12* delta/L12;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // Abbruch, falls delta_x die Länge des Vektors V01 oder die des Vektors V12 übersteigt.
    // In diesem Fall kann der krümmungsstetige Übergang mit dem vorgegebenem Radius nicht berechnet werden
    if( delta > L01 || delta > L12 ) {
        robot.log.errorStream() << "Fehler: Gewählter minimaler Radius nicht möglich.\n";
        return -1;
        }

    // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
/*    ofstream outfile(file.c_str()); // Datei mit Schreibzugriff öffnen
    Ps = P0;
    tb =
    sb =
*/
    // Zeichnen des ersten Geradenstücks bis zum Beginn der Klothoide
    L1 = (Pc0-P0).NormFrobenius();
    phi1 = atan2(V01(2),V01(1)); // Orientierung des Geradenstücks, d.h. des Vektors V01 berechnen
    for( s=0; s<L1; s+=ds) {
        P(1) = P0(1) + s*cos(phi1);
        P(2) = P0(2) + s*sin(phi1);
        robot.draw_point(P(1), P(2), 0, 255, 0, 'm');  // Punkt zeichnen

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
/*        if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            Delta_s =
        }
*/
    }

    // Zeichnen der Klothoiden
    k = 0.5/(thetaL*Rmin*Rmin);    // Parameter der Klothoiden (Vorzeichen von thetaL bestimmt Krümmungsrichtung)
    for( s=0; s<=L*2; s+=ds ) {    // Schleife über die Länge der zwei Klothoiden

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
        if( s<=L )
            theta = k*s*s/2;      // 1. Hälfte: Zunahme der Krümmung
        else
            theta = 2*thetaL-k*(2*L-s)*(2*L-s)/2;      // 2. Hälfte: Abnahme der Krümmung
        // Berechnung der lokalen Klothoidenkoordinaten vom Startpunkt Pc0 aus
        xs += ds*cos(theta);
        ys += ds*sin(theta);
        // Koordinatentransformation mit Berücksichtigung des Startpunktes Pc0 und der Richtung phi1
        px = Pc0(1) + xs*cos(phi1) - ys*sin(phi1);
        py = Pc0(2) + xs*sin(phi1) + ys*cos(phi1);

        P(1) = px;
        P(2) = py;

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        robot.draw_point(P(1), P(2), 255, 0, 255, 'm'); // Punkt zeichnen

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
/*        if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            Delta_s =
        }
*/
   }

    // Zeichnen des zweiten Geradenstücks hinter der Klothoide
    phi2 = atan2(V12(2),V12(1));
    L2 = (P2-Pc1).NormFrobenius();
    for( s=0; s<L2; s+=ds) {
        P(1) = Pc1(1) + s*cos(phi2);
        P(2) = Pc1(2) + s*sin(phi2);
        robot.draw_point(P(1), P(2), 0, 255, 0, 'm');

        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
/*        if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            Delta_s =
        }
*/
    }
    // Ausgabe der berechneten Teillängen
	robot.log.info("Length of 1st straight line: %.2lf m.", L1);
	robot.log.info("Length of clothoide: %.2lf m.", L*2);
	robot.log.info("Length of 2nd straight line: %.2lf m.", L2);

    while(1); // Endlosschleife
}
