/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include <newmat/newmatap.h>    // Lineare Algebra
#include <fstream>              // Für Datei-Zugriff in Aufgabe 4

using namespace AMS;
using namespace PlayerCc; // dtor, rtod, limit, normalize

AMS_Robot robot;               // Roboterobjekt
void DriveRobot(AMS_Robot* robotp, double L1, double LK, double L2, int turndir);

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

// a) Berechnung des minimalen Krümmungsradius Rmin basierend auf der maximalen Bahn- und Winkelgeschwindigkeit
    Rmin = robot.get_vmax() / robot.get_wmax();

// b) Berechnung der Vektoren V01 und V12 zwischen den Punkten P0 -> P1 und P1 -> P2 sowie deren Längen L01 und L12
    V01 = P1 - P0;
    V12 = P2 - P1;
    L01 = V01.NormFrobenius();
    L12 = V12.NormFrobenius();

// c) Berechnung des Tangentenwinkels thetaL an den Klothoiden-Scheitelpunkt
// - Berechnet den Winkel zwischen den Vektoren V01 und V12
// - Das Vorzeichen wird über das Kreuzprodukt V01xV12 bestimmt, um die Richtung (links/rechts) festzulegen
    thetaL = acos(DotProduct(V01, V12) / (L01 * L12)) / 2;
    double V01xV12 = V01(1) * V12(2) - V01(2) * V12(1);
    thetaL = V01xV12 >= 0 ? thetaL : -thetaL;

// d) Berechnung der halben Klothoidenlänge L und der Koordinaten xL und yL am Scheitelpunkt der Klothoide
// - L = Gesamtlänge der Klothoide, um die Krümmung flüssig anzupassen
// - xL: X-Koordinate am Scheitelpunkt der Klothoide, Taylor-Approximation für Genauigkeit
    L = abs(2 * thetaL * Rmin);
    xL = L * (1 - pow(thetaL, 2) / 10 + pow(thetaL, 4) / 216);  // Taylor-Reihen-Entwicklung zur Näherung der Fresnel-Integrale

// - yL: Y-Koordinate am Scheitelpunkt der Klothoide, basierend auf Taylor-Approximation für y
    yL = L * (thetaL / 3 - pow(thetaL, 3) / 42 + pow(thetaL, 5) / 1320);  // Höhere Ordnungen der Taylor-Reihe für Steigungsanpassung
// - delta: Offset für einen flüssigen Übergang von der Geraden zur Klothoide (delta = xL + yL * tan(thetaL))
    delta = xL + yL * tan(thetaL);

// e) Berechnung der Start- und Endpunkte Pc0 und Pc1 der Klothoide für einen krümmungsstetigen Übergang
// - Pc0: Startpunkt der Klothoide in Richtung P1, abgeleitet von P0 unter Berücksichtigung von delta
// - Pc1: Endpunkt der Klothoide in Richtung P2
    Pc0 = P0 + V01 * (L01 - delta) / L01;
    Pc1 = P1 + V12 * delta / L12;

// Hinweis zu den Konstanten in den Berechnungen:
// Die Werte wie 1/10, 1/216, 1/3, 1/42, und 1/1320 stammen aus der Taylor-Reihenentwicklung der Fresnel-Integrale,
// die die mathematische Darstellung der Klothoidenkurve annähern. Sie korrigieren x und y für die Klothoidenform,
// ohne das gesamte Integral berechnen zu müssen.

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

	    // Berechnung des Tangentenwinkels theta entlang der Klothoide
        if( s <= L )
            theta = k * s * s / 2;      // Erste Hälfte der Klothoide: Zunahme der Krümmung (Winkel nimmt zu)
        else
            theta = 2 * thetaL - k * (2 * L - s) * (2 * L - s) / 2;  // Zweite Hälfte: Abnahme der Krümmung

        // Berechnung der lokalen Koordinaten xs und ys für die Klothoide
        // - Schrittweite ds wird entlang des Winkels theta in x- und y-Richtung aufgeteilt
        xs += ds * cos(theta);
        ys += ds * sin(theta);

        // Transformation der lokalen Klothoidenkoordinaten (xs, ys) in globale Koordinaten (px, py)
        // - Startpunkt Pc0 und Ausrichtung phi1 werden berücksichtigt
        px = Pc0(1) + xs * cos(phi1) - ys * sin(phi1);  // Globale x-Koordinate
        py = Pc0(2) + xs * sin(phi1) + ys * cos(phi1);  // Globale y-Koordinate

        P(1) = px;
        P(2) = py;

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        robot.draw_point(P(1), P(2), 255, 0, 255, 'm'); // Zeichnen des Punktes P auf der Klothoide
    }


        // Zusätzlicher Quellcode für Aufgabe 4 (bitte bei deren Bearbeitung entkommentieren und ergänzen)
/*        if( (P-Ps).NormFrobenius() >= Delta_s ) {
            Ps = P;
            outfile << P(1) << " " << P(2) << endl;
            Delta_s =
        }
*/


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

	int turndir = 1;
	if(thetaL<0) turndir = -1;
	robot.set_sim_pos(P0(1), P0(2), phi1);
	robot.set_slip_const(0);

    DriveRobot(&robot, L1,L,L2,turndir);
    while(1); // Endlosschleife
}

void DriveRobot(AMS_Robot* robotp, double L1, double LK, double L2, int turndir) {

    double sb;           				// Beschleunigungsweg (= Bremsweg)
    double tb;           				// Beschleunigungszeit
    double tc;           				// Fahrtzeit mit vmax
    double dt;           				// aktuelle Bewegungszeit
    double t0;           				// Startzeitpunkt der Bewegung
    double t0K;          				// Startzeitpunkt der Kurvenfahrt
    double tbK;          				// Zeitdauer bis max. Winkelgeschwindigkeit erreicht wird
    double w;            				// aktuelle vorzeichenrichtige Winkelgeschwindigkeit
    double vmax = robotp->get_vmax();  // Maximale Bahngeschwindigkeit
    double vacc = robotp->get_vacc();  // Bahnbeschleunigung
    double wmax = robotp->get_wmax();  // maximale Winkelgeschwindigkeit

    robotp->init_push_mode();  // data shall be read from message queue

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    sb = (vmax*vmax) / (2*vacc);
    tb = vmax/vacc;
    tc = (L1 * 2*LK * L2 -2*sb)/vmax;
    tbK =  2*LK / 2*vmax;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // gleichmäßige Beschleunigung während tb
    robotp->wait_for_new_data(); // aktuelle Daten vom Roboter holen
    t0 = robotp->get_t(); // Startzeitpunkt
    robotp->log.info("Accelerating.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0; // Bewegungszeit updaten
        robotp->set_speed(vacc*dt, 0);
    }
    while( dt < tb);

    // gleichförmige Bewegung mit maximaler Bahngeschwindigkeit während tc
    // Währenddessen Befahren der Klothoiden
    robotp->set_speed(vmax, 0);
    robotp->log.info("Driving with constant speed.");

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
    t0K = (L1 - sb)/vmax + tb + t0;
    double t0bk = t0K + tbK;
    double tcEnd = t0 + tb + tc;

    robotp->log.info("Driving with constant speed.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0; // Bewegungszeit updaten
    }
    while( dt < t0K);

    robotp->log.info("Driving in Klothoid 1st Half");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0K; // Bewegungszeit updaten
        w = (wmax * turndir) * (dt/tbK);
        robotp->set_speed(vmax, w);
    }
    while( dt < tbK);

        robotp->log.info("Driving in Klothoid 2nd Half");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0bk; // Bewegungszeit updaten
        w = (wmax - (wmax * (dt/tbK))* turndir);
        robotp->set_speed(vmax, w);
    }
    while( dt < 2*tbK);

 do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0; // Bewegungszeit updaten
        robotp->set_speed(vmax, w);
    }
    while( dt > 2*t0K && dt < tcEnd );




    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // Abbremsen während tb
    robotp->log.info("Decelerating.");
    do {
        robotp->wait_for_new_data();
        dt = robotp->get_t() - t0 - tb - tc; // aktuelle Bremszeit
        robotp->set_speed(vmax-vacc*dt, 0);
    }
    while( dt < tb);

    robotp->stop(); // Stoppen
}
