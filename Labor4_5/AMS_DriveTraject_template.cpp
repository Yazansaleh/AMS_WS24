/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include <fstream>          // Für Datei-Zugriff

using namespace AMS;
using namespace PlayerCc;

AMS_Robot robot;

int main(int argc, char **argv) {

    const double T = 0.3;   // feste Zeitdauer zwischen zwei Wegpunkten in Sekunden
    double x, x_1;          // aktuelle und vorherige x-Koordinate
    double y, y_1;          // aktuelle und vorherige y-Koordinate
    double ds, ds_1;        // aktueller und vorheriger Abstand zwischen benachbarten Punkten
    double theta, theta_1;  // aktueller und vorheriger Winkel zwischen benachbarten Punkten
    double dtheta;          // Differenzwinkel zwischen aktueller und vorheriger Strecke
    double kapa;            // aktuelle Krümmung
    double v, w;            // Sollwerte für Bahn- und Winkelgeschwindigkeit
    string file;            // Datei mit Punkten der zu befahrenden Trajektorie
    double dt;              // Zeitdauer für das Durchlaufen der aktuellen Schleife
    ptime tref;             // Objekt der Klasse ptime zur Messung von dt

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.set_slip_const(0.0); // Schlupf des Roboters auf Null setzen

	// Pfad zu Datei mit abgetasteter Trajektorie
	file = "AMS_Trajekt.txt";

    robot.init_pull_mode();  // Aktuelle Werte aus Proxies lesen

    ifstream infile(file.c_str());     // Eingabe-Stream erzeugen und mit Dateinamen initialisieren
    infile >> x >> y;              // Koordinaten des Startpunktes aus Stream lesen
    infile >> x_1 >> y_1;          // Koordinaten des zweiten Punktes aus Stream lesen

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

	// Trajektorie in Stage-Fenster plotten

    ds_1 = sqrt((x_1 - x)*(x_1 -x) + (y_1 - y)*(y_1 -y));
    theta_1 = atan2(y_1 - y, x_1 -x);

    // Roboter im Stage-Fenster an Startpunkt setzen
	robot.set_sim_pos(x, y, theta_1);

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(infile >> x >> y) { // Schleife über Punkte der Trajektorie, dabei jeweils nächsten Punkt einlesen

        tref = microsec_clock::local_time(); // Referenzzeit zum Messen der Schleifendurchlaufzeit dt speichern

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
        robot.draw_point(x, y, 255, 0, 0, 'm');
        ds = sqrt((x - x_1)*(x - x_1) + (y - y_1)*(y - y_1));
        v = ds/T;
        theta = atan2(y - y_1, x - x_1);

        kapa = (theta - theta_1)*2/(ds + ds_1);
        w = v*kapa;

        // Sollgeschwindigkeiten setzen
        robot.set_speed(v, w);

        // Werte für nächsten Schleifendurchgang umkopieren
        x_1 = x;
        y_1 = y;
        ds_1 = ds;
        theta_1 = theta;


        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        // Warten um insgesamt die Zeitdauer T zwischen zwei aufeinanderfolgenden Schleifendurchläufen
        // Dabei Abbruch, falls Berechnungszeit pro Schleifendurchlauf T überschreitet
        dt = (double)(microsec_clock::local_time()-tref).total_milliseconds()/1000;
        if( dt > T ) {
            printf("Fehler! Abtastzeit zu kurz: dt=%.3lf T=%.3lf\n", dt, T);
            robot.stop(); // stoppen
            while(1);     // warten
        }
        usleep((T-dt)*1000000); // Warten in Micro-Sekunden
    }

    robot.stop(); // stoppen

    while(1); // Endlosschleife
    return 0;
}
