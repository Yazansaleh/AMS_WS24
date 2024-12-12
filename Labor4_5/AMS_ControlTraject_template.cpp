/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "boost/date_time/posix_time/posix_time.hpp" // Für Zeitmessungen
#include <fstream> // Für Datei-Zugriff

using namespace AMS;
using namespace PlayerCc;

AMS_Robot robot;

int main(int argc, char **argv) {

    const double T = 0.3;  // feste Zeitdauer zwischen zwei Wegpunkten in Sekunden
    double v, w;           // aktuelle Bahn- und Winkelgeschwindigkeit
    double delta, phi;     // im aktuellen Schritt zurückgelegte Distanz und überstrichener Drehwinkel
    double vx, vy;         // aktuelle x- und y-Komponenten der Bahngeschwindigkeit in Schritt i
    double x, y;           // aktuelle Position in Schritt i
    double theta;          // aktueller Ist-Winkel in Schritt i
    double v_st=0, w_st=0; // Stellgrößen für Bahn- und Winkelgeschwindigkeit
    double x_s, y_s;       // Soll-Koordinaten im aktuellen Schritt i
    double x_s1, y_s1;     // Soll-Koordinaten im vorherigen Schritt i-1
    double vx_s, vy_s;     // Soll-Bahngeschwindigkeit in Schritt i
    double vx_s1, vy_s1;   // Soll-Bahngeschwindigkeit in Schritt i-1
    double ax_s=0, ay_s=0; // Soll-Bahnbeschleunigung in Schritt i
    double ax, ay;         // Regelgrößen in Schritt i
    double a1, a2;         // Zwischengrößen für Regelung im Schritt i
    double e_x, e_y;       // Fehler zwischen Soll- und Ist-Koordinaten
    double e_vx, e_vy;     // Fehler zwischen Soll- und Ist-Geschwindigkeiten
    const double kI=1.0;   // Regelparameter für Geschwindigkeitsfehler (entspricht P-Anteil)
    const double kII=2;  // Regelparameter für Positionsfehler (entspricht I-Anteil)
    string file;            // Datei mit Punkten der zu befahrenden Trajektorie
    double dt;             // Zeitdauer für das Durchlaufen der aktuellen Schleife
    ptime tref;             // Objekt der Klasse ptime zur Messung von dt

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.set_slip_const(0.0); // Schlupf des Roboters auf Null setzen

    robot.init_pull_mode();  // Aktuelle Werte aus Proxies lesen

	// Pfad zu Datei mit abgetasteter Trajektorie
	file = "AMS_Trajekt.txt";

	robot.draw_traject(file, 255, 0, 0);   // Trajektorie plotten

    ifstream infile(file.c_str());         // Datei mit Trajektorie öffnen
    infile >> x_s >> y_s;                  // Soll-Koordinaten des Startpunktes lesen
    infile >> x_s1 >> y_s1;                // Soll-Koordinaten des zweiten Punktes lesen

    // Position des Roboters initialisieren
    x = x_s;
    y = y_s;

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    theta = atan2(y_s1 - y_s, x_s1 -x_s);
    robot.set_sim_pos(x, y, theta);

    vx_s1 = (x_s1 - x_s)/T;
    vy_s1 = (y_s1 - y_s)/T;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while( infile >> x_s >> y_s ) { // Schleife über alle Punkte der Trajektorie

        tref = microsec_clock::local_time(); // Referenzzeit zum Messen der Schleifendurchlaufzeit dt

        robot.wait_for_new_data(); // Neue Daten aus Proxies einlesen

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

        v = robot.get_v();
        w = robot.get_w();
        delta = v*T;
        phi = w*T;

        x = x + delta*cos(theta + phi/2);
        y = y + delta*sin(theta + phi/2);
        theta = theta + phi;

        vx = v * cos(theta);
        vy = v * sin(theta);

        vx_s = (x_s - x_s1)/T;
        vy_s = (y_s - y_s1)/T;
        ax_s = (vx_s - vx_s1)/T;
        ay_s = (vy_s - vy_s1)/T;

        e_x = x_s - x;
        e_y = y_s - y;
        e_vx = vx_s - vx;
        e_vy = vy_s - vy;

        ax = ax_s + kI*e_vx + kII*e_x;
        ay = ay_s + kI*e_vy + kII*e_y;
        a1 = ax*cos(theta) + ay*sin(theta);
        a2 = -ax*sin(theta) + ay*cos(theta);
        v_st = v_st + a1*T;
        w_st = a2/v_st;

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        robot.set_speed(v_st, w_st); // Einprägen der Führungsgrößen

        // Warten um insgesamt die Abtastzeit T zwischen zwei aufeinanderfolgenden Schritten
        // Dabei Abbruch, falls Berechnungszeit pro Schleifendurchlauf Abtastzeit überschreitet
        dt = (double)(microsec_clock::local_time()-tref).total_milliseconds()/1000;
        if( dt > T ) {
            printf("Fehler! Abtastzeit zu kurz: dt=%.3lf T=%.3lf --> Abbruch!\n", dt, T);
            robot.stop();  // stoppen
            while(1);     // warten
        }
        usleep((T-dt)*1000000); // Übergabe in Microsekunden

        // Soll-Werte für nächsten Schleifendurchgang umkopieren
        x_s1 = x_s;
        y_s1 = y_s;
        vx_s1 = vx_s;
        vy_s1 = vy_s;
    }

    robot.stop(); // stoppen

    while(1); // Endlosschleife
    return 0;
}
