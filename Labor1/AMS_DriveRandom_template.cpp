/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"

using namespace PlayerCc;
using namespace AMS;
using namespace std;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    AMS_Robot robot;      	// Roboterobjekt

	double min_right;       // minimaler Abstand zu Hindernis im rechten Sektor in [m]
	double min_left;        // minimaler Abstand zu Hindernis im linken Sektor in [m]
	double t;               // aktueller Zeitstempel in [s]
	double t_last = 0;      // gespeicherter Zeitstempel vom letzten Durchlauf in [s]
	double v = 0;           // aktuelle Bahngeschwindigkeit in [m/s]
	double w = 0;           // aktuelle Winkelgeschwindigkeit in [rad/s]
	double av;              // konstante Bahnbeschleunigung in [m/s²]
	double aw;              // konstante Winkelbeschleunigung in [rad/s²]
    double v_max;           // maximale Bahngeschwindigkeit in [m/s]
    double w_max;           // maximale Winkelgeschwindigkeit in  [rad/s]
	double v_soll = 0;      // aktueller Zielwert für Bahngeschwindigkeit in [m/s]
	double w_soll = 0;      // aktueller Zielwert für Winkelgeschwindigkeit in [rad/s]
	double StopLength;      // Stoppdistanz des Roboters bei maximaler Beschleunigung in [m]
    double TurnDist;        // zulässiger minimaler Abstand von Hindernissen in [m] ohne dass Drehung erfolgt
    double ScanWidth;       // opening angle that is used in [rad]

	// Benötigte Konfigurationsparameter in Roboterobjekt ergänzen
	robot.options.add_options()
		("scanwidth,sw",po::value<uint32_t>()->default_value(80),"Used width of the laser scan [deg].")
		("turndist,td",po::value<double>()->default_value(1.0),"Minimal distance leading to turns [m].");

    // Parameter aus Kommandozeile lesen und Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    // Benötigte Konfigurationsparamter aus Roboterobjekt in lokale Variablen kopieren
	ScanWidth = dtor(robot.config["scanwidth"].as<uint32_t>());
	TurnDist = robot.config["turndist"].as<double>();

    robot.init_pull_mode();    // Pull-Mode: Proxies fordern aktuelle Werte an

    // Beschleunigungen und maximale Geschwindigkeiten aus Proxies einlesen
	av = robot.get_vacc();
	aw = robot.get_wacc();
    v_max = robot.get_vmax();
    w_max = robot.get_wmax();

	// Endlosschleife
	while(true)	{

        robot.wait_for_new_data(); // Auf neue Daten in Proxies warten

        // Aktuelle Stoppdistanz berechnen
        StopLength = pow(robot.get_v(),2) / (2.0*av);

        // Entfernungen zu nächsten Hindernissen im rechten und linken Sektor bestimmen
        robot.get_min_range(-ScanWidth/2, 0.0, true, &min_left, NULL);
	    robot.get_min_range(0.0, ScanWidth/2, true, &min_right, NULL);

        // Zeichnen des aktuellen Scans mit Anzeige von StopLength und TurnDist
        robot.clear_graphics();  // vorherigen Plot löschen
        robot.draw_circle_seg(-ScanWidth/2, ScanWidth/2, TurnDist, 0.105, 0, 0, 255, 0, 'r');
        robot.draw_circle_seg(-ScanWidth/2, ScanWidth/2, StopLength, 0.105, 0, 255, 0, 0, 'r');

        // Sollwert für Bahngeschwindigkeit auf v_max setzen, falls nächstes Hindernis weit genug entfernt ist
        if( MIN(min_left,min_right) > StopLength + 0.6) {
            v_soll = v_max;
            w_soll = 0; // etwaige Drehbewegung beenden
        }
        else  // andernfalls bremsen
            v_soll = 0;

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/


double tmp, tmp2;
        int direction = 0;  // 0: keine Drehung, -1: links drehen, 1: rechts drehen

        // Bestimmen, in welche Richtung gedreht werden soll, basierend auf dem nächsten Hindernis
        if (min_left > min_right && min_right < TurnDist) {
            direction = -1;  // Links drehen, wenn rechts ein Hindernis näher ist
            tmp = min_right - StopLength;
        } else if (min_left < TurnDist) {
            direction = 1;   // Rechts drehen, wenn links ein Hindernis näher ist
            tmp = min_left - StopLength;
        }

        // Proportionale Drehgeschwindigkeit berechnen
        if (direction != 0) {  // Nur wenn eine Drehung nötig ist
            tmp2 = TurnDist - StopLength;
            if (tmp <= 0 || tmp2 <= 0) {
                w_soll = direction * w_max;  // Scharf drehen, wenn zu nah
            }  else if (min_left == min_right){
            w_soll = direction * 1.5 * w_max;  // Scharf drehen, wenn zu nah und min_left == min_right
            } else {
                w_soll = direction * w_max * (1 - tmp / tmp2);  // Proportional drehen
            }

        }
        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

		t = robot.get_t();  // aktuelle Zeit lesen

        // Geschwindigkeiten mit konstanten Beschleunigungen an Sollwerte anpassen
        v += (v_soll >= v ? av : -av) * (t-t_last);
        w += (w_soll >= w ? aw : -aw) * (t-t_last);

        // Geschwindigkeiten auf Maximalwerte begrenzen
        v = limit(v, -v_max, v_max);
        w = limit(w, -w_max, w_max);

        t_last = t; // Aktuelle Zeit als Referenz für nächsten Zeitschritt speichern

        robot.set_speed(v, w); // Setzen der aktuellen Werte für Bahn- und Winkelgeschwindigkeit
	}
	return 0;
}

