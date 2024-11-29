/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "main.hpp"

using namespace AMS;
using namespace PlayerCc; // dtor, rtod, limit, normalize

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

    sb = vmax*vmax/(2*vacc);
    tb = vmax/vacc;
    tc = (L1 + 2*LK + L2 - 2*sb)/vmax;
    tbK = LK/vmax;

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
    double t_now;
    double t_last = 0;
    t0K = t0 + tb + (L1 - sb) / vmax;
    double wacc = robotp->get_wacc();
    do {
        robotp->wait_for_new_data();
        t_now = robotp->get_t();
        dt = t_now - t0 - tb; // Bewegungszeit updaten

        if ((t_now >= t0K) && (t_now <= t0K + tbK)) {
            // Winkelgeschwindigkeit erhöhen (Beschleunigungsphase)
            w = (t_now - t0K) * (wmax / tbK);
        } else if ((t_now > t0K + tbK) && (t_now <= t0K + 2 * tbK)) {
            // Winkelgeschwindigkeit reduzieren (Abbremsphase)
            w = (2 * tbK - (t_now - t0K)) * (wmax / tbK);
        } else {
            // Geradeaus fahren (Winkelgeschwindigkeit = 0)
            w = 0;
        }

        robotp->set_speed(vmax, w * turndir);
        t_last = t_now;
    }

    while (dt < tc);


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
