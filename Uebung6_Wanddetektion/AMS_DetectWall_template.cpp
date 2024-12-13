/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "newmat/newmatio.h" // Input/output für Newmat

using namespace AMS;
using namespace PlayerCc;
using namespace std;
namespace po = boost::program_options;

AMS_Robot robot;

int main(int argc, char **argv)
{
    double *scan;                   // Zeiger auf aktuellen Scan
    int LUT[360];                   // Look-Up-Table mit Indizes zu allen relevanten Messwerten
    double MaxRange;                // Maximale Messdistanz des Entfernungssensors in m
    double sigma_r;                 // Standardabweichung des Entfernungssensors in m
    int N=0;                        // Anzahl der relevanten Messungen im Scan
    double PhiR;                    // Normalenwinkel der Regressionsgeraden vom Roboter aus
    double dR;                      // Normalenabstand der Regressionsgeraden von der Roboterposition
    double var_rho=0;               // Varianz des Abstands der Messpunkte von der Regressionsgeraden
    int i;                          // Laufvariable, gleichzeitig Winkel in deg
    double x, y, theta;             // Roboterkoordinaten
    double Phi;                     // globaler Winkel der gefundenen Wand
    double d;                       // globaler Abstand der gefundenen Wand
    double xi;                      // x-Koordinate des aktuellen Messpunktes
    double yi;                      // y-Koordinate des aktuellen Messpunktes
    double m_x=0;                   // Mittelwert der x-Koordinaten der Messpunkte
    double m_y=0;                   // Mittelwert der y-Koordinaten der Messpunkte
    double var_x=0;                 // Varianz der x-Koordinaten der Messpunkte
    double var_y=0;                 // Varianz der y-Koordinaten der Messpunkte
    double cov_xy=0;                // Kovarianz zwischen x- und y-Kordinaten der Messpunkte

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.init_push_mode();  // Daten von "robot" aus Warteschlange lesen
    robot.wait_for_new_data();
    //robot.set_sim_pos(1, -2, 0);

    robot.set_sigma_ranger(0.01);

    robot.get_scan(scan, MaxRange, sigma_r); // Messwerte, max. Messdistanz und radiale Standardabweichung auslesen

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Bestimmung der Anzahl der relevanten Messwerte und Besetzen der Look-Up-Table
    for(i = 0; i < 360; i++)
    {
        if(scan[i] > 0 && scan[i] <= MaxRange)
        {
            LUT[N] = i;
            N++;
        }
    }

    // Rücksprung, falls keine Messwerte vorhanden sind
    if(N == 0)
    {
        robot.log.error("Keine Messwerte vorhanden");
        return 0;
    }
    // Berechnung des Normalenwinkels der Regressionsgeraden
    double ri, theta_i;
    for(int j = 0; j < N; j++)
    {
        ri = scan[LUT[j]];
        theta_i = LUT[j]*M_PI/180;
        xi = ri*cos(theta_i);
        yi = ri*sin(theta_i);
        m_x += xi;
        m_y += yi;
    }
    m_x /= N;
    m_y /= N;

    for(int j = 0; j < N; j++)
    {
        ri = scan[LUT[j]];
        theta_i = LUT[j]*M_PI/180;
        xi = ri*cos(theta_i);
        yi = ri*sin(theta_i);
        var_x += (xi*xi - m_x*m_x);
        var_y += (yi*yi - m_y*m_y);
        cov_xy += (xi*yi - m_x*m_y);
    }
    var_x /= N;
    //var_x = sqrt(var_x);
    var_y /= N;
    //var_y = sqrt(var_y);
    cov_xy /= N;
    PhiR = 0.5*atan2(-2*cov_xy, var_y - var_x);

    // Berechnung des Normalenabstands der Regressionsgeraden
    dR = m_x*cos(PhiR) + m_y*sin(PhiR);

    // Korrektur bei negativem Normalenabstand
    if(dR < 0)
    {
        dR = abs(dR);
        PhiR -= M_PI;
    }

    // Berechnung des mittleren quadratischen Fehlers mit Rücksprung bei zu großem Fehler
    double rho[360];
    double rho_max = scan[LUT[0]];

    int idx_max   = 0;
    while(var_rho > 0.0001)
    {
        var_rho = 0;
        m_x = 0;
        m_y = 0;
        var_x = 0;
        var_y = 0;
        cov_xy = 0;
        for(int j = 0; j < N; j++)
        {
            ri = scan[LUT[j]];
            theta_i = LUT[j]*M_PI/180;
            xi = ri*cos(theta_i);
            yi = ri*sin(theta_i);
            rho[j] = (xi*cos(PhiR) + yi*sin(PhiR) - d);
            //rho_i = rho_i * rho_i;
            var_rho += rho[j]*rho[j];
            if(rho[j] > rho_max)
            {
                rho_max = rho[j];
                idx_max = j;
            }
        }
        N = idx_max - 10;

        for(int j = 0; j < N; j++)
        {
            ri = scan[LUT[j]];
            theta_i = LUT[j]*M_PI/180;
            xi = ri*cos(theta_i);
            yi = ri*sin(theta_i);
            m_x += xi;
            m_y += yi;
        }
        m_x /= N;
        m_y /= N;

        for(int j = 0; j < N; j++)
        {
            ri = scan[LUT[j]];
            theta_i = LUT[j]*M_PI/180;
            xi = ri*cos(theta_i);
            yi = ri*sin(theta_i);
            var_x += (xi*xi - m_x*m_x);
            var_y += (yi*yi - m_y*m_y);
            cov_xy += (xi*yi - m_x*m_y);
        }
        var_x /= N;
        //var_x = sqrt(var_x);
        var_y /= N;
        //var_y = sqrt(var_y);
        cov_xy /= N;
        PhiR = 0.5*atan2(-2*cov_xy, var_y - var_x);

        // Berechnung des Normalenabstands der Regressionsgeraden
        dR = m_x*cos(PhiR) + m_y*sin(PhiR);

        // Korrektur bei negativem Normalenabstand
        if(dR < 0)
        {
            dR = abs(dR);
            PhiR -= M_PI;
        }

        var_rho /= N;
        var_rho = sqrt(var_rho);
    }


    if(var_rho > sigma_r)
    {
        robot.log.alert("Keine Signifikante Gerade erkannt");
        return 1;
    }

    // Globale Koordinaten der gefundenen linearen Kontur bestimmen
    robot.get_sim_pos(x, y, theta);
    Phi = theta + PhiR;
    d = dR + x*cos(Phi) + y*sin(Phi);
    if(d < 0)
    {
        d = abs(d);
        Phi -= M_PI;
    }

    //robot.log.info("Roboter POsition: x = %.2lf, y = %.2lf, theta = %.2lf", x, y, theta*180/M_PI);
    //robot.log.info("Erkannte Gerade gegenüber Roboter-Position: Abstand %.2lfm, Winkel: %.2lf", dR, PhiR*180/M_PI);
    robot.log.info("Anzahl Messpubnkte N: %i", N);
    robot.log.info("Erkannte Gerade global: Abstand %.2lfm, Winkel: %.2lf°, Fehler: %.6lf", d, Phi*180/M_PI, var_rho);

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
