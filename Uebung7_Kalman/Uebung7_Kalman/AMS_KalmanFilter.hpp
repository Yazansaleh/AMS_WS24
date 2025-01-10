/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
// #include "AMS_WallMap.hpp"    // Zur Suche in bekannten Kartenkonturen nach aktuell gemessener Kontur
#include <iostream>
#include "newmat/newmatio.h"

using namespace AMS;
using namespace std;

class KalmanFilter
{
private:
    AMS_Robot* robotp;          // Zeiger auf Roboterprojekt; wird dem Konstruktor übergeben
    double b;                   // Abstand der Räder vom kinematischen Zentrum des Roboters [m]
    double ks;                  // Schlupfkonstante zur Berechnung der Varianz der Roboterbewegung in [m]
    Matrix D;                   // Matrix mit Kinematik des Differenzialantriebs
    Matrix P;                   // Kovarianzmatrix der Zustandsvariablen des Roboters
    int pt_count;               // Anzahl der Ellipsenpunkte
	player_point_2d_t* ellipse; // Zeiger auf Objekt zum Speichern der Fehlerellipse
    uint8_t red, green, blue;   // Farbe der Fehlerellipse
//    WallMap map;                // Objekt zum Speichern von und zur Identifikation von Wänden (erst für Aufgabe 7 relevant)

public:
    // Konstruktor
    KalmanFilter(AMS_Robot* robotpointer);
    // Plotten der Fehlerellipse um aktuelle Roboterposition
    void PlotEllipse(double xm, double ym);
    // Prädiktion der Systemkovarianzmatrix
    void PredictCov(double theta, double delta, double phi);
    // Korrektur der Roboterposition (erst für Aufgabe 8 relevant)
    bool Correction(double& x, double& y, double& theta);
};
