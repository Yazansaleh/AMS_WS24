/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer, M.Eng. Thimo Langbehn

#include "AMS_Robot.hpp"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <fstream>  // für Darstellung von Matrizen
#include <sstream>
#include <time.h>   // zur Initialisierung von srand()
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/SimpleLayout.hh>
#include <libplayerc++/utility.h>
#include <boost/numeric/ublas/io.hpp>   // io für Typen in matrix.hpp
#include "boost/date_time/posix_time/posix_time.hpp" // Für Zeitmessungen
#include <boost/numeric/ublas/vector.hpp> // Vektorrechnung
#include "newmat/newmatio.h" // Matrizenrechnung

namespace po = boost::program_options;
namespace ub = boost::numeric::ublas;
using namespace boost::numeric::ublas;
using namespace std;
using namespace PlayerCc;

/************************************************************************/
AMS::AMS_Robot::AMS_Robot(const string name)
    :log(log4cpp::Category::getInstance("name")),
	 options("Custom Options"),
	 client(NULL), drive(NULL), ranger(NULL),
	 got_first_valid_data_(false),
	 set_first_speed_(false)
{
    time_t t;
    time(&t);
    srand((unsigned int)t);   // Zufallsgenerator initialisieren
	log.setPriority(log4cpp::Priority::INFO);

    for( int i=0; i<4; i++) // Kovarianzmatrizen im Feld walls initialisieren
        (this->walls[i].R).ReSize(2);
}

/************************************************************************/
AMS::AMS_Robot::~AMS_Robot()
{
	try {
		if (NULL != drive) {
			drive->SetSpeed(0,0);
			drive->SetMotorEnable (false);
		}
	}
	catch (PlayerError & e) {
		log.errorStream() << "~AMS_Robot():" << e;
	}
	catch (...) { //  Destructor MUST not propagate exceptions
		log.fatal("~AMS_Robot(): Got unkonwn Exception!");
	}
	delete_clients();
	log4cpp::Category::shutdown();
}

/************************************************************************/
bool AMS::AMS_Robot::read_config(int argc, char** argv)
{
	po::options_description cmdline("Commandline Options");
	cmdline.add_options()
		("help,h", "produce help message")
		("config,c", po::value<string>(), "load options from a config file");

	po::options_description configopts("Configuration");
	configopts.add_options()
		("host,H", po::value<string>()->default_value(PLAYER_HOSTNAME),"Hostname to connect to")
		("port,P", po::value<uint32_t>()->default_value(PLAYER_PORTNUM),"Port where player listens")
		("logfile,l", po::value<string>(),"Write output to file [path]")
		("quiet,q", po::bool_switch(),"Suppress output to console")
		("simulationindex,si", po::value<uint32_t>()->default_value(0),"Simulation device index")
		("rangerindex,r", po::value<uint32_t>()->default_value(0),"Ranger device index")
		("positionindex,p", po::value<uint32_t>()->default_value(0),"Position2d device index")
		("graphicsindex,g", po::value<int32_t>()->default_value(0),"Graphics2d device index related to robot, -1 disables usage")
		("graphmapindex,gm", po::value<int32_t>()->default_value(1),"Graphics2d device index related to map, -1 disables usage")
		("urate,u", po::value<uint32_t>()->default_value(10),"Set server update rate [Hz]")
		("vmax,v", po::value<double>(&vmax_)->default_value(40),"Maximum speed in [cm/s]")
		("vaccel,va", po::value<double>(&vaccel_)->default_value(20),"Speed acceleration in [cm/s^2]")
		("wmax,w", po::value<double>()->default_value(40),"Maximum angular velocity in [deg/s]")
		("waccel,wa", po::value<double>()->default_value(20),"Angular acceleration in [deg/s^2]")
		("wheel_dist,wd", po::value<double>(&wheel_dist)->default_value(0.6),"Distance of both wheels of differential drive from kinematic center [m]")
		("slip_const,sc", po::value<double>(&slip_const)->default_value(0.005),"Slipping constant used for robot move and turn in [m/s]")
		("sigma_ranger,sr", po::value<double>(&sigma_ranger)->default_value(0.1),"Standard deviation of the ranger measurements in [m]");

	stringstream fmt_buffer;
	fmt_buffer << "Set server delivery mode\n  " <<
		PLAYER_DATAMODE_PUSH << " : Push\n  " <<
		PLAYER_DATAMODE_PULL << " : Pull";
	configopts.add_options()(
		"datamode, dm",
		po::value<uint32_t>()->default_value(PLAYER_DATAMODE_PUSH),
		fmt_buffer.str().c_str());

	fmt_buffer.str("");
	fmt_buffer << "Debug message level" <<
		"\n  0 : Debug" <<
		"\n  1 : Trace" <<
		"\n  2 : Commands" <<
		"\n  3 : Calculation";
	configopts.add_options()(
		"debug,d",
		po::value<int>()->default_value(-1),
		fmt_buffer.str().c_str());

	// transitive add
	configopts.add(options);
	cmdline.add(configopts);

	po::store(po::parse_command_line(argc, argv, cmdline), config);
	if (config.count("help")) {
		cout << cmdline << "\n";
		return false;
	}
	if (config.count("config")) {
		ifstream configfile(config["config"].as<string>().c_str());
		po::store(po::parse_config_file(configfile, configopts), config);
	}
	po::notify(config);

	apply_config(config);

	// Maximale Bahngeschwindigkeit, -beschleunigung und Standardabweichung in SI-Einheiten [m/s] umwandeln
	vmax_ /= 100;
	vaccel_ /= 100;

    // Überprüfung des Radabstandes
	if (wheel_dist == 0) {
		log.error("Wheel dist must be not zero!");
		return false;
		}

	log.log(AMS::Debug::TRACE, "Finished reading configuration.");
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::connect()
{
	try {
		client = new PlayerClient(config["host"].as<string>(), config["port"].as<uint32_t>());
		if (NULL == client) {
			log.fatal("Failed to connect to the robot player server.");
			return false;
		}
		client->SetDataMode(config["datamode"].as<uint32_t>());
		log.debugStream() << client;
        if( config["datamode"].as<uint32_t>() == PLAYER_DATAMODE_PUSH ) {
            // Set DataMode to PUSH for reading new data from message queue
            client->SetDataMode(PLAYER_DATAMODE_PUSH);
            // Deactivate replace rule in order to store all data in queue
            client->SetReplaceRule(false, PLAYER_MSGTYPE_DATA, -1, -1);
        }
        else {
            // Set DataMode to "pull" in order to ensure always latest data
            client->SetDataMode(PLAYER_DATAMODE_PULL);
            // Set replace rule for any data to avoid overflow of message queue
            client->SetReplaceRule(true, PLAYER_MSGTYPE_DATA, -1, -1);
        }

		stage = new SimulationProxy(client, config["simulationindex"].as<uint32_t>());
		if (NULL == stage) {
			log.fatal("Failed to access the simulation controller.");
			delete_clients();
			return false;
		}

		drive = new Position2dProxy(client, config["positionindex"].as<uint32_t>());
		if (NULL == drive) {
			log.fatal("Failed to access the drive controller.");
			delete_clients();
			return false;
		}
		log.debugStream() << drive;

        t_last = microsec_clock::local_time(); // Referenzzeit zum Messen der Verzögerungszeit dt

		ranger = new RangerProxy(client, config["rangerindex"].as<uint32_t>());
		if (NULL == ranger) {
			log.fatal("Failed to access the ranger sensor.");
			delete_clients();
			return false;
		}
		log.debugStream() << *ranger;
		ranger->RequestConfigure();

		graphics = NULL;
		if (0 <= config["graphicsindex"].as<int32_t>()) {
			try {
				graphics = new Graphics2dProxy(client, config["graphicsindex"].as<int32_t>());
			}
			catch (PlayerError& pe) {
				graphics = NULL;
			}
			if (NULL == graphics) {
				log.info("No graphics related to robot available, disabling. "\
					"Use -1 as index to deactivate "\
					"graphics on the start.");
			}
		}
		graphmap = NULL;
		if (0 <= config["graphmapindex"].as<int32_t>()) {
			try {
				graphmap = new Graphics2dProxy(client, config["graphmapindex"].as<int32_t>());
			}
			catch (PlayerError& pe) {
				graphmap = NULL;
			}
			if (NULL == graphmap) {
				log.info("No graphics related to map available, disabling. "\
					"Use -1 as index to deactivate "\
					"graphics on the start.");
			}
		}
	}
	catch (PlayerError& e)
	{
		log.errorStream() << "Exception in AMS_Robot.connect():\n" << e;
		delete_clients();
		return false;
	}
	log.log(AMS::Debug::TRACE, "Connection to robot established.");
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::get_sim_pos(double &x, double &y, double &theta)
{
    try {
        stage->GetPose2d( (char *)string("AMS-Robby").c_str(), x, y, theta);
    }
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.get_sim_pos():\n" << e;
        return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::set_sim_pos(double x, double y, double theta)
{
    try {
        stage->SetPose2d( (char *)string("AMS-Robby").c_str(), x, y, theta);
    }
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.set_sim_pos():\n" << e;
        return false;
	}
	return true;
}

/************************************************************************/
void AMS::AMS_Robot::wait_for_new_data()
{
    // Wait for new data
    client->Read();
}

/************************************************************************/
bool AMS::AMS_Robot::init_push_mode()
{
    try {
        // Set DataMode to PUSH for reading new data from message queue
        client->SetDataMode(PLAYER_DATAMODE_PUSH);
        // Deactivate replace rule in order to store all data in queue
        client->SetReplaceRule(false, PLAYER_MSGTYPE_DATA, -1, -1);
        // Empty message queue
        while(client->Peek())
             client->Read();
    }
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.init_push_mode():\n" << e;
        return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::init_pull_mode()
{
    try {
        // Set DataMode to "pull" in order to ensure always latest data
        client->SetDataMode(PLAYER_DATAMODE_PULL);
        // Set replace rule for any data to avoid overflow of message queue
        client->SetReplaceRule(true, PLAYER_MSGTYPE_DATA, -1, -1);
        // Empty message queue
        while(client->Peek())
             client->Read();
    }
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.init_pull_mode():\n" << e;
        return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::move_dist(double dist)
{
    double sb;           // Beschleunigungsweg (= Bremsweg)
    double tb;           // Beschleunigungszeit
    double tc;           // Fahrtzeit mit vmax_
    double vm;           // vorzeichenrichtige effektive Maximalgeschwindigkeit
    double a = vaccel_;  // vorzeichenrichtige Beschleunigung
    double dt;           // aktuelle Bewegungszeit
    double t0;           // Startzeitpunkt
    double x0;           // initiale x-Ortskoordinate
    double y0;           // initiale y-Ortskoordinate

    init_push_mode();  // data shall be read from message queue

    sb = vmax_*vmax_/(2*vaccel_);
    if (fabs(dist) > 2*sb) {  // Maximalgeschwindigkeit vmax_ wird erreicht
        tb = vmax_/vaccel_; // Zeit bis zum Erreichen von vmax_
        vm = vmax_;
        tc = (fabs(dist)-sb*2)/vm;
    }
    else {              // Roboter muss bereits vor Erreichen von vmax_ abbremsen
        tb = sqrt(fabs(dist)/vaccel_);    // Zeit bis zum Erreichen der maximal möglichen Geschwindigkeit
        vm = tb*vaccel_;    // Berechnung der verringerten Maximalgeschwindigkeit
        tc = 0;
    }
    // Vorzeichen beachten
    if( dist < 0) {
        vm = -vm;
        a = -a;
    }
	try {
        // Beschleunigung während tb
        wait_for_new_data();
        x0 = get_x();
        y0 = get_y();
        t0 = get_t(); // aktuelle Bewegungszeit
        do {
            wait_for_new_data();
            dt = get_t()-t0; // aktuelle Bewegungszeit
            set_speed(a*dt, 0);
        }
        while( dt < tb);
        // Ggf. gleichmäßige Bewegung während tc, falls Strecke lang genug
        if( tc > 0 ) {
            wait_for_new_data();
            t0 = get_t(); // aktuelle Bewegungszeit
            set_speed(vm, 0);
            do {
                wait_for_new_data();
                dt = get_t()-t0; // aktuelle Bewegungszeit
            }
            while( dt < tc);
        }
        // Abbremsen während tb
        wait_for_new_data();
        t0 = get_t(); // aktuelle Bewegungszeit
        do {
            wait_for_new_data();
            dt = get_t()-t0; // aktuelle Bewegungszeit
            set_speed(vm-a*dt, 0);
        }
        while( dt < tb);
        // Stoppen
        set_speed(0, 0);
		log.info("Driven distance: %.3lf m.", sqrt(pow(get_x()-x0, 2) + pow(get_y()-y0, 2)) );
        return true;
	}
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.move_dist():\n" << e;
        return false;
	}
}

/************************************************************************/
bool AMS::AMS_Robot::turn_angle(double angle)
{
    double phib;         // Beschleunigungswinkel (= Bremswinkel)
    double tb;           // Beschleunigungszeit
    double tc;           // Drehzeitzeit mit wmax_
    double wm;           // vorzeichenrichtige effektive Maximalwinkelgeschwindigkeit
    double wa = waccel_; // vorzeichenrichtige Winkelbeschleunigung
    double dt;           // aktuelle Drehzeit
    double t0;           // Startzeitpunkt
    double theta0;      // Initiale Ausrichtung

    init_push_mode();  // data shall be read from message queue

    phib = wmax_*wmax_/(2*waccel_);
    if (fabs(angle) > 2*phib) {  // Maximalwinkelgeschwindigkeit wmax_ wird erreicht?
        tb = wmax_/waccel_;     // Zeit bis zum Erreichen von wmax_
        wm = wmax_;
        tc = (fabs(angle)-phib*2)/wm;
    }
    else {              // Drehung muss bereits vor Erreichen von wmax_ abgebremst werden
        tb = sqrt(fabs(angle)/waccel_);    // Zeit bis zum Erreichen der maximal möglichen Winkelgeschwindigkeit
        wm = tb*waccel_;    // Berechnung der verringerten Maximalwinkelgeschwindigkeit
        tc = 0;
    }
    // Vorzeichen beachten
    if( angle < 0) {
     wm = -wm;
     wa = -wa;
    }
	try {
        // Beschleunigung wähend tb
        wait_for_new_data();
        theta0 = get_theta();
        t0 = get_t();        // Startzeit der Bewegung initialisieren
        do {
            wait_for_new_data();
            dt = get_t()-t0; // aktuelle Bewegungszeit
            set_speed(0, wa*dt);
        }
        while( dt < tb );
        // Ggf. gleichmäßige Bewegung während tc
        if( tc > 0 ) {
            wait_for_new_data();
            t0 = get_t();
            set_speed(0, wm);
            do {
                wait_for_new_data();
                dt = get_t()-t0;
            }
            while( dt < tc );
        }
        // Abbremsen während tb
        wait_for_new_data();
        t0 = get_t();        // Startzeit der Bewegung initialisieren
        do {
            wait_for_new_data();
            dt = get_t()-t0; // aktuelle Bewegungszeit
            set_speed(0, wm-wa*dt);
        }
        while( dt < tb );
        // Stoppen
        set_speed(0, 0);
		log.info("Rotated angle: %.2lf °.", rtod(normalize(get_theta()-theta0)));
        return true;
	}
	catch (PlayerError & e) {
		log.errorStream() << "Exception in AMS_Robot.turn_angle():\n" << e;
        return false;
	}
}

/************************************************************************/
double AMS::AMS_Robot::calc_angle(double dx, double dy)
{
    double angle = 0; // vorzeichenrichtiger Drehwinkel

    if( dx!=0 )
        angle = atan2(dy, dx) - get_theta();
    else if( dy!=0 )
        angle = (dy>0 ? M_PI/2 : -M_PI/2) - get_theta();
    // komplementären Drehwinkel verwenden, falls |angle| > pi
    if (angle > M_PI)
        angle = angle - 2*M_PI;
    else if (angle < -M_PI)
        angle = angle + 2*M_PI;
    return angle;
}

/************************************************************************/
bool AMS::AMS_Robot::drive_polyline(player_point_2d_t *P, int count)
{
    int i;
    double x1, y1; // Koordinaten der jeweils aktuellen Roboterposition
    double dx, dy; // Koordinatendifferenzen zum jeweils nächsten Punkt

    try {
        wait_for_new_data();
        x1 = get_x();  // Initialisierung mit Startposition
        y1 = get_y();

        for(i=0; i<count; i++) {
            client->Read();     // Auslesen der Proxies
            dx = P[i].px-x1;
            dy = P[i].py-y1;
            if( abs(dx)>0.01 || abs(dy)>0.01 ) {
                turn_angle(calc_angle(dx, dy));     // Neue Position andrehen
                move_dist(sqrt(dx*dx+dy*dy));       // Neue Position anfahren
                x1 = P[i].px;
                y1 = P[i].py;
            }
        }
        if( count==1 ) {    // Falls nur der erste Punkt angefahren werden soll, den Zweiten noch andrehen
            dx = P[i].px-x1;
            dy = P[i].py-y1;
            turn_angle(calc_angle(dx, dy));
        }
    }
    catch (PlayerError & e) {
        log.errorStream() <<"Exception in AMS_Robot.drive_points():\n" << e;
        return false;
    }
    return true;
}

/************************************************************************/
bool AMS::AMS_Robot::calc_traject(player_point_2d_t *P, int N, double dt_soll, string opt, string file)
{
    int i;
    ub::vector<double> vi0(2), vi1(2), vi2(2); // Vektoren zu den aktuellen Punkten i, i+1, i+2
    ub::vector<double> v1(2);       // Vektor vom aktuellen zum jeweils nächsten Punkt des Polygons
    ub::vector<double> v2(2);       // Vektor vom nächsten zum übernachsten Punkt des Polygons
    ub::vector<double> v_start(2);  // Vektor zum Startpunkt des nächsten krümmungsstetigen Übergangs
    ub::vector<double> v_end(2);    // Vektor zum Endpunkt des nächsten krümmungsstetigen Übergangs
    double L_v1, L_v2;          // Länge der Vektoren zwischen den Punkten i und i+1 sowie i+1 und i+2
    double delta_x=0;           // Abstand des Beginns des jeweils nächsten Übergangs vom Scheitelpunkt
    double LG;                  // Länge des nächsten Geradenstücks
    double L=0;                 // Länge der jeweils nächsten Klothoide
    double Asq=0;               // Krümmungsparameter für Klothoide
    double xL=0, yL=0;          // Parameter für Klothoide
    double alpha;               // Halber Winkel zwischen v1 und v2
    double x0=0;                // Parameter des krümmungsstetigen Übergangs mit cos-Funktion
    double A=0;                 // Parameter des krümmungsstetigen Übergangs mit cos-Funktion
    double phi_v1;              // Winkel des Vektors zwischen den aktuellen Punkten i und i+1
    double s, xs, ys;           // lokale Koordinaten
    double px, py;              // aktuelle mit hoher Auflösung berechnete Koordinaten des Übergangs
    double phi=0;               // Winkel zum Plotten der Klothoide
    player_point_2d_t P_xy[1];  // Koordinaten zum Plotten und Dateischreiben
    double sb;                  // Beschleunigungsweg
    double tb;                  // Beschleunigungszeit
    double v;                   // Geschwindigkeit, mit der Trajektorie durchfahren wird
    double ds;
    double ds_max;
    double R_min = vmax_/wmax_; // Radius der Klothoide im Scheitelpunkt

    // Abbruch, falls weniger als drei Punkte übergeben werden
    if( N < 3 ) {
        log.errorStream() << "Error in AMS_Robot.calc_traject(): Weniger als 3 Punkte übergeben\n";
        return false;
    }
    ofstream outfile(file.c_str());

    sb = 0.5*vmax_*vmax_/vaccel_;
    tb = vmax_/vaccel_;
    ds_max = vmax_*dt_soll;
    v = vmax_*dt_soll/tb;        //  Geschwindigkeit beim Erreichen des zweiten Punktes
    ds = v*dt_soll;              //  Abstand zwischen dem ersten und zweiten Punkt

    // Initialisierung zum Zeichnen des ersten Geradenstücks und für Abbruchkriterium
    v_end[0] = P[0].px;
    v_end[1] = P[0].py;

    // Ersten Punkt auf Trajektorie speichern
    P_xy[0] = P[0];
    outfile << P_xy[0].px << " " << P_xy[0].py << endl;

    for(i=0; i<N-2; i++) {   // Schleife über Eckpunkte des Polygons
        vi0(0) = P[i].px;
        vi0(1) = P[i].py;
        vi1(0) = P[i+1].px;
        vi1(1) = P[i+1].py;
        vi2(0) = P[i+2].px;
        vi2(1) = P[i+2].py;
        v1 = vi1 - vi0;
        v2 = vi2 - vi1;
        L_v1 = norm_2(v1); // Länge des Vektors zwischen den Punkten i und i+1
        L_v2 = norm_2(v2); // Länge des Vektors zwischen den Punkten i+1 und i+2

        phi_v1 = atan2(v1(1),v1(0));

        alpha = acos(inner_prod(v1,v2)/L_v1/L_v2)/2; // Skalarprodukt liefert betrag des Winkels

        // Parameter für krümmungsstetigen Übergang mittels cos-Funktion berechnen
        if( opt == "cos" ) {
            x0 = M_PI*R_min*tan(alpha)/2;
            A = R_min*tan(alpha)*tan(alpha);
            delta_x = x0 / cos(alpha); // Abstand vom jeweils nächsten Eckpunkt, ab dem die Krümmung beginnt
        }
        if( v1(0)*v2(1)-v1(1)*v2(0) < 0 ) {// z-Komponente des Vektorproduktes liefert Richtung des Winkels
            A = -A;
            alpha = -alpha;
        }

        // Alternativ Parameter für krümmungsstetigen Übergang mit Klothoide berechnen
        if( opt != "cos" ) {
            L = 2*fabs(alpha)*R_min;      // Länge der Klothoide bis Krümmungsscheitel
            Asq = 2*alpha*R_min*R_min;      // Parameter der Klothoide (Vorzeichen bestimmt Krümmungsrichtung)
            xL = L*(1 - alpha*alpha/10 + alpha*alpha*alpha*alpha/216);   // x-Koordinate des Scheitelpunkts der Klothoide
            yL = L*(alpha/3 - alpha*alpha*alpha/42 + alpha*alpha*alpha*alpha*alpha/1320);   // y-Koordinate des Scheitelpunkts der Klothoide
            delta_x = xL + tan(alpha)*yL;
        }

        // Abbruch, falls delta_x den maximal zulässigen Wert übersteigt, der durch den Abstand
        // vom jeweils nächsten Polygoneckpunkt bestimmt wird.
        // In diesem Fall können mit vorgegebenem Radius keine Übergänge berechnet werden
        if( delta_x > norm_2(vi1-v_end) ) {
            log.errorStream() << "Error in AMS_Robot.calc_traject(): Minimaler Radius zu groß für Polygon.\n";
            return false;
        }
        v_start = vi0 + v1*(L_v1-delta_x)/L_v1;

        // Punkte auf Geradenstücken zwischen den Übergängen zeichnen.
        // Dabei entspricht Endpunkt eines Übergangs jeweils dem Startpunkt der Geraden
        // und der Startpunkt des krümmungsstetigen Übergangs jeweils dem Endpunkt der Geraden
        LG = norm_2(v_start-v_end);
        for( s=0; s<LG; s+=ds) {
            P_xy[0].px += ds*cos(phi_v1);
            P_xy[0].py += ds*sin(phi_v1);
            outfile << P_xy[0].px << " " << P_xy[0].py << endl;
            // Während Anfangsbeschleunigung Abstände der Punkte vergößern
            if( i == 0 ) {
                v += vmax_*dt_soll/tb;
                v > vmax_ ? ds=ds_max : ds=v*dt_soll;
            }
        }

        v_end = vi1 + v2*(delta_x/L_v2);      // Endpunkt der Krümmung setzen

        // Zeichnen einer Klothoide als krümmungsstetigen Übergang
        if ( opt != "cos" ) {
            xs = 0;
            ys = 0;
            for(s=0; s<=L*2; s+=ds/10) { // Klothoide mit zehnfach höherer Auflösung berechnen
                if( s<=L )
                    phi = s*s/(Asq*2);      // 1. Hälfte: Zunahme der Krümmung
                else
                    phi = alpha*2 - (s-L*2)*(s-L*2)/(Asq*2); // 2. Hälfte: Abnahme der Krümmung
                xs += cos(phi)*ds/10;
                ys += sin(phi)*ds/10;
                px = v_start(0) + xs*cos(phi_v1) - ys*sin(phi_v1);
                py = v_start(1) + xs*sin(phi_v1) + ys*cos(phi_v1);
                // ds während Beschleunigungsphase vergößern
                if( i == 0 ) {
                    v += vmax_*dt_soll/tb;
                    v > vmax_ ? ds=ds_max : ds=v*dt_soll;
                }
                // Aktuellen Punkt nur zeichnen, falls hinreichend großer Abstand zum vorherigen Punkt
                if( (P_xy[0].px-px)*(P_xy[0].px-px)+(P_xy[0].py-py)*(P_xy[0].py-py)>ds*ds ) {
                    P_xy[0].px = px;
                    P_xy[0].py = py;
                    outfile << P_xy[0].px << " " << P_xy[0].py << endl;
                }
            }
        }
        else {  // Zeichnen einer cos-Funktion als krümmungsstetigen Übergang
            for(xs=-x0; xs<x0; xs+=ds/20) {
                ys = -A*cos(M_PI*xs/(2*x0));
                px = v_start(0) + (xs+x0)*cos(phi_v1+alpha) - ys*sin(phi_v1+alpha);
                py = v_start(1) + (xs+x0)*sin(phi_v1+alpha) + ys*cos(phi_v1+alpha);
                // Aktuellen Punkt nur zeichnen, falls hinreichend großer Abstand zum vorherigen Punkt
                if( (P_xy[0].px-px)*(P_xy[0].px-px)+(P_xy[0].py-py)*(P_xy[0].py-py)>ds*ds ) {
                    P_xy[0].px = px;
                    P_xy[0].py = py;
                    outfile << P_xy[0].px << " " << P_xy[0].py << endl;
                }
            }
        }
    }
    // Abbruch, falls letzter Polygonpunkt zu nahe liegt
    if( delta_x > norm_2(vi2-vi1) ) {
       log.errorStream() << "Error in AMS_Robot.calc_traject(): Gewählter minimaler Radius für Übergänge passt nicht zu letztem Punkt des Polygons\n";
       return false;
       }
    // Letztes Geradenstück bis Endpunkt berechnen und speichern
    v1 = vi2-v_end;
    phi_v1 = atan2(v1(1),v1(0));
    LG = norm_2(v1);
    if( LG < sb ) {
       log.errorStream() << "Error in AMS_Robot.calc_traject(): Letzte Gerade kürzer als Bremsweg\n";
       return false;
    }
    v = vmax_;
    for( s=0; s<LG; s+=ds) {
        P_xy[0].px += ds*cos(phi_v1);
        P_xy[0].py += ds*sin(phi_v1);
        outfile << P_xy[0].px << " " << P_xy[0].py << endl;
        // Am Ende Abstand der Punkte zum Bremsen verringern
        if( LG-s < sb ) {
            v -= vmax_*dt_soll/tb;
            v > 0 ? ds=v*dt_soll : s=LG;
        }
    }
    return true;
}

/************************************************************************/
bool AMS::AMS_Robot::draw_point(double x, double y, int r, int g, int b, char mode)
{
	// skip if either graphics or graphmap are not available
	if ( (mode=='r' && NULL==graphics) || (mode=='m' && NULL==graphmap) )
		return true;

    player_point_2d_t P_xy[1];  // Plotkoordinaten

    P_xy[0].px = x;
    P_xy[0].py = y;

	try {
	   if( mode=='r' ) {
            graphics->Color(r,g,b,0);
            graphics->DrawPoints(P_xy,1);
	   }
	   else {
            graphmap->Color(r,g,b,0);
            graphmap->DrawPoints(P_xy,1);
	   }
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.draw_point():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::draw_traject(string file, int r, int g, int b)
{
	// skip if no graphmap available
	if ( NULL==graphmap )
		return true;

    player_point_2d_t P_xy[1];  // Plotkoordinaten

	try {
        ifstream infile(file.c_str());
        graphmap->Color(r, g, b, 0);
        while( infile >> P_xy[0].px >> P_xy[0].py )
            graphmap->DrawPoints(P_xy,1);
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.draw_traject():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::clear_graphics()
{
	// skip if no graphics are available
	if (NULL == graphics)
		return true;

	try {
		graphics->Clear();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.clear_graphics():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::clear_graphmap()
{
	// skip if no graphics are available
	if (NULL == graphmap)
		return true;

	try {
		graphmap->Clear();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.clear_graphmap():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::draw_circle_seg(double astart, double astop,
            double radius, double dx, double dy, uint8_t r, uint8_t g, uint8_t b, char mode)
{
	// skip if no graphics are available
	if ( (mode=='r' && NULL==graphics) || (mode=='m' && NULL==graphmap) )
		return true;

	astart = normalize(astart);
	astop = normalize(astop);
	double angle = astop - astart;
	if (astop < astart)
		angle += 2*M_PI;
	// round down, thus getting the number of full degrees (can be 0).
	int seg_pt_count = static_cast<int>(rtod(angle));

	// the polygon starts with the center, then adds the segment
	// aproximation.
	// The segment starts with astart, in steps of 1 degree.
	// Add 1 for the astop point, and another 1 for the end center.

	player_point_2d_t* pts = new player_point_2d_t[2+seg_pt_count+2];

	pts[0].px = dx;
	pts[0].py = dy;
	// Calculate segment points excluding astop and both centers.
	double da = 0;
	for (int i=1; i < 2+seg_pt_count; ++i) {
		da = dtor(i);
		pts[i].px = cos(astart + da) * radius + dx;
		pts[i].py = sin(astart + da) * radius + dy;
	}
	pts[2+seg_pt_count+1 -1].px = cos(astop) * radius + dx;
	pts[2+seg_pt_count+1 -1].py = sin(astop) * radius + dy;
	pts[2+seg_pt_count+2 -1].px = dx;
	pts[2+seg_pt_count+2 -1].py = dy;

	try {
	   if( mode=='r' ) {
            graphics->Color(r,g,b,0);
            graphics->DrawPolyline(pts, 2+seg_pt_count+2);
	   }
	   else {
            graphmap->Color(r,g,b,0);
            graphmap->DrawPolyline(pts, 2+seg_pt_count+2);
	   }
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.draw_circle_seq():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::get_scan(double*& scan, double& MaxRange, double& sigma)
{
    int angle;
    int i;
    double range;
    double Xn;              // Normalverteilte Zufallsvariable mit E(x)=0 und var(x)=sigma_²

    scan = range_scan_;
    sigma = sigma_ranger;

	try {
        MaxRange = ranger->GetMaxRange();
        double MinAngle = rtod(ranger->GetMinAngle());
        double MaxAngle = rtod(ranger->GetMaxAngle());
        double AngRes = rtod(ranger->GetAngularRes());

        for( angle=-180; angle < 180; angle++ ) {
            if( angle>=rint(MinAngle) && angle<=rint(MaxAngle) ) {
                range = ranger->GetRange(rint((angle - MinAngle -0.5)/AngRes));
                if( range < MaxRange ) {
                    // Addition eines normalverteilten Fehlers mit Standardabweichung sigma_ranger
                    for( Xn=0, i=0; i<12; i++)
                        Xn += sigma*((double)rand()/RAND_MAX - 0.5);
                    range += Xn;
                }
                else
                    range = -1; // Anzeige, dass kein Hindernis in Reichweite
            }
            else
                range = 0;
            range_scan_[(angle+360)%360] = range;
        }
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_scan():\n" << e;
		return false;
	}
	return true;
}

/************************************************************************/
void AMS::AMS_Robot::draw_wall(int phi_start, int phi_stop, double* scan, int r, int g, int b)
{
    int i;
    double x, y;
    double x_exp, y_exp, th_exp;

    x_exp = get_x();
    y_exp = get_y();
    th_exp = get_theta();

    for( i = phi_start; i <= phi_stop; i++ ) {
        x = x_exp + scan[i%360]*cos(dtor(i)+th_exp);
        y = y_exp + scan[i%360]*sin(dtor(i)+th_exp);
        draw_point(x, y, r, g, b, 'm');
    }
}

/************************************************************************/
bool AMS::AMS_Robot::get_wall(int index, double& phiR, double& dR, SymmetricMatrix& R)
{
    if( index<1 || index>this->numb_walls )
        return false;

    phiR = this->walls[index-1].phi;
    dR = this->walls[index-1].dist;
    R = this->walls[index-1].R;

    return true;
}

/************************************************************************/
void AMS::AMS_Robot::calc_wall(int phi_1, int phi_2, double* scan, double& phiR, double& dR, double& MSE, int& phi_SE)
{
    int i;
    int N = phi_2 - phi_1;
    double xq=0, yq=0, vx=0, vy=0, cv=0;
    double x, y, r;
    double SE=0;
    double SEmax=0;
    bool flag = false;

    /// Algorithmus zur Berechnung des Winkels der Regressionsgeraden (siehe Vorlesung AMS)
    for( i=phi_1; i<phi_2; i++ ) {
        if( scan[i%360] <= 0 )
            N--;
        else {
            r = scan[i%360];
            x = cos(M_PI/180*i)*r;
            y = sin(M_PI/180*i)*r;
            xq += x;
            yq += y;
            vx += x*x;
            vy += y*y;
            cv += x*y;
        }
    }
    xq = xq/N;
    yq = yq/N;
    vx = vx/N - xq*xq;
    vy = vy/N - yq*yq;
    cv = cv/N - xq*yq;
    phiR = 0.5*atan2(-cv*2, vy-vx);

    /// Berechnung des Normalenabstands der Regressionsgeraden
    dR = xq*cos(phiR) + yq*sin(phiR);
    // Bei negativem Normalenabstand dessen Vorzeichen vertauschen und Normalenwinkel um Pi verändern
    if( dR < 0 ) {
        //cout << "d negativ!" << endl;
        dR = -dR;
        phiR <= M_PI ? phiR+=M_PI : phiR-=M_PI;
    }
    //cout << "Normalenwinkel aus Kontur: " << rtod(phiR) << endl;
    //cout << "Normalenabstand aus Kontur: " << dR << endl;

    /// Berechnung des mittleren quadratischen Fehlers und Ermittlung des Winkels
    /// zu dem Messpunkt mit dem größten quadratischen Fehlers (für Konturteilung)
    phi_SE = phi_1;
    MSE = 0;
    for( i=phi_1; i<phi_2; i++ ) {
        if( scan[i%360] <= 0 )
            N--;
        else {
            SE = pow(scan[i%360]*cos(M_PI/180*i-phiR)-dR, 2);
            if( !flag && SE<0.001 ) // 1. Minimum von SE --> Suche beginnen
                flag = true;
            if( flag && SEmax>0.005 && SE<0.001 ) // 2. Minimum von SE --> Suche beenden
                flag = false;
            if( flag && SE>SEmax ) {
                SEmax = SE;
                phi_SE = i;
            }
        }
        //cout << "calc_MSE: i=" << i << " SE=" << SE << " flag=" << flag << " SEmax=" << SEmax  << " phi_SE=" << phi_SE<< endl;
        MSE += SE;
    }
    MSE /= N;
     //cout << "MSE aus Kontur: " << MSE << endl;
     //cout << "phi_SE aus Kontur: " << phi_SE << endl;
}

/************************************************************************/
void AMS::AMS_Robot::store_wall(int phi_start, int phi_stop, double phiR, double dR, double MSE)
{
    double L;                       // Länge des erkannten Geradenstücks
    double xoff;                    // Offset zur Regressionsnormalen
    double var_Phi;                 // Varianz des Normalenwinkels der berechneten Geraden
    SymmetricMatrix Cov(2);
    int N = phi_stop - phi_start;
    double var_Phi_add;

    xoff = dR/2 * fabs(tan(phiR-dtor(phi_stop)) + tan(phiR-dtor(phi_start)));
    L = dR * fabs(tan(phiR-dtor(phi_stop)) - tan(phiR-dtor(phi_start)));
    var_Phi_add = xoff*xoff*0.0003 + 0.000005;
    var_Phi = MSE * 12/(L*L*N) + var_Phi_add;         // Varianz von Phi [rad]

    // Kovarianzmatrix besetzen
    Cov(1,1) = xoff*xoff*var_Phi + MSE/N + 0.001; // Varianz von d
    Cov(1,2) = -xoff * var_Phi;           // Kovarianz zwischen d und Phi
    Cov(2,2) = var_Phi;                   // Varianz von Phi
    //cout << this->numb_walls+1 << ". Wand: " << "sig_Phi_add= " << rtod(sqrt(var_Phi_add)) << "  xoff= " << xoff  << "  N= " << N   << "  L= " << L << "  MSE= " << MSE << endl;
    //cout << "sigma_d= " << sqrt(Cov(1,1))*100 << "cm  sigma_phi= " << rtod(sqrt(Cov(2,2))) << "°  corr= " << Cov(1,2)/sqrt(Cov(1,1)*Cov(2,2))  << endl;

    // store parameters in attributs
    this->walls[this->numb_walls].phi = phiR;
    this->walls[this->numb_walls].dist = dR;
    this->walls[this->numb_walls].R = Cov;
    this->walls[this->numb_walls].phi_start = phi_start;
    this->walls[this->numb_walls].phi_stop = phi_stop;
    this->numb_walls++;

    //cout << "Wand gespeichert:  phiR=" << rtod(phiR) << "° dR=" << dR << " sigma_Phi=" << rtod(sqrt(var_Phi)) << "°" << endl;
}

/************************************************************************/
int AMS::AMS_Robot::detect_wall(double& phi, double& dist, SymmetricMatrix& R, int N_min)
{
    double *scan;                   // Zeiger auf aktuellen Scan
    double MaxRange;                // Maximale Messdistanz in m
    double sigma;                   // Standardabweichung des Entfernungssensors
    int i;                          // Laufvariable, gleichzeitig Winkel in deg
    int phi_0;                      // erster Winkel ohne Hinderniskontakt
    int phi_start=-1;               // erster Winkel nach phi_0 eines Hindernisses
    int phi_stop=-1;                // erster Winkel nach phi_start ohne Hinderniskontakt
    int N=0;                        // Anzahl der relevanten Messungen der ersten Kontur im Scan
    double alpha, d;                // line parameters
    double MSE=0;                   // Mittlere quadratische Fehler der Regressionsgeraden
    double MSE_limit;               // maximal allowed MSE for a valid straigt line
    const double MSE_fix = 0.0002;
    int phi_SE;                     // Winkel mit maximalem quadratischen Fehler für Bestimming der Teilkonturen
    int i_max;                      // maximaler Index bei Suche nach Kontur
    int iter=0;                     // number of iterations
    int i_opt=0;                    // stores index of wall with minimum angular variance
    double var_alpha_min;           // minimum variance of alpha of any line

    this->numb_walls = 0; // initialize number of stored walls
    get_scan(scan, MaxRange, sigma); // aktuellen Scan einlesen
    /*for( i=0; i<360; i++) {
        printf("%.2lf ",scan[i]);
        if( (i+1)%10==0)
            printf("\n");
    }
    */
    /// Ermittlung der begrenzenden Winkel von zusammenhängenden Hinderniskonturen
    i = 1; // Startwinkel+1 für Suche nach 1. Kontur;
    i_max = 720; // maximaler Suchwinkel bei Suche nach 1. Kontur

    while( i<i_max && phi_stop<360 && this->numb_walls<MAX_WALLS) {
        iter++;
        phi_0 = -1;
        phi_start = -1;
        phi_stop = -1;
        for( i-=1; i<i_max && phi_stop==-1; i++ ) {
            if( phi_0==-1 && scan[i%360]<=0 )  // detect gap in contour
                phi_0 = i;
            if( phi_0>=0 && phi_start==-1 && i-phi_0>2 && scan[i%360]>0 )
                phi_start = i+1;
            if( phi_start>=0 && i-phi_start>2 && scan[i%360]<=0 )
                phi_stop = i-1;
            }
        N = phi_stop - phi_start;
        if( N>=N_min ) {
            //cout << iter << ". Kontur von " << phi_start << "° bis " << phi_stop << "°" << endl;
            /// Bei genügend großer Kontur darin nach Wand suchen
            calc_wall(phi_start, phi_stop, scan, alpha, d, MSE, phi_SE);
            MSE_limit = sigma*sigma + MSE_fix;
            //cout << "MSE=" << MSE << " MSE_limit=" << MSE_limit << endl;
            if( MSE <= MSE_limit  ) {
                store_wall(phi_start, phi_stop, alpha, d, MSE);
                //draw_wall(phi_start, phi_stop, scan, 255, 0, 0);
            }
            else {
                /// Aufteilung des Scans, falls keine Gerade aus Kontur ermittelt werden konnte
                N = phi_SE-1 - phi_start;
                if( N>N_min && this->numb_walls<MAX_WALLS ) { // 1. Teilkontur
                    //cout << "1. Teilkontur von " << phi_start << "° bis " << phi_SE-1 << "°" << endl;
                    calc_wall(phi_start, phi_SE-1, scan, alpha, d, MSE, phi_0);
                    MSE_limit = sigma*sigma + MSE_fix;
                    //cout << "MSE=" << MSE << " MSE_limit=" << MSE_limit << endl;
                    if( MSE <= MSE_limit  ) {
                        store_wall(phi_start, phi_SE-1, alpha, d, MSE);
                        //draw_wall(phi_start, phi_SE-1, scan, 255, 0, 0);
                    }
                }
                N = phi_stop - (phi_SE+1);
                if( N>N_min && this->numb_walls<MAX_WALLS ) { // 2. Teilkontur
                    //cout << "2. Teilkontur von " << phi_SE+1 << "° bis " << phi_stop << "°" << endl;
                    calc_wall(phi_SE+1, phi_stop, scan, alpha, d, MSE, phi_0);
                    MSE_limit = sigma*sigma + MSE_fix;
                    //cout << "MSE=" << MSE << " MSE_limit=" << MSE_limit << endl;
                    if( MSE <= MSE_limit  ) {
                        store_wall(phi_SE+1, phi_stop, alpha, d, MSE);
                        //draw_wall(phi_SE+1, phi_stop, scan, 255, 0, 0);
                    }
                }
            } // end else
        }
        if( i_max == 720 )
            i_max = 360+phi_start; // Maximaler Suchwinkel bei Suche nach jeder weiteren Kontur
    } // end while

    if( this->numb_walls > 0 ) {
        /// Search for line with minimum variance of alpha and return its parameters
        var_alpha_min = 1000;
        i_opt = 0;
        for( i=0; i < this->numb_walls; i++) {
            if( this->walls[i].R(2,2) < var_alpha_min ) {
                var_alpha_min = this->walls[i].R(2,2);
                i_opt = i;
            }
        }
        phi = this->walls[i_opt].phi;
        dist = this->walls[i_opt].dist;
        R = this->walls[i_opt].R;
        phi_start = this->walls[i_opt].phi_start;
        phi_stop = this->walls[i_opt].phi_stop;

        if( i_opt != 0 ) // store line with minimum variance as first wall in array
            this->walls[i_opt] = this->walls[0];
        this->walls[0].phi = phi;
        this->walls[0].dist = dist;
        this->walls[0].R = R;
        this->walls[0].phi_start = phi_start;
        this->walls[0].phi_stop = phi_stop;
    }

    return this->numb_walls;
}

/************************************************************************/
bool AMS::AMS_Robot::get_ranger_index(double angle, uint32_t* index)
{
	assert(index);
	double r_ranger_fov = ranger->GetMaxAngle() - ranger->GetMinAngle();
	if (r_ranger_fov <= 0) {
		log.error("Unsupported ranger field of view, "	\
			  "condition (start angle < 0 < stop "	\
			  "angle) does not hold.");
		return false;
	}

	if (ranger->GetMinAngle() > angle || ranger->GetMaxAngle() < angle ) {
		log.errorStream() <<
			"The ranger field of fiew does not include " << rtod(angle) << " deg"			\
			"\nMinAngle = " << rtod(ranger->GetMinAngle()) <<
			"\nMaxAngle = " << rtod(ranger->GetMaxAngle());
		return false;;
	}

	*index = static_cast<uint32_t>(
		rint((angle - ranger->GetMinAngle()) /
		     ranger->GetAngularRes()));
	return true;
}

/************************************************************************/
bool AMS::AMS_Robot::get_min_range(double rStart, double rStop, bool invToMax,
				double* range, uint32_t* index)
{
	RangerProxy& rp(*ranger);
	double minRange = rp.GetMaxRange() +1;
	uint32_t minIdx = 0;
	uint32_t i, stop;

	if (rStart > rStop) {
		minRange = rStart;
		rStart = rStop;
		rStop = minRange;
		minRange = rp.GetMaxRange() + 1;
	}
	get_ranger_index(rStart, &i);
	get_ranger_index(rStop, &stop);
	try {
		// angles rotate in mathmatical positive direction
		for (; i <= stop; ++i) {
			if (rp[i] > rp.GetMinRange() && rp[i] < minRange) {
				minRange = rp[i];
				minIdx = i;
			}
		}
	}
	catch (PlayerError & e) {
		log.errorStream() << e;
		return false;
	}

	log.getStream(AMS::Debug::CALC) <<
		"Ranger min [" << rtod(rStart) << " " << rtod(rStop) <<
		"]deg: range[" << minIdx << "] = " << minRange << "m";
	if (minRange > ranger->GetMaxRange())	{
		if (invToMax) {
			log.debug("No valid measurement, returning max range + 1 (%.2f) "\
				  "on index %ud.", minRange, minIdx);
		} else {
			log.error("No valid range mesurements in range [%f %f]deg.",
				  rtod(rStart), rtod(rStop));
			return false;
		}
	}
	if (range != NULL) {
		*range = minRange;
	}
	if (index != NULL) {
		*index = minIdx;
	}
	return true;
}


/************************************************************************/
void AMS::AMS_Robot::set_vmax(double vmax)
{
    vmax_ = vmax;
}

/************************************************************************/
void AMS::AMS_Robot::set_vacc(double vacc)
{
    vaccel_ = vacc;
}

/************************************************************************/
void AMS::AMS_Robot::set_wmax(double wmax)
{
    wmax_ = dtor(wmax);
}

/************************************************************************/
void AMS::AMS_Robot::set_wacc(double wacc)
{
    waccel_ = dtor(wacc);
}

/************************************************************************/
bool AMS::AMS_Robot::set_speed(double v, double w)
{
    double  vr;         // Geschwindigkeit des rechten Rades
    double  vl;         // Geschwindigkeit des linken Rades
    double sigma_vr;    // Standardabweichung des rechten Rades in m
    double sigma_vl;    // Standardabweichung des linken Rades in m

	log.log(AMS::Debug::TRACE, "Setting Speed to %f m/s and %f rad/s", v, w);
	try {
		if (!set_first_speed_) {
			drive->SetMotorEnable (true);
		}
		set_first_speed_ = true;
        // Begrenzung von v und omega und Fehlerausgabe, falls Geschwindigkeiten Limits überschreiten
        if( fabs(w) > 1.1*wmax_ || fabs(v) > 1.1*vmax_ ){
            if( w > wmax_ ){
                w>wmax_ ? w=wmax_ : w=-wmax_;
                log.errorStream() << "Max. Winkelgeschwindigkeit überschritten --> Begrenzung auf wmax\n";
            }
            if( v > vmax_ ){
                v>vmax_ ? v=vmax_ : v=-vmax_;
                log.errorStream() << "Max. Bahngeschwindigkeit überschritten --> Begrenzung auf vmax\n";
            }
        }
        // Generierung eines gaußverteilten Fehlers für v und omega
        // Radgeschwindigkeiten über inverse Kinematik berechnen
        vr = v + w*wheel_dist;
        vl = v - w*wheel_dist;
        // Standardabweichungen der Radgeschwindigkeiten ermitteln
        sigma_vr = sqrt(fabs(slip_const*vr));
        sigma_vl = sqrt(fabs(slip_const*vl));
        // Tatsächliche simulierte Geschwindigkeiten mit normalverteiltem Fehler verfälschen
        // Durch 12-fache Überlagerung gleichverteilter Zufallsgrößen mit sigma entsteht normalverteilte Zufallsgröße auch mit sigma
        for( int i=0; i<12; i++){
            vr += sigma_vr*((double)rand()/RAND_MAX - 0.5);
            vl += sigma_vl*((double)rand()/RAND_MAX - 0.5);
        }
        // Robotergeschwindigkeiten über Kinematik berechnen
        v = (vr + vl)/2;
        w = (vr - vl)/(wheel_dist*2);
        if( fabs(w) > 1.1*wmax_ )
            w>wmax_ ? w=wmax_ : w=-wmax_;
        if( fabs(v) > 1.1*vmax_ )
            v>vmax_ ? v=vmax_ : v=-vmax_;
        drive->SetSpeed(v,w);   // ermittelte Geschwindigkeiten an position2d-Proxy übergeben
        return true;
	}
	catch (PlayerError& e) {
		log.fatalStream() << "Exception in AMS_Robot.set_speed(" << v << ", " << w << "):\n" << e;
	}
    return false;
}

/************************************************************************/
void AMS::AMS_Robot::set_slip_const(double slip_const)
{
    this->slip_const = slip_const;
}

/************************************************************************/
void AMS::AMS_Robot::set_sigma_ranger(double sigma_ranger)
{
    this->sigma_ranger = sigma_ranger;
}

/************************************************************************/
bool AMS::AMS_Robot::stop()
{
	try {
        drive->SetSpeed(0,0);
        return true;
	}
	catch (PlayerError& e) {
		log.fatalStream() << "Exception in AMS_Robot.stop():\n" << e;
	}
	return false;
}

/************************************************************************/
double AMS::AMS_Robot::get_t() const
{
	double t;
	try {
		t = drive->GetDataTime();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_t():\n" << e;
		return false;
	}
	return t;
}

/************************************************************************/
double AMS::AMS_Robot::get_x()
{
    double x;
	try {
		x = drive->GetXPos();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_x():\n" << e;
		return false;
	}
	return x;
//    update_pos();
//    return x_exp;
}

/************************************************************************/
double AMS::AMS_Robot::get_y()
{
    double y;
	try {
		y = drive->GetYPos();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_y():\n" << e;
		return false;
	}
	return y;
//    update_pos();
//    return y_exp;
}

/************************************************************************/
double AMS::AMS_Robot::get_theta()
{
    double theta;
	try {
		theta = drive->GetYaw();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_theta():\n" << e;
		return false;
	}
	return theta;
//    update_pos();
//    return th_exp;
}

/************************************************************************/
double AMS::AMS_Robot::get_v() const
{
    double v;
	try {
		v = drive->GetXSpeed();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_v():\n" << e;
		return false;
	}
	return v;
}

/************************************************************************/
double AMS::AMS_Robot::get_w() const
{
    double w;
	try {
		w = drive->GetYawSpeed();
	}
	catch (PlayerError& e) {
		log.errorStream() << "Exception in AMS_Robot.get_w():\n" << e;
		return false;
	}
	return w;
}

/************************************************************************/
double AMS::AMS_Robot::get_vmax() const
{
	return vmax_;
}

/************************************************************************/
double AMS::AMS_Robot::get_vacc() const
{
	return vaccel_;
}

/************************************************************************/
double AMS::AMS_Robot::get_wmax() const
{
	return wmax_;
}

/************************************************************************/
double AMS::AMS_Robot::get_wacc() const
{
	return waccel_;
}

/************************************************************************/
int AMS::Debug::to_dbg(const log4cpp::Priority::Value plvl)
{
	return (plvl < log4cpp::Priority::DEBUG) ? -1 :
		(plvl % 100) / 10;
}

/************************************************************************/
int AMS::Debug::from_dbg(const int dbg_lvl)
{
	return (dbg_lvl < 0 || dbg_lvl > to_dbg(MAX)) ?
		log4cpp::Priority::NOTSET :
		log4cpp::Priority::DEBUG + dbg_lvl * 10;
}

/************************************************************************/
void AMS::AMS_Robot::apply_config(
	const boost::program_options::variables_map& cfg)
{
    if (cfg.count("debug") && cfg["debug"].as<int>() >= 0) {
        log.setPriority(AMS::Debug::from_dbg(cfg["debug"].as<int>()));
	}
    if (!cfg["quiet"].as<bool>()) {
		log4cpp::Appender *console = new log4cpp::OstreamAppender("Console",&clog);
		console->setLayout(new log4cpp::SimpleLayout());
		log.setAppender(console); // takes ownership
	}
	else {
//      to shut playerc up
//		freopen ("/dev/null","w",stderr);
	}
	if (cfg.count("logfile")) {
		log4cpp::Appender *logfile = new log4cpp::FileAppender(
				"Logfile",
				cfg["logfile"].as<string>());
		logfile->setLayout(new log4cpp::SimpleLayout());
		log.setAppender(logfile); // takes ownership
	}

	log.log(AMS::Debug::TRACE, "Converting angular values:");
	// convert wmax_ and waccel_ into rad
	wmax_ = dtor(cfg["wmax"].as<double>());
	waccel_ = dtor(cfg["waccel"].as<double>());
}

/************************************************************************/
void AMS::AMS_Robot::delete_clients()
{
	if (NULL != ranger) {
		delete ranger;
		ranger = NULL;
	}
	if (NULL != drive) {
		delete drive;
		drive = NULL;
	}
	if (NULL != client) {
		delete client;
		client = NULL;
	}
}
