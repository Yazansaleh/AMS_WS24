/// Berliner Hochschule für Technik
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer, M.Eng. Thimo Langbehn

/// This header-file defines the aggregation class "AMS_Robot",
/// which can be used to control a simulated or a real robot
/// equipped with a differential drive and a laser sensor.
/// The class implements a position2d- and a ranger-interface within
/// the player-framework, and can be used to change or to read state
/// parameters of the robot.
/// In case of a simulated robot also some methods for plotting
/// within the "stage"-window using graphic2d-proxies are available.
/// Besides, the robot position in "stage" can be set or retrieved.
/// Further, the class provides a number of higher-lever methods for
/// robot navigation and it implements an error model assuming slipping.
/// The class evaluates exceptions thrown within the player proxies
/// and contains a facility for error and event logging.
/// Control parameters can be delivered via command-line or as
/// a config file, and new programm option can be dynamically added.

#ifndef __AMS_ROBOT_HPP__
#define __AMS_ROBOT_HPP__

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <string>
#include <boost/program_options.hpp>                    // setting of parameters
#include <log4cpp/Category.hh>                          // event/error logging
#include <libplayerc++/playerc++.h>                     // Player-Proxies
#include "boost/date_time/posix_time/posix_time.hpp"    // time tracking
#include <newmat/newmatap.h>                            // Linear algebra

using namespace boost::posix_time;
using namespace NEWMAT;

namespace AMS {

/// Constants and conversion for additional debug levels
struct Debug {
	/// Normal debug level (same as log4cpp::Priority::DEBUG)
	static const log4cpp::Priority::Value DEBUG =	log4cpp::Priority::DEBUG + 0;
	/// Enter and/or exit traces for important functions.
	static const log4cpp::Priority::Value TRACE =	log4cpp::Priority::DEBUG + 10;
	/// Additional information regarding command processing.
	static const log4cpp::Priority::Value CMDS =	log4cpp::Priority::DEBUG + 20;
    /// Printing of (interim) calculation results.
	static const log4cpp::Priority::Value CALC =	log4cpp::Priority::DEBUG + 30;
	/// All (even frequent) debugging info there is.
	static const log4cpp::Priority::Value MAX = CALC;
	/// Convert a log4cpp priority value to a debug level.
	static int to_dbg(const log4cpp::Priority::Value plvl);
	/// Convert a debug level to a log4cpp priority value.
	static int from_dbg(const int dbg_lvl);
};

struct wall {
    double phi;
    double dist;
    SymmetricMatrix R;
    int phi_start;
    int phi_stop;
};
#define MAX_WALLS 4 // maximum number of walls to be detected within a scan

/// Robot
class AMS_Robot {
public:
	/// The logging facility, can be used by clients, too.
	log4cpp::Category& log;
	/// Custom command-line / configuration parameters.
	boost::program_options::options_description options;
	/// Values for all command-line / configuration parameters
	boost::program_options::variables_map config;
	/// Player client instance (connection).
	PlayerCc::PlayerClient* client;
	/// Position2D proxy for speed and odometry.
	PlayerCc::Position2dProxy* drive;
	/// Simulation proxy for retrieving and setting robot position.
	PlayerCc::SimulationProxy* stage;
	/// Ranger proxy to access laser ranger data.
	PlayerCc::RangerProxy* ranger;
	/// Graphics2d proxy to draw related to robot position.
	PlayerCc::Graphics2dProxy* graphics;
	/// Graphics2d proxy to draw related to map position.
	PlayerCc::Graphics2dProxy* graphmap;

	/// Constructor initializing the logging facility.
	/// @param name [IN] specifies the name of the log category.
	AMS_Robot(const std::string name="AMS_Robot");

	/// Destructor cleaning up logging and open player ressources.
	~AMS_Robot();

	/// Parsing the provided parameters and filling AMS_Robot::config.
	/// This evaluates the command line arguments with respect to the
	/// predefined and custom options available. If a config-file is
	/// given, it is evaluated, too. In that case the command line
	/// take precedence.
	/// Afterwards, the parameter are available as entered in
	/// AMS_Robot::config, therefore they may still need to be converted.
	/// @param argc [IN] specifies the number of arguments (size of argv).
	/// @param argv [IN] contains the command line parameters as array of c-strings.
	/// @returns true if the configuration parameters could be read and
	///     converted, false otherwise.
	bool read_config(int argc, char** argv);

	/// Connecting to the player server on the robot and its devices.
	/// @returns true if all proxies could be connected, false otherwise.
	bool connect();

	/// Retrieves position of the robot from the simulator
	/// @param x [OUT] specifies the x-coordinate of the robot [m].
	/// @param y [OUT] specifies the y-coordinate of the robot [m].
	/// @param theta [OUT] specifies the angular coordinate of the robot [rad].
	/// @returns true if position could be retrieved, false otherwise.
	bool get_sim_pos(double &x, double &y, double &theta);

	/// Positions the robot in the simulator
	/// @param x [IN] specifies the x-coordinate of the robot [m].
	/// @param y [IN] specifies the y-coordinate of the robot [m].
	/// @param theta [IN] specifies the angular coordinate of the robot [rad].
	/// @returns true if position could be retrieved, false otherwise.
	bool set_sim_pos(double x, double y, double theta);

	/// Waits until new data is received from player server.
    void wait_for_new_data();

	/// Sets the PUSH mode in the client for requesting data and deactivates the
	/// replace rule for any data. Besides, the data message queue is emptied.
	/// Attention: If data is not read regularly in short intervals, the queue
	/// will not be empty and thus the received data may become out of date
	/// @returns true if during execution no exception occurs, false otherwise
	bool init_push_mode();

	/// Sets the PULL mode and activates an replace rule for any data message,
	/// in order to avoid flooding of the data queue at the server.
	/// Besides, the data message queue is emptied
	/// Attention: By using PULL mode some longer delay may occur from requesting
	/// new data until actually data is obtained.
	/// @returns true if during execution no exception occurs, false otherwise
	bool init_pull_mode();

	/// Provides the maximal translational speed of the robot.
	/// @returns the maximal speed in [m/s].
	double get_vmax() const;

	/// Provides the maximal translational acceleration of the robot.
	/// This Value is also used for maximum deceleration
	/// @returns the maximal acceleration in [m/s^2].
	double get_vacc() const;

	/// Provides the maximal angular velocity of the robot.
	/// Assuming a symmetric construction, this is equal for turning
	/// left and right (positive and negative).
	/// @returns the maximal angular velocity in [rad/s].
	double get_wmax() const;

	/// Provides the maximal angular acceleration of the robot.
	/// Assuming a symmetric construction, this should be the same
	/// as for deceleration (acceleration in reverse direction).
	/// Acceleration is given positive (turn left), but is used for negative
	/// as well (turn right).
	/// @returns the maximal acceleration/deceleration in [rad/s^2].
	double get_wacc() const;

	/// Provides the current time of the robot.
	/// The resolution depends on server update rate (default: 0.1s)
	/// @returns the time t in [s].
	double get_t() const;

	/// Provides the current x-position of the robot.
	/// @returns the x-coordinate in [m].
	double get_x();

	/// Provides the current y-position of the robot.
	/// @returns the y-coordinate in [m].
	double get_y();

	/// Provides the current orientation of the robot.
	/// @returns the angle theta in [rad].
	double get_theta();

	/// Provides the current speed of the robot in x-direction.
	/// @returns the speed in [m/s].
	double get_vx() const;

	/// Provides the current speed of the robot in y-direction.
	/// @returns the speed in [m/s].
	double get_vy() const;

	/// Provides the current speed of the robot in forward direction.
	/// @returns the speed in [m/s].
	double get_v() const;

	/// Provides the current angular speed of the robot.
	/// @returns the speed in [m/s].
	double get_w() const;

    /// Sets a maximum value for the translational speed
    /// @param vmax in [m/s]
    void set_vmax(double vmax);

    /// Sets a maximum value for the translational acceleration
    /// @param vmax in [m/s²]
    void set_vacc(double vacc);

    /// Sets a maximum value for the rotational speed
    /// @param wmax in [deg/s]
    void set_wmax(double wmax);

    /// Sets a maximum value for the rotational acceleration
    /// @param vmax in [deg/s²]
    void set_wacc(double wacc);

	/// Set a new value for the slippling constant
	/// @param slip_const [IN] specifies the slipping const used in the error model.
	void set_slip_const(double slip_const);

	/// Set a new value for the standard deviation of the ranger sensor
	/// @param sigma_ranger [IN] specifies the standard deviation in cm.
	void set_sigma_ranger(double sigma_ranger);

 	/// Limits the speeds to vmax_ and wmax_ and sets the robot v-omega speed
	/// by means of applying an error model. For this purpose, the delivered velocities
	/// v and w are considered as exspected values of normal distributed random variables
	/// dependent on the parameter slip_const stored as attribute.
	/// @param v [IN] specifies the required translational velocity in [m/s].
	/// @param w [IN] specifies the required angular velocity in [rad/s].
	/// @returns true if the given velocities could be set correctly, false otherwise.
	bool set_speed(double v, double w);

	/// Stops the robot immediately
	bool stop();

    /// Drives the robot a certain straight distance forward or backwards
    /// while applying a constant acceleration and a maximum velocity.
    /// @param dist [IN] defines the distance in [m] to be traversed
    /// @returns true if the delivered distance has been travelled successfully.
    bool move_dist(double dist);

    /// Rotates the robot a certain angle to the left or to the right
    /// while applying a constant angular acceleration and a maximum angular velocity.
    /// @param angle [IN] defines the angle in [rad] to be rotated
    /// @returns true if the delivered angle has been turned successfully.
    bool turn_angle(double angle);

    /// Drives to a number of points by means of turning in the
    /// direction of the next point and then travelling the distance
    /// towards that point, respectively.
    /// If 'count' is set to '1', the robot will turn in the direction of
    /// the second point after arriving at the first point.
    /// @param P [IN] pointer to a field of waypoints
    /// @param count [IN] holds the number of waypoints to be travelled
	bool drive_polyline(player_point_2d_t *P, int count);

	/// Calculates a trajectory by means of connecting the passed
	/// points creating "round" corners with smooth bending
	/// either with cosinus functions or with clothoides.
    /// @param P [IN] pointer to a field of waypoints
    /// @param N [IN] holds the number of points
    /// @param dt_soll [IN] defines the time difference between adjacent points
    ///     assuming maximum speed of the robot
    /// @param opt [IN] defines, if "cos" or "clothoide" is used as curvature
    /// @param file [IN] holds the file name to store the trajectory
    /// @returns true if the trajectory has been calculated successfully
	bool calc_traject(player_point_2d_t *P, int N, double dt_soll, std::string opt, std::string file);

    /// Draws a point at coordinates (x,y) with color defined by components r, g and b
    /// either related to the position of the robot or related to the map
    /// @param x [IN] x-coordinate
    /// @param y [IN] y-coordinate
    /// @param r [IN] red color component
    /// @param g [IN] green color component
    /// @param b [IN] blue color component
	/// @param mode specifies if the circle segment is moved with the robot (mode='r')
    ///     or if it is fixed in the map at the position it has been plotted (mode='m')
    bool draw_point(double x, double y, int r, int g, int b, char mode);

	/// Opens a file with a number of close points defining a trajectory
	/// and plots these data using color defined by components r, g and b
    /// @param file [IN] holds the file name with the trajectory
    /// @param r [IN] red color component
    /// @param g [IN] green color component
    /// @param b [IN] blue color component
    bool draw_traject(std::string file, int r, int g, int b);

    /// Draws a circle segment in the stage window either related
    /// to the position of the robot or related to the map
	/// Draws the radius (a straight line) from the center in the
	/// direction of the start angle with the given length,
	/// an arc in math. positive direction to the stop angle and
	/// then a line (the radius) back to the center.
	/// The center is specified by its offset to the robot center.
	/// Note that the graphic will automatically be adjusted if
	/// the robot is moved. To remove the graphic (or change it),
	/// use AMS::AMS_Robot::clear_graphics() and redraw.
	/// @param astart specifies the start angle relative to the
	///     robot direction in [rad].
	/// @param astop specifies the stop angle relative to the robot
	///     direction in [rad].
	/// @param radius specifies the length of the radius in [m].
	/// @param dx in case of mode=='r' specifies the x-axis offset
	///     of the segment center from the robot center in [m].
	///     In case mode=='m' it specifies the x-coordinate of the plot
	/// @param dy in case of mode=='r' specifies the y-axis offset
	///     of the segment center from the robot center in [m].
	///     In case mode=='m' it specifies the x-coordinate of the plot
	/// @param r specifies the red portion of the draw color.
	/// @param g specifies the green portion of the draw color.
	/// @param b specifies the blue portion of the draw color.
	/// @param mode specifies if the circle segment is moved with the robot (mode='r')
    ///     or if it is fixed in the map at the position it has been plotted (mode='m')
	/// @returns true if the command was successfull or if no stage
	///     exists (in the case of a real robot).
	bool draw_circle_seg(double astart, double astop, double radius,
        double dx, double dy, uint8_t r, uint8_t g, uint8_t b, char mode);

 	/// Deletes all custom graphics on stage window related to the robot.
	/// @returns true if the command was successfull or if no stage
	///     exists (in the case of a real robot).
	bool clear_graphics();

	/// Deletes all custom graphics on stage window related to the map.
	/// @returns true if the command was successfull or if no stage
	///     exists (in the case of a real robot).
	bool clear_graphmap();

    /// Provides the current scan of the range sensor. The index in the
    /// scan field refers to the angle in the range from 0 to 359 deg.
    /// If in a certain direction no measurement is taken, the value in scan is set to 0.
    /// If in a certain direction no obstacle was detected, the value in scan is set to -1
    /// A normal distributed error with standard deviation sigma_ranger is added
    /// to all valid range measurements.
    /// @param scan is a reference to a pointer to the field of range values.
    /// @param MaxRange returns the maximal distance of the measurement in [m].
    /// @param sigma returns the standard deviation of the range sensor in [m]
    /// @returns true if the reading was successful, false otherwise.
    bool get_scan(double*& scan, double& MaxRange, double& sigma);

    /// Evaluates the current scan and tries to detect linear contours within it
    /// For this purpose, first a connected contour of an obstacle is found, which
    /// needs to consist at least of N_min measurement points, then the parameters
    /// of the fitted straight line (alpha and dist) are determined. Out of
    /// two alternatives the valid straight line is found by means of the MSE.
    /// If necessary, a second contour is searched for. The returned parameters alpha,
    /// d, and R correspond to that line with minimum variance of alpha.
    /// If the MSE is to high, the scan is divided into to parts and again fitting
    /// of a straight line is performed. Finally, by means of a simplified error
    /// model the covariance matrix is determined.
    /// @param alpha_back comprises the angle of a found straight line in [rad]
    /// @param d_back comprises the distance from the origin of a found straight line in [m]
    /// @param R contains the covariance matrix of alpha and d
    /// @param N_min describes the minimum number of required contour points of a line
    /// @returns the number of detected straight lines
    int detect_wall(double& phi, double& dist, SymmetricMatrix& R, int N_min=10);

    void draw_wall(int phi_start, int phi_stop, double* scan, int r, int g, int b);

    /// returns wall parameter found by detect_wall() corresponding to index
	/// @param index [IN] points to a wall within the array "walls".
	/// @param phiR [IN] is the normal angle of the line in [rad].
	/// @param dR [IN] is the normal dist of the line in [rad].
	/// @param R [IN] holds the covariance matrix of the line parameters.
    /// @returns true if index is in the valid range 1-4
    bool get_wall(int index, double& phiR, double& dR, SymmetricMatrix& R);

	/// Searches the minimum ranger distance in a given angular range.
	/// The given angular range [rStart, rStop] is searched for the minimum
	/// distance value, ignoring invalid values (< minRange).
	/// On success, the range and corresponding index are returned.
	/// In the special case of no valid measurement in the given range,
	/// either the maxRange+1 / Index 0 pair can be returned (success) or
	/// the function aborts with an error (failes). This behavior can
	/// be selected with the invToMax flag.
	/// @param rStart [IN] specifies the start angle [rad] (inclusive).
	/// @param rStop [IN] specifies the stop angle [rad] (inclusive).
	/// @param invToMax [IN] set to true returns MaxRange+1 and index 0
	///     in case of all invalid ranges, if false such a situation leads
	///     to abortion and return value of false.
	/// @param range [OUT] is set to the minimum distance value [m] if one
	///     exists or to maxRange+1 in the special case). If not needed, set to NULL.
	/// @param index [OUT] is set to the corresponding index or 0 for the
	///     special case. If not needed, set to NULL.
	/// @returns true if a valid mimimum was found (or invToMax is true),
	///     alse otherwise.
	bool get_min_range(double rStart, double rStop, bool inv_to_max,
			   double* range, uint32_t* index);

private:

    /// Used by drive_polyline() to calculate the minimum rotation angle
    /// including sign from the current position towards the point
    /// specified by the offset values dx and dy.
    /// Attention: wait_for_new_data() must be called before this method.
    /// @param dx specifies the x-offset from current position in [m]
    /// @param dy specifies the y-offset from current position in [m]
    double calc_angle(double dx, double dy);

 	/// Used by get_min_range() to convert a given angle to the best-match
 	/// index for the ranger. If the angle is not contained in the region
 	/// [MinAngle, MaxAngle] of the ranger, the mapping fails.
	/// @param angle [IN] specifies the angle in [rad] to be mapped.
	/// @param index [OUT] is set to the corresponding range index if the
	///     angle could be translated, otherwise it remains unchanged.
	/// @returns true if the angle could be mapped, false otherwise.
	bool get_ranger_index(double angle, uint32_t* index);

    /// Used by "detect_wall()" to store line parameters measured from the
    /// current robot position in the array "walls".
	/// @param phi_start [IN] is the first scanned angle in [deg] to be mapped.
	/// @param phi_stop [IN] is the last scanned angle in [deg] to be mapped.
	/// @param phiR [IN] is the normal angle of the line in [rad].
	/// @param dR [IN] is the normal dist of the line in [rad].
	/// @param MSE [IN] is the mean squared error of the line points.
    void store_wall(int phi_start, int phi_stop, double phiR, double dR, double MSE);

    /// Used by "detect_wall()" to calculate line parameters of a scanned contour
    /// @param phi_1 [IN] is the first scanned angle in [deg] to be mapped.
	/// @param phi_2 [IN] is the last scanned angle in [deg] to be mapped.
	/// @param scan [IN] points to the scan.
	/// @param phiR [IN] is the normal angle of the line in [rad].
	/// @param dR [IN] is the normal dist of the line in [rad].
	/// @param MSE [IN] is the mean squared error of the line points.
	/// @param Phi_SE [IN] holds the angle to the point with the largest squared error.
    void calc_wall(int phi_1, int phi_2, double* scan, double& phiR, double& dR, double& MSE, int& phi_SE);

 	/// Use the parsed config values, convert them and set attributes.
	/// @param cfg [IN] contains the map with the parameters to use.
	void apply_config(const boost::program_options::variables_map& cfg);

	/// Clean up player proxies and the client.
	void delete_clients();

    /// Attributes
	bool got_first_valid_data_; // Indicates if data for all proxies is available.
    bool set_first_speed_;      // First speed command triggers Position2dProxy::SetMotorEnable().
	double last_update_time_;   // Timestamp of the last speed command send by the ramp controller.
	double vmax_;               // Maximal translational speed [m/s] of the robot.
	double vaccel_;             // Maximal translational acceleration [m/s²] of the robot.
	double wmax_;               // Maximal angular velocity [rad/s] of the robot.
	double waccel_;             // Maximal angular accelleration/deceleration [rad/s²] of the robot.
	double range_scan_[360];    // field which contains the latest ranger readings after call of get_scan()
	ptime t_last;                // Time stemp of the last position update of the robot (used for measuring dt)
	double sigma_ranger;        // Simulated standard deviation of the ranger in [m]
	double wheel_dist;          // distance between the robot wheels
	double slip_const;          // slipping constant
	int numb_walls;              // number of stored walls
    wall walls[MAX_WALLS];       // array, which stores detected straight lines
};

} // end of namespace definition

/// \namespace PlayerCc contains the Proxies to communicate with player servers.

#endif //__AMS_ROBOT_HPP__
