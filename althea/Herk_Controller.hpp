#pragma once

#include "Controller_base.hpp"
#include "Tools.hpp"
#include "Herk.hpp"

//! Motor ID for Althea II Linear Translation
static const uint8_t MOTOR_TRANSLATION = 1;
//! Offset in degrees for Althea II Linear Translation
static const uint8_t MOTOR_TRANSLATION_BIAS = 180;
//! Conversion ratio from degree to milimeter for Althea II Linear Translation
static const double TRANSLATION_MM_DEG = 0.3658;

//! Motor ID for Althea II Twist
static const uint8_t MOTOR_TWIST = 2;
//! Offset in degrees for Althea II Twist to keep catheter upright and horizontal
static const uint8_t MOTOR_TWIST_BIAS = 109;// +109;// +107;

//! Motor ID for Althea II Bend
static const uint8_t MOTOR_BEND = 219;
//! Offset in degrees for Althea II Bending to keep catheter tip zero degrees
//static const int MOTOR_BEND_BIAS = 80;//+45;
static const int MOTOR_BEND_BIAS = 80;// +47;
//! Conversion from input degree of the motor to output degree of catheter tip for Althea II Bending
static const double MOTOR_BEND_M_B = 1;//1.1/(4*0.145);

/*!
 * \enum    Herk_mode
 *
 * \brief   The current configuration the Herkulex motors are in.
 */
enum Herk_mode
{
    //! Running
    ON,
    //! Disabled or Off
    OFF,
    //! Have reached maximum position
    MAX,
    //! Have reached minimum position
    MIN
};

enum direction
{
    //! Running
    STOP,
    //! Disabled or Off
    FORWARD,
    //! Have reached maximum position
    BACKWARD
};

enum timing
{
    //! Running
    CONTINUE,
    //! Disabled or Off
    WAIT
};

/*!
* \enum    Herk_control
*
* \brief   The means of controlling the Herkulex motors.
*/
enum Herk_control
{
    //! Uses position only, by sending a position command to the motors
    POSITION,
    //! Uses velocity only, causeing the motors to spin indefintly
    VELOCITY,
    //! Uses user-set velocity and position to move the motor
    VEL_POS
};

/*!
 * \class   Herk_Controller
 *
 * \brief   A controller for handling communication with the the Herkulex DRS 0602 Smart motors.
 *
 * \author  Ibrahim Abdulhafiz
 * \date    7/5/2019
 */
class Herk_Controller: public Tools
{
public:
	friend class Althea_Algo;
	/*!
	 * \fn  Herk_Controller::Herk_Controller();
	 *
	 * \brief   Default constructor. Initializes all variables, opens the serial port, and starts the main communication thread.
	 *
	 * \author  Ibrahim Abdulhafiz
	 * \date    7/5/2019
	 */
	Herk_Controller();

    /*!
     * \fn  Herk_Controller::Herk_Controller();
     *
     * \brief   Default constructor. Initializes all variables, opens the serial port, and starts the main communication thread. 
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     */
    Herk_Controller(M_Frame* parent);

    /*!
     * \fn  Herk_Controller::~Herk_Controller();
     *
     * \brief   Destructor
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     */
    ~Herk_Controller();

	bool SetUp(std::string com);
	bool Close();

    /*!
     * \fn  bool Herk_Controller::Is_initialized();
     *
     * \brief   Query if this object is initialized
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     *
     * \returns True if initialized, false if not.
     */
    bool Is_initialized();

    /*!
     * \fn  bool Herk_Controller::Set_translation_mode(Herk_mode mode);
     *
     * \brief   Sets translation mode
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     *
     * \param   mode    The mode (or state) of the translation motor.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_translation_mode(Herk_mode mode);

    /*!
     * \fn  bool Herk_Controller::Set_translation_limit(double max_position_ms, double min_position_ms);
     *
     * \brief   Sets the translation limit in millimeters for the translation motor.
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     *
     * \param   max_position_ms The maximum position in millimeters.
     * \param   min_position_ms The minimum position in millimeters.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_translation_limit(double max_position_ms, double min_position_ms);

    /*!
     * \fn  bool Herk_Controller::Set_translation_velocity(double velocity_mm_s);
     *
     * \brief   Sets the translation velocity in millimeters per second.
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     *
     * \param   velocity_mm_s   The velocity in millimetres per second.
     * \param   ctrl   (Optional) The control mode of the motor (refer to Herk_control).
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_translation_velocity(double velocity_mm_s, Herk_control ctrl = VEL_POS);

    /*!
     * \fn  bool Herk_Controller::Set_translation_position(double position_mm, double time_ms = 1200);
     *
     * \brief   Sets the translation position of the translation motor.
     *
     * \author  Ibrahim Abdulhafiz
     * \date    7/5/2019
     *
     * \param   position_mm The position of the motors in millimetres.
     * \param   time_ms     (Optional) The time to complete the opperation.
     * \param   ctrl     (Optional) The control mode of the motor (refer to Herk_control).
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_translation_position(double position_mm, Herk_control ctrl = VEL_POS, double time_ms = 1200, timing t = CONTINUE);


    /*!
     * \fn  bool Herk_Controller::Get_translation_velocity();
     *
     * \brief   Gets translation velocity in millimeters per second.
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_translation_velocity(double &vel);

    /*!
     * \fn  bool Herk_Controller::Get_translation_position(double &_position_mm);
     *
     * \brief   Gets translation position in millimeters.
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \param [in,out]  _position_mm    The current position in millimetres.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_translation_position(double &_position_mm);

    /*!
     * \fn  bool Herk_Controller::Get_translation_limits();
     *
     * \brief   Gets translation limits
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_translation_limits();

    /*!
     * \fn  bool Herk_Controller::Set_twist_mode(Herk_mode mode);
     *
     * \brief   Sets twist mode
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \param   mode    The mode.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_twist_mode(Herk_mode mode);

    /*!
     * \fn  bool Herk_Controller::Set_twist_limit(double max_position_deg, double min_position_deg);
     *
     * \brief   Sets twist limit
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \param   max_position_deg    The maximum position degrees.
     * \param   min_position_deg    The minimum position degrees.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_twist_limit(double max_position_deg, double min_position_deg);

    /*!
     * \fn  bool Herk_Controller::Set_twist_velocity();
     *
     * \brief   Sets twist velocity
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_twist_velocity(double position_deg_s, Herk_control ctrl = VEL_POS);

    /*!
     * \fn  bool Herk_Controller::Set_twist_position();
     *
     * \brief   Sets twist position
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_twist_position(double position_deg, Herk_control ctrl = VEL_POS, double time_ms = 1200, timing t = CONTINUE);

    /*!
     * \fn  bool Herk_Controller::Set_twist_limits();
     *
     * \brief   Sets twist limits
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_twist_limits();

    /*!
     * \fn  bool Herk_Controller::Get_twist_velocity();
     *
     * \brief   Gets twist velocity
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_twist_velocity(double &vel);

    /*!
     * \fn  bool Herk_Controller::Get_twist_position();
     *
     * \brief   Gets twist position
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_twist_position(double &_position_deg);

    /*!
     * \fn  bool Herk_Controller::Get_twist_limits();
     *
     * \brief   Gets twist limits
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_twist_limits();

    /*!
     * \fn  bool Herk_Controller::Set_bend_mode(Herk_mode mode);
     *
     * \brief   Sets bend mode
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \param   mode    The mode.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_bend_mode(Herk_mode mode);

    /*!
     * \fn  bool Herk_Controller::Set_bend_limit(double max_position_deg, double min_position_deg);
     *
     * \brief   Sets bend limit
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \param   max_position_deg    The maximum position degrees.
     * \param   min_position_deg    The minimum position degrees.
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_bend_limit(double max_position_deg, double min_position_deg);

    /*!
     * \fn  bool Herk_Controller::Set_bend_velocity();
     *
     * \brief   Sets bend velocity
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_bend_velocity(double position_deg_s, Herk_control ctrl = VEL_POS);

    /*!
     * \fn  bool Herk_Controller::Set_bend_position();
     *
     * \brief   Sets bend position
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_bend_position(double position_deg, Herk_control ctrl = VEL_POS, double time_ms = 1200, timing t = CONTINUE);

    /*!
     * \fn  bool Herk_Controller::Set_bend_limits();
     *
     * \brief   Sets bend limits
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Set_bend_limits();

    /*!
     * \fn  bool Herk_Controller::Get_bend_velocity();
     *
     * \brief   Gets bend velocity
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_bend_velocity(double &vel);

    /*!
     * \fn  bool Herk_Controller::Get_bend_position();
     *
     * \brief   Gets bend position
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_bend_position(double &_position_deg);

    /*!
     * \fn  bool Herk_Controller::Get_bend_limits();
     *
     * \brief   Gets bend limits
     *
     * \author  Mieuser
     * \date    7/5/2019
     *
     * \returns True if it succeeds, false if it fails.
     */
    bool Get_bend_limits();

	void Home(bool hard_command = false, double pos_trans = 0, double pos_twist = 0, double pos_bend = 0);

	void e_stop(bool _return = false);

    boost::shared_ptr<Herk> _motors;
    
    // Run the twist and translation motor back and forth 5 times before stopping. 
    bool RunDemo1(void)
    {
        //if (!Is_initialized())
        //{
        //    return false;
        //}

        //Set_translation_mode(Herk_mode::ON);
        //Set_twist_mode(Herk_mode::ON);
        //Set_bend_mode(Herk_mode::ON);

        //for (int i = 0; i < 5; i++)
        //{
        //    Set_translation_position(_min_translation_position_mm, 1000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(1000);
        //    Set_translation_position(_max_translation_position_mm, 2000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(2000);
        //}

        //Set_translation_mode(Herk_mode::OFF);
        //Set_twist_mode(Herk_mode::OFF);
        //Set_bend_mode(Herk_mode::OFF);
    }

    bool RunDemo2(void)
    {
        //if (!Is_initialized())
        //{
        //    return false;
        //}

        //Set_translation_mode(Herk_mode::ON);
        //Set_twist_mode(Herk_mode::ON);
        //Set_bend_mode(Herk_mode::ON);

        //for (int i = 0; i < 5; i++)
        //{
        //    Set_translation_position(_min_translation_position_mm, 1000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(1000);
        //    Set_translation_position(_max_translation_position_mm, 2000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(2000);
        //}

        //Set_translation_mode(Herk_mode::OFF);
        //Set_twist_mode(Herk_mode::OFF);
        //Set_bend_mode(Herk_mode::OFF);

    }

    bool RunDemo3(void)
    {
        //if (!Is_initialized())
        //{
        //    return false;
        //}

        //Set_translation_mode(Herk_mode::ON);
        //Set_twist_mode(Herk_mode::ON);
        //Set_bend_mode(Herk_mode::ON);

        //for (int i = 0; i < 5; i++)
        //{
        //    Set_translation_position(_min_translation_position_mm, 1000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(1000);
        //    Set_translation_position(_max_translation_position_mm, 2000);
        //    //Set_twist_position(); // Should be the same as above
        //    Sleep(2000);
        //}

        //Set_translation_mode(Herk_mode::OFF);
        //Set_twist_mode(Herk_mode::OFF);
        //Set_bend_mode(Herk_mode::OFF);
    }

	boost::atomic<bool> _running;

private:

    void recieve_info_thread();

	wxWindow* _parent_form;


	void wait_for_value(double desired, bool (*function)(double&), double accuracy = 1, int timeout_ms = 30);

    bool _is_initialized;

    boost::shared_ptr<boost::thread> _info_thread;

    boost::mutex _variables;

    boost::mutex _current_translation_position_mm_mutex;

    boost::mutex _current_twist_position_deg_mutex;

    boost::mutex _current_bend_position_deg_mutex;

public:

    boost::atomic<Herk_mode> _translation_mode;

    boost::atomic<Herk_control> _translation_control;

    boost::atomic<direction> _translation_direction;

    boost::atomic<double> _translation_position;

    boost::atomic<double> _translation_velocity;

    boost::atomic<double> _max_translation_vel_mm_s;

    boost::atomic<double> _new_translation_velocity_mm_s;

    boost::atomic<double> _translation_velocity_time_ms;

    boost::atomic<double> _new_translation_position_mm;

    boost::atomic<double> _current_translation_velocity_mm_s;

    boost::atomic<double> _current_translation_position_mm;

    boost::atomic<double> _max_translation_position_mm;

    boost::atomic<double> _min_translation_position_mm;

    boost::atomic<double> _translation_sensitivity;


    boost::atomic<Herk_mode> _twist_mode;

    boost::atomic<Herk_control> _twist_control;

    boost::atomic<direction> _twist_direction;

    boost::atomic<double> _twist_position;

    boost::atomic<double> _twist_velocity;

    boost::atomic<double> _max_twist_vel_deg_s;

    boost::atomic<double> _new_twist_velocity_deg_s;

    boost::atomic<double> _twist_velocity_time_ms;

    boost::atomic<double> _new_twist_position_deg;

    boost::atomic<double> _current_twist_velocity_deg_s;

    boost::atomic<double> _current_twist_position_deg;

    boost::atomic<double> _max_twist_angle_deg;

    boost::atomic<double> _min_twist_angle_deg;

    boost::atomic<double> _twist_sensitivity;


    boost::atomic<Herk_mode> _bend_mode;

    boost::atomic<Herk_control> _bend_control;
    
    boost::atomic<direction> _bend_direction;
    
    boost::atomic<double> _bend_position;

    boost::atomic<double> _bend_velocity;

    boost::atomic<double> _max_bend_vel_deg_s;

    boost::atomic<double> _new_bend_velocity_deg_s;

    boost::atomic<double> _bend_velocity_time_ms;

    boost::atomic<double> _new_bend_position_deg;

    boost::atomic<double> _current_bend_velocity_deg_s;

    boost::atomic<double> _current_bend_position_deg;

    boost::atomic<double> _max_bend_angle_deg;

    boost::atomic<double> _min_bend_angle_deg;

    boost::atomic<double> _bend_sensitivity;
};
