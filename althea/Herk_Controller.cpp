#include "Herk_Controller.hpp"
#include "Herk.hpp"
#include "Buffered_async_serial.hpp"
#include "Math.hpp"

Herk_Controller::Herk_Controller(M_Frame* parent) : Tools(parent),
	_parent_form((wxWindow*)parent),
    _is_initialized(false),
	_running(false),
    _min_translation_position_mm(-200),
    _max_translation_position_mm(400),
    _max_translation_vel_mm_s(300),
    _translation_velocity_time_ms(50),
    _translation_sensitivity(4),
	_new_translation_position_mm(MOTOR_TRANSLATION_BIAS),
	_new_translation_velocity_mm_s(0),
	_current_translation_position_mm(0),
	_current_translation_velocity_mm_s(0),

    _min_twist_angle_deg(-181),
    _max_twist_angle_deg(361),
    _max_twist_vel_deg_s(100),
    _twist_velocity_time_ms(50),
    _twist_sensitivity(5),
	_new_twist_position_deg(MOTOR_TWIST_BIAS),
	_new_twist_velocity_deg_s(0),
	_current_twist_position_deg(0),
	_current_twist_velocity_deg_s(0),

    _min_bend_angle_deg(-56),
    _max_bend_angle_deg(56),
    _max_bend_vel_deg_s(200),
    _bend_velocity_time_ms(1000),
    _bend_sensitivity(1),
	_new_bend_position_deg(MOTOR_BEND_BIAS),
	_new_bend_velocity_deg_s(0),
	_current_bend_position_deg(0),
	_current_bend_velocity_deg_s(0),
	_bend_velocity(0),

    _bend_control(VEL_POS),
    _translation_control(VEL_POS),
    _twist_control(VEL_POS)
{
    
}

Herk_Controller::Herk_Controller() : Tools(),
_parent_form(0),
_is_initialized(false),
_running(false),
_min_translation_position_mm(-200),
_max_translation_position_mm(400),
_max_translation_vel_mm_s(300),
_translation_velocity_time_ms(50),
_translation_sensitivity(1),

_min_twist_angle_deg(-181),
_max_twist_angle_deg(361),
_max_twist_vel_deg_s(100),
_twist_velocity_time_ms(50),
_twist_sensitivity(1),

_min_bend_angle_deg(-45),
_max_bend_angle_deg(45),
_max_bend_vel_deg_s(200),
_bend_velocity_time_ms(1000),
_bend_sensitivity(1),

_bend_control(VEL_POS),
_translation_control(VEL_POS),
_twist_control(VEL_POS)
{

}

Herk_Controller::~Herk_Controller()
{
	Close();
}

bool Herk_Controller::SetUp(std::string com)
{
	Tools::add(com);
	Tools::post();
	
	if (!_is_initialized && Tools::Test_com(com))
	{
		//_running = true;
		std::cout << "returning false;";
		_motors.reset(new Herk(com, 200000));
		int error, detail;
		_motors->Status(MOTOR_TRANSLATION, error, detail);
		if (true || error == 0) {
			Home(true);
			_info_thread.reset(new boost::thread(&Herk_Controller::recieve_info_thread, this));
			_is_initialized = true;
		}
	}
	return _is_initialized;
}

bool Herk_Controller::Close()
{
	Tools::add("bad!");
	Tools::post();
	if (_is_initialized)
	{
		_is_initialized = false;
		if (_info_thread)
		{
			_info_thread->interrupt();
			//_info_thread->join();
		}
		
		Tools::add( "thread interrupted");
		Tools::post();
		_motors->Independent_Move(MOTOR_TRANSLATION_BIAS, 2000, MOTOR_TRANSLATION);
		_motors->Independent_Move(MOTOR_TWIST_BIAS, 2000, MOTOR_TWIST);
		_motors->Independent_Move(MOTOR_BEND_BIAS, 2000, MOTOR_BEND);
		
		//_motors->~Herk(); <-- DONT DO THIS
		_motors.reset();
		return true;
	}
	return true;
}

void Herk_Controller::Home(bool hard_command, double pos_trans, double pos_twist, double pos_bend) {
	if (_is_initialized)
	{
		Set_translation_position(0, POSITION, 2000);
		Set_twist_position(0, POSITION, 2000);
		Set_bend_position(0, POSITION, 2000);
		if (hard_command) {
			std::cout << "moving motors" << std::endl;
			Set_translation_mode(ON);
			_motors->Independent_Move(pos_trans + MOTOR_TRANSLATION_BIAS, 2000, MOTOR_TRANSLATION);
			Set_twist_mode(ON);
			_motors->Independent_Move(pos_twist + MOTOR_TWIST_BIAS, 2000, MOTOR_TWIST);
			Set_bend_mode(ON);
			_motors->Independent_Move(pos_bend + MOTOR_BEND_BIAS, 2000, MOTOR_BEND);
		}
	}
}

void Herk_Controller::e_stop(bool _return) {
	if (_is_initialized)
	{
		if (_return)
			Home(true);
		_is_initialized = false;
		if (_info_thread)
		{
			_info_thread->interrupt();
		}
		_motors->Set_torque(TORQUE_FREE, MOTOR_TRANSLATION);
		_motors->Set_torque(TORQUE_FREE, MOTOR_TWIST);
		_motors->Set_torque(TORQUE_FREE, MOTOR_BEND);
		_motors.reset();
	}
}

bool Herk_Controller::Is_initialized(){
    return _is_initialized;
}


void Herk_Controller::wait_for_value(double desired, bool (*function)(double&), double accuracy, int timeout_ms)
{
	double temp;
	do {
		function(temp);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(timeout_ms));
	} while (std::abs(temp - desired) < accuracy);
	

}
// <

bool Herk_Controller::Set_translation_mode(Herk_mode mode)
{
	if (_is_initialized)
	{
		if (mode == ON)
		{
			_motors->Set_torque(TORQUE_ON, MOTOR_TRANSLATION);
		}
		else if (mode == OFF)
		{
			_motors->Set_torque(TORQUE_FREE, MOTOR_TRANSLATION);
		}
		return false;
	}
}

bool Herk_Controller::Set_translation_limit(double max_position_ms, double min_position_ms)
{
    _max_translation_position_mm = max_position_ms/TRANSLATION_MM_DEG;
    _min_translation_position_mm = min_position_ms / TRANSLATION_MM_DEG;
    return false;
}

bool Herk_Controller::Set_translation_velocity(double velocity_mm_s, Herk_control ctrl)
{
    _translation_control = ctrl;
    if (_current_translation_position_mm_mutex.try_lock()) {
        _new_translation_velocity_mm_s = velocity_mm_s/TRANSLATION_MM_DEG;
        _current_translation_position_mm_mutex.unlock();
    }
    //_motors->Infinite_Turn(velocity_mm_s, 100, MOTOR_TRANSLATION);
    return false;
}

bool Herk_Controller::Set_translation_position(double position_mm, Herk_control ctrl, double time_ms, timing t)
{
    _translation_control = ctrl;
    if (position_mm > _max_translation_position_mm)
    {
        position_mm = _max_translation_position_mm;
    }
    else if (position_mm < _min_translation_position_mm)
    {
        position_mm = _min_translation_position_mm;
    }

	_translation_velocity_time_ms = time_ms;
	_new_translation_position_mm = position_mm / TRANSLATION_MM_DEG + MOTOR_TRANSLATION_BIAS;

	if (t == WAIT)
	{
		double temp;
		do {
			Get_translation_position(temp);
		} while (std::abs(temp - position_mm) < _translation_sensitivity);
	}
    //_motors->Independent_Move(position_mm, time_ms, MOTOR_TRANSLATION);
    return false;
}

bool Herk_Controller::Get_translation_velocity(double &vel)
{
    vel = _translation_velocity;
    return false;
}

bool Herk_Controller::Get_translation_position(double &_position_mm)
{
    _position_mm = (_current_translation_position_mm - MOTOR_TRANSLATION_BIAS)*TRANSLATION_MM_DEG;
    /*if (_current_translation_position_mm_mutex.try_lock())
    {
        _position_mm = _current_translation_position_mm;
        _current_translation_position_mm_mutex.unlock();
    }*/
    return false;
}

bool Herk_Controller::Get_translation_limits()
{
    return false;
}

// <

bool Herk_Controller::Set_twist_mode(Herk_mode mode)
{
    if (mode == ON)
    {
        _motors->Set_torque(TORQUE_ON, MOTOR_TWIST);
    }
	else if (mode == OFF)
	{
		_motors->Set_torque(TORQUE_FREE, MOTOR_TWIST);
	}
    return false;
}

bool Herk_Controller::Set_twist_limit(double max_position_ms, double min_position_ms)
{
    _max_twist_angle_deg = max_position_ms;
    _min_twist_angle_deg = min_position_ms;
    return false;
}

bool Herk_Controller::Set_twist_velocity(double velocity_mm_s, Herk_control ctrl)
{
    _twist_control = ctrl;
    _new_twist_velocity_deg_s = velocity_mm_s;
    //_motors->Infinite_Turn(velocity_mm_s, 100, MOTOR_TRANSLATION);
    return false;
}

bool Herk_Controller::Set_twist_position(double position_deg,  Herk_control ctrl, double time_ms, timing t)
{
    _twist_control = ctrl;
	_twist_velocity_time_ms = time_ms;
    if (position_deg > _max_twist_angle_deg)
    {
        position_deg = _max_twist_angle_deg;
    }
    else if (position_deg < _min_twist_angle_deg)
    {
        position_deg = _min_twist_angle_deg;
    }
    _new_twist_position_deg = position_deg + MOTOR_TWIST_BIAS;
    //_motors->Independent_Move(position_mm, time_ms, MOTOR_TRANSLATION);

	if (t == WAIT)
	{
		double temp;
		do {
			Get_twist_position(temp);
		} while (std::abs(temp - position_deg) < _twist_sensitivity);
	}
    return false;
}

bool Herk_Controller::Set_twist_limits()
{
    return false;
}

bool Herk_Controller::Get_twist_velocity(double &vel)
{
    vel = _twist_velocity;
    return false;
}

bool Herk_Controller::Get_twist_position(double &_position_deg)
{
    _position_deg = _current_twist_position_deg - MOTOR_TWIST_BIAS;
    /*if (_current_twist_position_deg_mutex.try_lock())
    {
        _position_mm = _current_twist_position_deg;
        _current_twist_position_deg_mutex.unlock();
    }*/
    return false;
}

bool Herk_Controller::Get_twist_limits()
{
    return false;
}

// <

bool Herk_Controller::Set_bend_mode(Herk_mode mode)
{
    if (mode == ON)
    {
        _motors->Set_torque(TORQUE_ON, MOTOR_BEND);
    }
	else if (mode == OFF)
	{
		_motors->Set_torque(TORQUE_FREE, MOTOR_BEND);
	}
    return false;
}

bool Herk_Controller::Set_bend_limit(double max_angle_ms, double min_angle_ms)
{
    _max_bend_angle_deg = max_angle_ms;
    _min_bend_angle_deg = min_angle_ms;
    return false;
}

bool Herk_Controller::Set_bend_velocity(double velocity_mm_s, Herk_control ctrl)
{
    _bend_control = ctrl;
    _new_bend_velocity_deg_s = velocity_mm_s;
    //_motors->Infinite_Turn(velocity_mm_s, 100, MOTOR_TRANSLATION);
    return false;
}

bool Herk_Controller::Set_bend_position(double position_deg, Herk_control ctrl, double time_ms, timing t)
{
    _bend_control = ctrl;
	_bend_velocity_time_ms = time_ms;
    if (position_deg > _max_bend_angle_deg)
    {
        position_deg = _max_bend_angle_deg;
    }
    else if (position_deg < _min_bend_angle_deg)
    {
        position_deg = _min_bend_angle_deg;
    }
    _new_bend_position_deg = position_deg / MOTOR_BEND_M_B + MOTOR_BEND_BIAS;
    if (_bend_control == POSITION)
    {
		std::cout << "COMMAND FOR BEND MOTOR SENT....!!!!!!!!!!" << std::endl;
		//_motors->Independent_Move(_new_bend_position_deg, _bend_velocity_time_ms, MOTOR_BEND);
    }
	//if (t == WAIT)
		//wait_for_value(position_deg, &Herk_Controller::Get_bend_position, _bend_sensitivity);

	if (t == WAIT)
	{
		double temp;
		do {
			Get_bend_position(temp);
		} while (std::abs(temp - position_deg) < _bend_sensitivity);
	}
    return false;
}

bool Herk_Controller::Set_bend_limits()
{
    return false;
}

bool Herk_Controller::Get_bend_velocity(double &vel)
{
    vel = _bend_velocity;
    return false;
}

bool Herk_Controller::Get_bend_position(double &_position_mm)
{
    _position_mm = (_current_bend_position_deg)*MOTOR_BEND_M_B;
    /*if (_current_bend_position_deg_mutex.try_lock())
    {
        _position_mm = _current_bend_position_deg;
        _current_bend_position_deg_mutex.unlock();
    }*/
    return false;
}

bool Herk_Controller::Get_bend_limits()
{
    return false;
}

void Herk_Controller::recieve_info_thread()
{
    try
    {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
		Home(true);
        while (true)
        {
            double temp, tempv;
            
            // Translation
			if (_is_initialized)
			{
				temp = _translation_position;
				_motors->Get_pos_angle(temp, MOTOR_TRANSLATION);
				_motors->Get_velocity_angle(tempv, MOTOR_TRANSLATION);
				_translation_position = temp;
				_translation_velocity = tempv * TRANSLATION_MM_DEG;
				_translation_direction = direction((1 * (tempv < 0) + 2 * (tempv > 0)));
				temp -= MOTOR_TRANSLATION_BIAS;


				if (temp > _max_translation_position_mm || temp < _min_translation_position_mm)
				{
					_motors->Infinite_Turn(0, 0, MOTOR_TRANSLATION);
					std::cout << "Motor Translation out of range" << std::endl;
				}
				else if (_translation_control == VEL_POS) {
					double ntp = _new_translation_position_mm;
					double ntv = std::abs(_new_translation_velocity_mm_s);
					_current_translation_position_mm = temp * TRANSLATION_MM_DEG;

					if (std::abs(temp - ntp + MOTOR_TRANSLATION_BIAS) < _translation_sensitivity)
					{
						_motors->Infinite_Turn(0, 0, MOTOR_TRANSLATION);
						std::cout << "Froze" << std::endl;
					}
					else if ((temp - ntp + MOTOR_TRANSLATION_BIAS) <= -1 * _translation_sensitivity)
					{
						_motors->Infinite_Turn(ntv, _translation_velocity_time_ms, MOTOR_TRANSLATION);
						std::cout << "Forward" << std::endl;
					}
					else if ((temp - ntp + MOTOR_TRANSLATION_BIAS) >= _translation_sensitivity)
					{
						_motors->Infinite_Turn(-1 * ntv, _translation_velocity_time_ms, MOTOR_TRANSLATION);
						std::cout << "Backward" << std::endl;
					}
					else {
						std::cout << "Position Unkown" << std::endl;
					}
				}
				else if (_translation_control == VELOCITY)
				{
					_motors->Infinite_Turn(_new_translation_velocity_mm_s, _translation_velocity_time_ms, MOTOR_TRANSLATION);
				}
				else if (_translation_control == POSITION)
				{
					_motors->Independent_Move(_new_translation_position_mm, _translation_velocity_time_ms, MOTOR_TRANSLATION);
				}
			}
			


            // Twist
			if (_is_initialized)
			{
				temp = _twist_position;
				_motors->Get_pos_angle(temp, MOTOR_TWIST);
				_motors->Get_velocity_angle(tempv, MOTOR_TWIST);
				_twist_position = temp;
				_twist_velocity = tempv;
				_twist_direction = direction((1 * (tempv < 0) + 2 * (tempv > 0)));
				temp -= MOTOR_TWIST_BIAS;
				_current_twist_position_deg = temp;
				double nwp = _new_twist_position_deg;
				double nwv = std::abs(_new_twist_velocity_deg_s);
				if (temp > _max_twist_angle_deg || temp < _min_twist_angle_deg)
				{
					_motors->Infinite_Turn(0, 0, MOTOR_TWIST);
				}
				else if (_twist_control == VEL_POS && !(nwp > _max_twist_angle_deg || nwp < _min_twist_angle_deg))
				{
					if (std::abs(temp - nwp + MOTOR_TWIST_BIAS) < _twist_sensitivity*std::abs(tempv)/5)
					{
						_motors->Infinite_Turn(0, 0, MOTOR_TWIST);
					}
					else if ((temp - nwp + MOTOR_TWIST_BIAS) <= -1 * _twist_sensitivity * std::abs(tempv) / 5)
					{
						_motors->Infinite_Turn(nwv, _twist_velocity_time_ms, MOTOR_TWIST);
						//_motors->Independent_Move(_new_twist_position_deg, 200, MOTOR_TWIST);
					}
					else if ((temp - nwp + MOTOR_TWIST_BIAS) >= _twist_sensitivity * std::abs(tempv) / 5)
					{
						_motors->Infinite_Turn(-1 * nwv, _twist_velocity_time_ms, MOTOR_TWIST);
						//_motors->Independent_Move(_new_twist_position_deg, 200, MOTOR_TWIST);
					}
				}
				else if (_twist_control == VELOCITY)
				{
					_motors->Infinite_Turn(_new_twist_velocity_deg_s, _twist_velocity_time_ms, MOTOR_TWIST);
				}
				else if (_twist_control == POSITION || _twist_control == VEL_POS)
				{
					_motors->Independent_Move(_new_twist_position_deg, _twist_velocity_time_ms, MOTOR_TWIST);
				}
			}

            // Bend
			if (_is_initialized)
			{
				temp = _bend_position;
				_motors->Get_pos_angle(temp, MOTOR_BEND);
				_motors->Get_velocity_angle(tempv, MOTOR_BEND);
				_bend_position = temp;
				_bend_velocity = tempv;
				_bend_direction = direction((1 * (tempv < 0) + 2 * (tempv > 0)));
				temp -= MOTOR_BEND_BIAS;
				_current_bend_position_deg = temp;
				double nbp = _new_bend_position_deg;
				double nbv = std::abs(_new_bend_velocity_deg_s);
				
				if (temp > _max_bend_angle_deg || temp < _min_bend_angle_deg)
				{
					_motors->Infinite_Turn(0, 0, MOTOR_BEND);
				}
				else if (_bend_control == VEL_POS)
				{
					if (std::abs(temp - nbp + MOTOR_BEND_BIAS) < _bend_sensitivity)
					{
						_motors->Infinite_Turn(0, 0, MOTOR_BEND);
					}
					else if ((temp - nbp + MOTOR_BEND_BIAS) <= -1 * _bend_sensitivity)
					{
						_motors->Infinite_Turn(nbv, _bend_velocity_time_ms, MOTOR_BEND);
					}
					else if ((temp - nbp + MOTOR_BEND_BIAS) >= _bend_sensitivity)
					{
						_motors->Infinite_Turn(-1 * nbv, _bend_velocity_time_ms, MOTOR_BEND);
					}
				}
				else if (_bend_control == VELOCITY)
				{
					_motors->Infinite_Turn(_new_bend_velocity_deg_s, _bend_velocity_time_ms, MOTOR_BEND);
				}
				else if (_bend_control == POSITION)
				{
					_motors->Independent_Move(_new_bend_position_deg, _bend_velocity_time_ms, MOTOR_BEND);
				}
			}
            

            boost::this_thread::interruption_point();
            //boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
			//std::cout << "tab\n";
        }
    }
    catch (boost::thread_interrupted& interruption)
    {
        std::cout << "THREAD STOPPED";
        // thread was interrupted, this is expected. 
        //std::cout << "interepted\n";
    }
    catch (std::exception& e)
    {
        std::cout << "THREAD STOPPED Ex";
        // an unhandled exception reached this point, this constitutes an error 
        //std::cout << "exception\n";
    }
}
