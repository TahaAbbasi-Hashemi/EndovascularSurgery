//Herk.hpp
#include <iostream>
//Boost
#include <boost/shared_ptr.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp>
#include <boost/algorithm/hex.hpp>
//Std
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "Serial.hpp"
#include "Buffered_async_serial.hpp"

#include "newLogger.hpp" // Taha Logger!!

#define RED 0x04
#define GREEN 0x01
#define BLUE 0x02

static const uint8_t HERK_TIMEOUT = 500;
static const uint8_t HERK_BUFFER = 223;  // maximum allowable packet size

static const uint8_t ALL_MOTORS = 0xFE;

static const uint8_t EEP_WRITE = 1;
static const uint8_t EEP_READ = 2;
static const uint8_t RAM_WRITE = 3;
static const uint8_t RAM_READ = 4;
static const uint8_t I_JOG = 5;
static const uint8_t S_JOG = 6;
static const uint8_t STAT = 7;
static const uint8_t ROLLBACK = 8;
static const uint8_t REBOOT = 9;

static const uint8_t TORQUE_ON = 0x60;
static const uint8_t BREAK_ON = 0x40;
static const uint8_t TORQUE_FREE = 0x00;

class Herk
{
public:
    Herk(std::string port, int baud = 200000, int id = 1);
    ~Herk();
    bool Is_initialized(void);
    bool Close();
	static bool Test_com(std::string port, int baud = 200000);

    bool Set_motor(int id); // Do this one
    bool Get_pos_angle(double& pos, int id = -1);
    bool Get_pos_raw(int& pos, int id = -1);
    bool Get_dif_angle(double& pos, int id = -1);
    bool Get_velocity_angle(double& vel, int id);
    bool Set_colour(int colour, int id = -1);
    bool Independent_Move(double angle, double ms, int id = -1);
    bool Independent_Move_Raw(int pos, int playtime, int id);
    bool Infinite_Turn(double velocity, double ms, int id = -1);
    bool Independent_Continous(int direction, int playtime, int id = -1);
    bool Synchronized_Move(double angle, int id = -1);
    bool Reboot(int id = -1);
    bool Status(int, int& error, int& detail);
    void Raw_message(unsigned char c[], std::string &out);
    bool Read_RAM(std::string &out, int id = -1);
    bool Set_torque(uint8_t condition = TORQUE_FREE, int id = -1);
    bool Set_default_pos(int id = -1);

private:

    bool Send_and_wait_for_input(unsigned char* c, unsigned char out[HERK_BUFFER]);
    bool Send_input(unsigned char* c);
    bool Generate_cmd(int id, int cmd, unsigned char* data, int len, unsigned char* out);

    bool _is_initialized;
    bool _still_ok;
    bool _is_running;
    bool _message_ok;
    int _id;
    std::string _temp;
	std::string _com;
	int _baud;
    boost::asio::io_service m_ioService;
	boost::asio::deadline_timer _timer;
    boost::shared_ptr<SerialPort> _Herk;
    boost::shared_ptr<boost::asio::serial_port>	_serial;
	boost::shared_ptr<Buffered_async_serial>	_aserial;
	std::future<void> _async;
};

