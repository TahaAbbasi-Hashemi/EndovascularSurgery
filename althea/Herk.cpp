#include <iostream>
#include "Herk.hpp"



Herk::Herk(std::string port, int baud, int id) :
	_timer(m_ioService),
	_com(port),
	_baud(baud)
{
    _serial.reset(new boost::asio::serial_port(m_ioService));
    _still_ok = true;
    _id = id;

    //_serial->open(port);
    _serial->open(port);
	_serial->set_option(boost::asio::serial_port_base::baud_rate(baud));
    // following code enables torque
    /*unsigned char mv[HERK_BUFFER];
    unsigned char datas[3] = {0x34, 0x01, 0x60};
    Generate_cmd(1, RAM_WRITE, datas, 3, mv);
    Send_input(mv);*/
    //Set_torque(TORQUE_ON);

}

Herk::~Herk() {
	if (_is_running) {
    }
	_serial->close();
}

bool Herk::Send_and_wait_for_input(unsigned char* c, unsigned char out[HERK_BUFFER]) {
	//makeLog("Async not", BASIC);
    Send_input(c);
	//makeLog("Async Error", BASIC);
	_async = std::async(std::launch::async, [this, out] {; });// 
	_serial->read_some(boost::asio::buffer(out, HERK_BUFFER));
	//Sleep(50);
	if (false) {
		_async.~future();
		return false;
	}

    unsigned char ch1 = (out[2] ^ out[3] ^ out[4]);
    for (int i = 7; i<out[2]; i++) {
        ch1 ^= out[i];
    }
    unsigned char ch2 = ~ch1 & 0xFE;
    ch1 &= 0xFE;

    if (out[5] == ch1 && out[6] == ch2) {
        return true;
    }
    return false;
}


bool Herk::Send_input(unsigned char c[]) {
    _serial->write_some(boost::asio::buffer(c, c[2]));
    return true;
}

// Used to easily form the message structure given the specific arguments
bool Herk::Generate_cmd(int id, int cmd, unsigned char* data, int len, unsigned char* out) {
    int size = len + 7;
    int ch1 = (size ^ id ^ cmd);
    int ch2 = (size ^ id ^ cmd);
    for (int i = 0; i<len; i++) {
        ch1 ^= data[i];
        ch2 ^= data[i];
    }
    ch1 &= 0xFE;
    ch2 = ~ch2 & (unsigned int) 0xFE;
    char temp[7] = { 0xFF, 0xFF, size, id, cmd, ch1, ch2 };

    for (int i = 0; i<size; i++) {
        if (i < 7)
        {
            out[i] = temp[i];
        }
        else
        {
            out[i] = data[i - 7];
        }
    }

    return true;
}


// To be implemented
bool Herk::Is_initialized() {
    return _still_ok;
}

bool Herk::Set_motor(int id) {
    _id = id;
    return true;
}

// get the absolute position from _ to _ *(needs to be tested for exact range)
bool Herk::Get_pos_angle(double& pos, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x3C, 0x02 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    _message_ok = Send_and_wait_for_input(mv, value);
    Raw_message(value, _temp);
    if (_message_ok) {
        int raw_pos = ((unsigned int)(value[10])) * 256 + ((unsigned int)(value[9]));
        //if ((int)value[10] < 0)
        //	raw_pos += 256;	
        //std::cout << "Pos: " << (int)pos <<  "R1: " << (int)value[9] << ";  "<< "R2: " << (int)value[10] << ";    " << std::endl;
        pos = (raw_pos - 9904) / 36.00;
        return true;
    }
    return false;
}

// get the absolute velocity from _ to _ *(needs to be tested for exact range)
bool Herk::Get_velocity_angle(double& vel, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x3E, 0x02 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    _message_ok = Send_and_wait_for_input(mv, value);
    //Raw_message(value, _temp);
    if (_message_ok) {
		int raw_vel = (((unsigned int)(value[10]) * 256 + (unsigned int)(value[9])) );
        //if ((int)value[10] < 0)
        //	raw_pos += 256;	
        //std::cout << "Pos: " << (int)pos <<  "R1: " << (int)value[9] << ";  "<< "R2: " << (int)value[10] << ";    " << std::endl;
        
		if (raw_vel > (65536/2))
			raw_vel -= 65536;

		vel = raw_vel * 0.62;
        return true;
    }
    return false;
}
/*
bool Herk::Get_velocity_angle(double& vel, int id) {
	if (id == -1) id = _id;
	unsigned char mv[HERK_BUFFER];
	unsigned char datas[2] = { 0x48, 0x02 };
	Generate_cmd(id, RAM_READ, datas, 2, mv);
	unsigned char value[HERK_BUFFER];
	_message_ok = Send_and_wait_for_input(mv, value);
	Raw_message(value, _temp);
	if (_message_ok) {
		int raw_vel = ((value[10])) * 256 + ((value[9]));
		//if ((int)value[10] < 0)
		//	raw_pos += 256;	
		//std::cout << "Pos: " << (int)pos <<  "R1: " << (int)value[9] << ";  "<< "R2: " << (int)value[10] << ";    " << std::endl;
		vel = raw_vel * 0.034;
		return true;
	}
	return false;
}
*/
// get the absolute position from _ to _ *(needs to be tested for exact range)
bool Herk::Get_pos_raw(int& pos, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x3C, 0x02 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    _message_ok = Send_and_wait_for_input(mv, value);
    Raw_message(value, _temp);
    if (_message_ok) {
        pos = ((unsigned int)(value[10])) * 256 + ((unsigned int)(value[9]));
        return true;
    }
    return false;
}

bool Herk::Get_dif_angle(double& pos, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x3E, 0x02 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    Send_and_wait_for_input(mv, value);
    std::string temp;
    Raw_message(value, temp);

    int raw_pos = ((int)(value[10]) << 8) + ((int)(value[9]));
    if ((int)value[9] < 0)
        raw_pos += 256;
    //std::cout << "R: " << (int)raw_pos << ";    " << "R1: " << (int)value[10] << ";    "  << "R2: " << (int)value[9] << ";    " << std::endl;
    pos = (raw_pos) * 0.62;
    return true;
}

bool Herk::Set_colour(int colour, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[3] = { (unsigned char)0x35, (unsigned char)0x01, colour };
    Generate_cmd(id, RAM_WRITE, datas, 3, mv);
    //unsigned char value[HERK_BUFFER];
    Send_input(mv);
    return true;
}

bool Herk::Set_default_pos(int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x3C, 0x02 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    _message_ok = Send_and_wait_for_input(mv, value);

    unsigned char data[4] = { (unsigned char) 0x34, (unsigned char)0x02, value[9] & 0, value[10]&0 };
    Generate_cmd(id, EEP_WRITE, data, 4, mv);
    Send_input(mv);
    return true;
}

bool Herk::Set_torque(uint8_t condition, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[3] = { 0x34, 0x01, condition };
    Generate_cmd(id, RAM_WRITE, datas, 3, mv);
    //unsigned char value[HERK_BUFFER];
    Send_input(mv);
    return true;
}

//Hi Ibrahim, this is good (you are writing to the registers cleanly) but 
//1. There is already a nice serial class in the other program Boost (v. fast)
//2. Use static_cast not c-style cast
//3. Do not use #define to define any constants (eg HERK_BUFFER, HERK_TIMEOUT), use static const members, or just plain members #define is only for header guards and checking build options eg #if defined() ... #elif defined(LINUX) ...
//4. Most important: Need to contol VELOCITY and read position, not contyrol only position
//5. It is achieveable using pos commands as long as you can go one motion to the next without STOPPING (buffer options on Herc?) v d theta/dt ~ (p_star - p_curr)/delta t (use real timed delta t!!!). Better yet find the velocity register and write to that (need +- directional vel)

// moves the motor to a given position (angle from 0 to 360 degrees +- 1)
bool Herk::Infinite_Turn(double velocity, double ms, int id)
{
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    int speed;
    if (velocity < 0)
        speed = 0x4000 - (int)(velocity / 0.62);
    else
        speed = (int)(velocity / 0.62);
    unsigned char lsb = speed & 0xFF;
    unsigned char msb = speed >> 8;
    //std::cout << "S: " << (int)speed << ";    " << "S1: " << (int)msb << ";    " << "S: " << (int)lsb << ";    " << std::endl;
    int playtime = (int)(ms / 11.2);
    unsigned char datas[5] = { lsb, msb, (unsigned char)0x02, id, playtime };
    Generate_cmd(id, I_JOG, datas, 5, mv);
    //unsigned char d[HERK_BUFFER];
    //	Send_and_wait_for_input(mv, d);
    Send_input(mv);
    return true;
}

bool Herk::Independent_Move(double angle, double ms, int id)
{
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    int pos = (int)(angle * 36.00) + 9904;
    unsigned char lsb = pos & 0xFF;
    unsigned char msb = (pos - lsb) / 256;
    //std::cout << "Pos: " << (int)pos << ";  "<< "S1: " << (int)lsb << ";  "<< "S2: " << (int)msb << ";    "<< std::endl;
    int playtime = (int)(ms / 11.2);
    unsigned char datas[5] = { lsb, msb, (unsigned char)0x00, id, playtime };
    Generate_cmd(id, I_JOG, datas, 5, mv);
    //unsigned char d[HERK_BUFFER];
    //	Send_and_wait_for_input(mv, d);
    Send_input(mv);
    return true;
}

bool Herk::Independent_Move_Raw(int pos, int playtime, int id)
{
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char lsb = pos & 0xFF;
    unsigned char msb = (pos - lsb) / 256;
    //std::cout << "Pos: " << (int)pos << ";  "<< "S1: " << (int)lsb << ";  "<< "S2: " << (int)msb << ";    "<< std::endl;
    unsigned char datas[5] = { lsb, msb, (unsigned char)0x00, id, playtime };
    Generate_cmd(id, I_JOG, datas, 5, mv);
    //unsigned char d[HERK_BUFFER];
    //	Send_and_wait_for_input(mv, d);
    Send_input(mv);
    return true;
}

bool Herk::Independent_Continous(int direction, int playtime, int id)
{
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    int di = 0;
    if (direction < 0) {
        di = 0x40;
    }
    unsigned char datas[5] = { di, (unsigned char)0x01, (unsigned char)0x02, id, playtime };
    Generate_cmd(id, I_JOG, datas, 5, mv);
    //unsigned char d[HERK_BUFFER];
    //	Send_and_wait_for_input(mv, d);
    Send_input(mv);
    return true;
}

// moves the motor to a given position (angle from 0 to 360 degrees +- 1)
bool Herk::Synchronized_Move(double angle, int id)
{
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    int pos = (int)(angle / 0.02778) + 9903;
    unsigned char msb = pos >> 8;
    unsigned char lsb = pos;
    unsigned char datas[4] = { lsb, msb, (unsigned char)0x00, id };
    Generate_cmd(id, S_JOG, datas, 4, mv);
    //unsigned char d[HERK_BUFFER];
    //	Send_and_wait_for_input(mv, d);
    Send_input(mv);
    return true;
}

bool Herk::Reboot(int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[1];
    Generate_cmd(id, REBOOT, datas, 0, mv);
    Send_input(mv);
    return true;
}

bool Herk::Read_RAM(std::string &out, int id) {
    if (id == -1) id = _id;
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x34, 0x16 };
    Generate_cmd(id, RAM_READ, datas, 2, mv);
    unsigned char value[HERK_BUFFER];
    Send_and_wait_for_input(mv, value);
    Raw_message(value, out);
    return true;
}

bool Herk::Status(int id, int &error, int &detail) {
    unsigned char mv[HERK_BUFFER];
    unsigned char datas[2] = { 0x00, 0x40 };
    Generate_cmd(id, STAT, datas, 0, mv);
	unsigned char value[HERK_BUFFER];
	Send_and_wait_for_input(mv, value);

	//_aserial.reset(new Buffered_Async_serial(_com, _baud));
	

	error = value[7];
	detail = value[8];
    return true;
}

// DOES NOT WORK
void Herk::Raw_message(unsigned char c[HERK_BUFFER], std::string &out)
{
    std::ostringstream convert;
    char hex[10];
    char dec[10];
    for (int i = 0; i<c[2]; i++) {
        //itoa(c[i], hex, 16);
        hex[2] = '\0';
        //itoa(c[i], dec, 10);
        dec[2] = '\0';
        convert << std::setfill(' ') << std::setw(3) << i + 1 << " -> " << std::setfill('0') << std::setw(2) << hex << "  :  " << dec << std::endl;
    }
    out = convert.str();
}


