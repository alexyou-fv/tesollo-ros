#include "modbusTCP.hpp"
#include "delto_2f_enum.hpp"

#include <thread>
#include <mutex>

class Delto2F_TCP
{

public:
    Delto2F_TCP(std::string &ip, uint16_t port)
        : ip_(ip), port_(port)
    {
        ModbusClient_ = std::make_unique<ModbusClient>(ip, port);
    };

    virtual ~Delto2F_TCP() = default;
    void connect();
    double get_position();
    bool set_postion(double position);
    void grasp(bool do_grasp);

    private:
    std::unique_ptr<ModbusClient> ModbusClient_;
    std::string ip_;
    uint16_t port_;
    std::mutex mutex_;
};

void Delto2F_TCP::connect()
{
    ModbusClient_->connect();
}

double Delto2F_TCP::get_position()
{
    std::vector<int16_t> positions;

    positions = ModbusClient_->readInputRegisters(MOTOR_CURRENT_POSITION, 1);

 
    float positions_float = positions[0] / 10.0;
    

    return positions_float;
}


bool Delto2F_TCP::set_postion(double position)
{
    int position_int;


    position_int = position *10;
    
    ModbusClient_->writeSingleRegister(OPEN_POSITION, position_int);
}

void Delto2F_TCP::grasp(bool do_grasp)
{
    if (do_grasp)
    {
        ModbusClient_->writeSingleCoil(GRASP, 1);
    }
    else
    {
        ModbusClient_->writeSingleCoil(GRASP, 0);
    }
}
