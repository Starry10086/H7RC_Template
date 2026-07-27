#pragma once

#include <cstdint>
#include <atomic>
#include <algorithm>
#include <cstring>
#include <bit>
#include <chrono>

/*
@breif DM电机返回的和设置的参数均以经过减速比转换后的输出端参数
*/

namespace librmcs::device {
class DmMotor {
public:
    enum class Type : uint8_t {DM_J4310_2EC,DM_J4340_2EC};
    enum class ControlMode : uint8_t {MIT,Pos_Vel,Vel};

    struct Config{
        explicit Config(Type motor_type)
        : control_mode(ControlMode::MIT)
        , reduction_ratio(reduction_ratio_init(motor_type))
        , KP_Max(500.0)
        , KD_Max(5.0)
        , Pos_Max(12.5)
        , Vel_Max(45.0)
        , Tor_Max(max_torque_init(motor_type))
        ,reversed(false)
        ,multi_turn_angle_enabled(false) {}

        Config& set_reversed() { return reversed = true, *this; }
        Config& set_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_reduction_ratio(float value) { return reduction_ratio = value, *this; }
        Config& set_control_mode(ControlMode value) { return control_mode = value, *this; }
    
    ControlMode control_mode;
    float reduction_ratio;
    float KP_Max;
    float KD_Max;
    float Pos_Max;
    float Vel_Max;
    float Tor_Max;
    bool reversed;
    bool multi_turn_angle_enabled;
    };

    DmMotor()
    :ERR(0)
    ,Angle(0.0)
    ,VEL(0.0)
    ,TOR(0.0)
    ,T_MOS(0.0)
    ,T_Rotor(0.0)
    ,reversed_sign(1.0) {}
    
    explicit DmMotor(const Config& config)
    :ERR(0)
    ,Angle(0.0)
    ,VEL(0.0)
    ,TOR(0.0)
    ,T_MOS(0.0)
    ,T_Rotor(0.0)
    ,reversed_sign(1.0) {
        configure(config);
    }

    //禁用拷贝
    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;

    void configure(const Config& config) {
        control_mode_ = config.control_mode;
        reduction_ratio = config.reduction_ratio;
        KP_Max = config.KP_Max;
        KD_Max = config.KD_Max;
        Pos_Max = config.Pos_Max;
        Vel_Max = config.Vel_Max;
        Tor_Max = config.Tor_Max;
        reversed = config.reversed;
        multi_turn_angle_enabled = config.multi_turn_angle_enabled;
        reversed_sign = config.reversed ? -1.0 : 1.0;
    }

    void store_status(uint64_t can_data) {
        can_data_.store(can_data, std::memory_order_relaxed);
        feedback_seen_ = true;
        last_feedback_time_ = std::chrono::steady_clock::now();
    }
    bool online(std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) const {
        if(!feedback_seen_)
            return false;
        auto now = std::chrono::steady_clock::now();
        return (now - last_feedback_time_) <= timeout;
    }

    void update_status() {
        auto feedback = std::bit_cast<DmMotorFeedback>(can_data_.load(std::memory_order_acquire));

        uint16_t POS_ = feedback.Pos();
        uint16_t VEL_ = feedback.Vel();
        uint16_t TOR_ = feedback.Tor();
        uint8_t T_MOS_ = feedback.T_MOS();
        uint8_t T_Rotor_ = feedback.T_Rotor();
        
        ERR = feedback.ERR();
        reversed_sign = reversed ? -1.0 : 1.0;

        Angle = uint_to_float(POS_, -Pos_Max, Pos_Max, 16) * reversed_sign;
        VEL = uint_to_float(VEL_, -Vel_Max, Vel_Max, 12) * reversed_sign;
        TOR = uint_to_float(TOR_, -Tor_Max, Tor_Max, 12) * reversed_sign;
        T_MOS = uint_to_float(T_MOS_, 0.0, 100.0, 8);
        T_Rotor = uint_to_float(T_Rotor_, 0.0, 100.0, 8);
    }

    uint64_t generate_command(float target_pos, float target_vel, float target_Kp,float target_Kd,float target_tff) {
        uint8_t can_data[8] = {0};
        uint64_t result_can_data = 0;
        target_pos *= reversed_sign;
        target_vel *= reversed_sign;
        target_tff *= reversed_sign;

        target_pos = std::clamp(target_pos, -Pos_Max, Pos_Max);
        target_vel = std::clamp(target_vel, -Vel_Max, Vel_Max);
        target_tff = std::clamp(target_tff, -Tor_Max, Tor_Max);
        target_Kp = std::clamp(target_Kp, 0.0f, KP_Max);
        target_Kd = std::clamp(target_Kd, 0.0f, KD_Max);
        
        uint16_t target_pos_uint = float_to_uint(target_pos, -Pos_Max, Pos_Max, 16);
        uint16_t target_vel_uint = float_to_uint(target_vel, -Vel_Max, Vel_Max, 12);
        uint16_t target_tff_uint = float_to_uint(target_tff, -Tor_Max, Tor_Max, 12);
        uint16_t target_Kp_uint = float_to_uint(target_Kp, 0.0f, KP_Max, 12);
        uint16_t target_Kd_uint = float_to_uint(target_Kd, 0.0f, KD_Max, 12);

        switch(control_mode_){
            case ControlMode::MIT:
                can_data[0] = (target_pos_uint >> 8) & 0xff;
                can_data[1] = target_pos_uint & 0xff;
                can_data[2] = (target_vel_uint >> 4) & 0xff;
                can_data[3] = (target_vel_uint & 0x0f) << 4 | ((target_Kp_uint >> 8 & 0x0f));
                can_data[4] = target_Kp_uint & 0xff;
                can_data[5] = (target_Kd_uint >> 4) & 0xff;
                can_data[6] = (target_Kd_uint & 0x0f) << 4 | ((target_tff_uint >> 8) & 0x0f);
                can_data[7] = target_tff_uint & 0xff;
            break;
            case ControlMode::Pos_Vel:
                memcpy(&can_data[0],&target_pos,sizeof(float));// 从 target_pos 的内存位置复制 4 字节到 can_data[0] 开始的位置
                memcpy(&can_data[4],&target_vel,sizeof(float));
            break;
            case ControlMode::Vel:
                memcpy(&can_data[0],&target_vel,sizeof(float));
                can_data[4] = 0x00;
                can_data[5] = 0x00;
                can_data[6] = 0x00;
                can_data[7] = 0x00;
            break;
            default: break;
        }
        memcpy(&result_can_data,can_data,8);
        return result_can_data;
    }
     
    static uint64_t Enable_Motor() {
        uint8_t can_data[8] = {0};
        can_data[0] = 0xFF;
        can_data[1] = 0xFF;
        can_data[2] = 0xFF;
        can_data[3] = 0xFF;
        can_data[4] = 0xFF;
        can_data[5] = 0xFF;
        can_data[6] = 0xFF;
        can_data[7] = 0xFC;
        auto result_can_data = std::bit_cast<uint64_t>(can_data);
        return result_can_data;
    }
    static uint64_t Disable_Motor() {
        uint8_t can_data[8] = {0};
        can_data[0] = 0xFF;
        can_data[1] = 0xFF;
        can_data[2] = 0xFF;
        can_data[3] = 0xFF;
        can_data[4] = 0xFF;
        can_data[5] = 0xFF;
        can_data[6] = 0xFF;
        can_data[7] = 0xFD;

        auto result_can_data = std::bit_cast<uint64_t>(can_data);
        return result_can_data;
    }
    static uint64_t Save_Zero_Position() {
        uint8_t can_data[8] = {0};
        can_data[0] = 0xFF;
        can_data[1] = 0xFF;
        can_data[2] = 0xFF;
        can_data[3] = 0xFF;
        can_data[4] = 0xFF;
        can_data[5] = 0xFF;
        can_data[6] = 0xFF;
        can_data[7] = 0xFE;

        auto result_can_data = std::bit_cast<uint64_t>(can_data);
        return result_can_data;
    }
    static uint64_t Clear_Error() {
        uint8_t can_data[8] = {0};
        can_data[0] = 0xFF;
        can_data[1] = 0xFF;
        can_data[2] = 0xFF;
        can_data[3] = 0xFF;
        can_data[4] = 0xFF;
        can_data[5] = 0xFF;
        can_data[6] = 0xFF;
        can_data[7] = 0xFB;

        auto result_can_data = std::bit_cast<uint64_t>(can_data);
        return result_can_data;
    }
    float angle() const { return Angle; }
    float velocity() const { return VEL; }
    float torque() const { return TOR; }
    float temperature_mos() const { return T_MOS; }
    float temperature_rotor() const { return T_Rotor; }
    float max_torque() const { return Tor_Max; }
    uint8_t get_err() const { return ERR; }


private:
    static float reduction_ratio_init(Type motor_type) {
        switch (motor_type) {
            case Type::DM_J4310_2EC: return 10.0;
            case Type::DM_J4340_2EC: return 40.0;
            default: return 0.0;
        }
    }
    static float max_torque_init(Type motor_type) {
        switch (motor_type) {
            case Type::DM_J4310_2EC: return 12.5;
            case Type::DM_J4340_2EC: return 27.0;
            default: return 0.0;
        }
    }
    static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
    }

    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        if(x > x_max) x=x_max;
        else if(x < x_min) x= x_min;
        return static_cast<uint16_t>((x - offset) * static_cast<float>((1 << bits) - 1) / span);
    }

    //强制结构体按照uint64_t对齐，因为要用std::bit_cast转换为uint64_t
    struct alignas(uint64_t) DmMotorFeedback {
        uint8_t data[8];
        uint8_t ID() const {return (data[0] >> 4) & 0x0f;}
        uint8_t ERR() const{return (data[0] & 0x0f);}
        uint16_t Pos() const{return (data[1] << 8) | data[2];}
        uint16_t Vel() const{return (data[3] << 4) | data[4] >> 4;}
        uint16_t Tor() const{return ((data[4] & 0x0f)<< 8) | data[5];}
        uint8_t T_MOS() const{return data[6];}
        uint8_t T_Rotor() const{return data[7];}
    };
    
    std::atomic<uint64_t> can_data_ = 0;
    uint8_t ERR;
    float Angle;
    float VEL;
    float TOR;
    float T_MOS;
    float T_Rotor;
    float reversed_sign;
    
    ControlMode control_mode_;
    float reduction_ratio;
    float KP_Max;
    float KD_Max;
    float Pos_Max;
    float Vel_Max;
    float Tor_Max;
    bool reversed;
    bool multi_turn_angle_enabled;
    bool feedback_seen_ = false;
    std::chrono::steady_clock::time_point last_feedback_time_;

};
}
