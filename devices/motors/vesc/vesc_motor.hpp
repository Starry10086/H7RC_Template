#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>

namespace device{

class VescMotor {
public:
    struct CANCommand{
        uint32_t can_id;        //扩展CAN ID
        uint64_t can_data;      // 8字节CAN数据
        uint8_t can_data_length;// 实际数据长度（4字节）
    };
    enum class STATUS_CAN_PACKET_ID : uint32_t {
        CAN_PACKET_STATUS = 9,          // ERPM, 电流, 占空比
        CAN_PACKET_STATUS_2 = 14,       // Ah 已用量, Ah 已充电量
        CAN_PACKET_STATUS_3 = 15,       // Wh 已用量, Wh 已充电量
        CAN_PACKET_STATUS_4 = 16,       // MOS管温度，电机温度，输入电流，PID位置
        CAN_PACKET_STATUS_5 = 27,       // 转速计/累积转数，输入电压
        CAN_PACKET_STATUS_6 = 58,       // ADC1, ADC2, ADC3, 遥控信号
    };
    enum class CMD_CAN_PACKET_ID : uint32_t {
        CAN_PACKET_SET_DUTY = 0,                    // 设置占空比
        CAN_PACKET_SET_CURRENT,                     // 设置电流
        CAN_PACKET_SET_CURRENT_BRAKE,               // 设置制动电流
        CAN_PACKET_SET_RPM,                         // 设置转速RPM
        CAN_PACKET_SET_POS,                         // 设置位置
        CAN_PACKET_SET_CURRENT_REL = 10,            // 设置相对电流，以百分比形式设置电流
        CAN_PACKET_SET_CURRENT_BRAKE_REL,           // 设置相对制动电流，以百分比形式设置制动电流
        CAN_PACKET_SET_CURRENT_HANDBRAKE,           // 设置手刹电流，用于保持电机静止
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,       // 设置相对手刹电流，以百分比形式设置手刹电流
        CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,  //仅用于枚举类型占位，保证枚举类型为32位，不用于实际命令
    };
    struct Config{
        Config()
        : pole_pairs(21)      // U10 电机极对数为 21
        , kt_rough(0.064)     // U10 电机扭矩常数约 0.064 Nm/A ， XC5500 0.0189 Nm/A
        , max_vel(60.0)       // 最大速度 60 rad/s
        , max_tor(1.5)       // U10 最大扭矩 1.5 Nm              XC5500 最大扭矩 0.87 Nm
        , encoder_zero_point(0.0)   // 零点位置
        {}

        Config& set_pole_pairs(int value) { return pole_pairs = value, *this;}
        Config& set_kt_rough(float value) { return kt_rough = value, *this;}
        Config& set_max_vel(float value) { return max_vel = value, *this;}
        Config& set_max_tor(float value) { return max_tor = value, *this;}
        Config& set_reversed() { return reversed = true, *this;}
        Config& set_encoder_zero_point(float value) { return encoder_zero_point = value, *this;}    // 手动设置0点位置
        Config& auto_set_zero_point() { return auto_set_zero_point_ = true,*this;}                  // 设置上电位置为0点
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this;}
        

        int pole_pairs;     // 极对数
        float kt_rough;     // 转矩常数,反映电流和扭矩的关系
        float max_vel;      // 最大速度，rad/s
        float max_tor;      // 最大扭矩，Nm
        float encoder_zero_point;   // 零点位置
        bool reversed = false;      // 是否反转
        bool auto_set_zero_point_ = false;   // 是否启用上电自动确认零点位置
        bool multi_turn_angle_enabled = false;  // 是否启用多圈控制
    };

    VescMotor() 
        : rpm(0.0)
        , pos(0.0)
        , current(0.0)
        , temperature(0.0)
        , first_update_(true)
        , total_turns_(0)
        , last_raw_pos_deg_(0.0)
        , encoder_zero_point_(0.0){
        can_data_status_1_.store(0, std::memory_order_relaxed);
        can_data_status_4_.store(0, std::memory_order_relaxed);
    }
    explicit VescMotor(const Config& config)
        : rpm(0.0), pos(0.0), current(0.0), temperature(0.0){
            config_para(config);
        }

    VescMotor(const VescMotor&) = delete;
    VescMotor& operator=(const VescMotor&) = delete;
    void config_para(const Config& config){
        pole_pairs = config.pole_pairs;
        kt_rough = config.kt_rough;
        sign_reversed = config.reversed ? -1.0f : 1.0f;
        max_vel = config.max_vel;
        max_tor = config.max_tor;
        encoder_zero_point_ = config.encoder_zero_point;
        auto_set_zero_point = config.auto_set_zero_point_;
        enable_multi_turn_angle = config.multi_turn_angle_enabled;
    }
    void store_status(uint32_t can_id, uint64_t can_data){
        STATUS_CAN_PACKET_ID cmd = static_cast<STATUS_CAN_PACKET_ID>((can_id >> 8) & 0xFF);
        motor_id = can_id & 0xFF;
        feedback_seen_ = true;
        switch (cmd) {
            case STATUS_CAN_PACKET_ID::CAN_PACKET_STATUS:
                can_data_status_1_.store(can_data, std::memory_order_relaxed);
            break;
            case STATUS_CAN_PACKET_ID::CAN_PACKET_STATUS_4:
                can_data_status_4_.store(can_data, std::memory_order_relaxed);
            break;
            default:
            break;
        }
        last_feedback_time_ = std::chrono::steady_clock::now();
    }
    bool online(std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) const {
        if(!feedback_seen_)
            return false;
        auto now = std::chrono::steady_clock::now();
        return (now - last_feedback_time_) <= timeout;
    }
    void update_status(){
        auto feedback_1 = std::bit_cast<Status_Message_1_FeedBack>(can_data_status_1_.load(std::memory_order_acquire));
        auto feedback_4 = std::bit_cast<Status_Message_4_FeedBack>(can_data_status_4_.load(std::memory_order_acquire));

        rpm = static_cast<float>(sign_reversed) * static_cast<float>(feedback_1.ERPM()) / static_cast<float>(pole_pairs);
        current = static_cast<float>(feedback_1.Toal_Current());
        temperature = static_cast<float>(feedback_4.Motor_Temp());
        
        if(first_update_ && auto_set_zero_point){
            encoder_zero_point_ = feedback_4.PID_Pos();
        }
        // 获取原始单圈角度（0-360度）
        float raw_pos_deg = feedback_4.PID_Pos() - encoder_zero_point_;
        if(raw_pos_deg < 0)
            raw_pos_deg += 360.0f;
        if(first_update_){
            last_raw_pos_deg_ = raw_pos_deg;
            first_update_ = false;
        }
        float diff = raw_pos_deg - last_raw_pos_deg_;
        if(diff > 180.0f)
            total_turns_--;
        else if(diff < -180.0f)
            total_turns_++;
        last_raw_pos_deg_ = raw_pos_deg;
        float final_deg = 0.0f;
        if(enable_multi_turn_angle)
            final_deg = (static_cast<float>(total_turns_) * 360.0f) + raw_pos_deg;
        else
            final_deg = raw_pos_deg;

        pos = static_cast<float>(sign_reversed) * final_deg
            * (std::numbers::pi_v<float> / 180.0f);
    }

    CANCommand generate_command_current(uint8_t motor_id,double torque){
        if (std::isnan(torque)) {
            return CANCommand{0, 0, 0};
        } 
        
        torque = std::clamp(static_cast<float>(torque), -max_tor, max_tor);
        double current_cmd = torque_to_current(torque);
        int32_t current_raw = static_cast<int32_t>(std::round(current_cmd * 1000.0)); // 电流以mA为单位发送
        uint32_t can_id = motor_id | (static_cast<uint32_t>(CMD_CAN_PACKET_ID::CAN_PACKET_SET_CURRENT) << 8);

        return pack_int32(can_id, current_raw);
    }

    CANCommand generate_command_vel(uint8_t motor_id, double rad_s) const {
        return generate_command_vel(motor_id, rad_s, static_cast<double>(max_vel));
    }

    CANCommand generate_command_vel(uint8_t motor_id,double rad_s,double max_rad_s) const {
        if (std::isnan(rad_s)) {
            return CANCommand{0, 0, 0};
        }   
       
        rad_s = sign_reversed * std::clamp(rad_s, -max_rad_s, max_rad_s);
        float rpm = static_cast<float>(rad_s) * 60.0f
            / (2.0f * std::numbers::pi_v<float>);
        int32_t erpm = static_cast<int32_t>(rpm * static_cast<float>(pole_pairs));
        uint32_t can_id = motor_id | (static_cast<uint32_t>(CMD_CAN_PACKET_ID::CAN_PACKET_SET_RPM) << 8);
    
        // VESC期望big-endian格式的int32
        // uint64_t can_data = (static_cast<uint64_t>(erpm & 0xFF) << 24) |
        //                     (static_cast<uint64_t>((erpm >> 8) & 0xFF) << 16) |
        //                     (static_cast<uint64_t>((erpm >> 16) & 0xFF) << 8) |
        //                     (static_cast<uint64_t>((erpm >> 24) & 0xFF));
    
        return pack_int32(can_id, erpm); 
    }

    CANCommand generate_command_pos(uint8_t motor_id,double rad) const {
        if (std::isnan(rad)) {
            return CANCommand{0, 0, 0};
        }   
        
        double degree = sign_reversed * rad * 180.0 / std::numbers::pi;
        int32_t send_val = static_cast<int32_t>(degree * 1000000.0);
        uint32_t can_id = motor_id | (static_cast<uint32_t>(CMD_CAN_PACKET_ID::CAN_PACKET_SET_POS) << 8);
        
        // VESC期望big-endian格式的int32
        // uint64_t can_data = (static_cast<uint64_t>(send_val & 0xFF) << 24) |
        //                     (static_cast<uint64_t>((send_val >> 8) & 0xFF) << 16) |
        //                     (static_cast<uint64_t>((send_val >> 16) & 0xFF) << 8) |
        //                     (static_cast<uint64_t>((send_val >> 24) & 0xFF));
    
        return pack_int32(can_id, send_val);
    }

    float get_velocity() const { 
        float vel = rpm * 2 * std::numbers::pi_v<float> / 60.0f;
        return std::clamp(vel, -max_vel, max_vel);  // 限制在 ±max_vel rad/s
    }  
    float get_position() const { return pos; }
    float get_current() const { return current; }
    float get_temperature() const { return temperature; }

private:
    // current = torque / Kt 电流 = 扭矩 / 转矩常数
    double torque_to_current(double torque, double max_current = 43.0)  const{
           if (std::isnan(torque)) 
                return 0.0;
           return sign_reversed * std::clamp(torque / kt_rough, -max_current, max_current);
    }

    CANCommand pack_int32(uint32_t can_id, int32_t val) const {
        uint8_t buffer[8] = {0};
        buffer[0] = (val >> 24) & 0xFF;
        buffer[1] = (val >> 16) & 0xFF;
        buffer[2] = (val >> 8) & 0xFF;
        buffer[3] = val & 0xFF;
        
        uint64_t data_u64;
        std::memcpy(&data_u64, buffer, 8); 

        return CANCommand{can_id, data_u64, 4};
    }
    struct alignas(uint64_t) Status_Message_1_FeedBack{
        uint8_t data[8];
        int32_t ERPM() const { 
            uint32_t raw = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]);
            return static_cast<int32_t>(raw);
        }
        float Toal_Current() const { 
            uint16_t raw = (data[4] << 8 | data[5]);
            return static_cast<float>(raw) / 10.0f;
        }
        float Duty_Cycle() const { 
            uint16_t raw = (data[6] << 8 | data[7]);
            return static_cast<float>(raw) / 1000.0f;
        }
    };
    struct alignas(uint64_t) Status_Message_4_FeedBack{
        uint8_t data[8];
        float FET_Temp() const { 
            uint16_t raw = (data[0] << 8 | data[1]);
            return static_cast<float>(raw) / 10.0f;
        }
        float Motor_Temp() const { 
            uint16_t raw = (data[2] << 8 | data[3]);
            return static_cast<float>(raw) / 10.0f;
        }
        float Toal_Current_In() const { 
            uint16_t raw = (data[4] << 8 | data[5]);
            return static_cast<float>(raw) / 10.0f;
        }
        float PID_Pos() const { 
            uint16_t raw = (data[6] << 8 | data[7]);
            return static_cast<float>(raw) / 50.0f;
        }
    };

    float rpm;
    float pos;
    float current;
    float temperature;
    int pole_pairs;
    float kt_rough;
    float max_vel;
    float max_tor;
    bool enable_multi_turn_angle;
    bool auto_set_zero_point;
    bool first_update_;
    float sign_reversed;
    int32_t total_turns_;
    float last_raw_pos_deg_;
    float encoder_zero_point_;
    bool feedback_seen_ = false;
    std::chrono::steady_clock::time_point last_feedback_time_;
    uint8_t motor_id;

    std::atomic<uint64_t> can_data_status_1_;        // CAN数据缓冲区1
    std::atomic<uint64_t> can_data_status_4_;        // CAN数据缓冲区4
};

}
