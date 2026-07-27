#pragma once

#include <cstdint>
#include <atomic>
#include <cstring>
#include <algorithm>
#include <bit>

/*
@breif RSO1返回的和设置的参数均以经过减速比转换后的输出端参数
*/
namespace librmcs::device {
class Rs01
{
public:
enum class ExtendedMode : uint8_t {
    operational_control = 0x1,  // 运控模式
    motor_enable = 0x3,         // 电机使能模式
    motor_stop = 0x4,           // 电机停止模式, 数据段数据为0 byte[0]=1时：清除故障
    set_motor_zero = 0x6,       // 设置电机零位模式
    set_motor_canid = 0x7,      // 设置电机CAN ID模式
    single_param_write = 0x12,  // 单参数写入模式(掉电丢失)
    error_feedback = 0x15,      // 错误反馈模式
    save_motor_param = 0x16,    // 保存电机参数模式
    param_return = 0x18,        // 是否开启参数回报，电机回报的也是0x18
};
// 可读写参数列表(与单参数写入模式对应，不全)
enum ParamList : uint16_t{
    run_mode = 0x7005,  // 0: 运控模式 1: 位置模式 （PP） 2: 速度模式 3: 电流模式 5：位置模式（CSP）
    loc_ref = 0x7016,   // 位置模式角度指令 rad
    vel_max = 0x7024,   // 位置模式（PP）速度 rad/s
    acc_set = 0x7025,   // 位置模式（PP）加速度 rad/s^2
    dcc_set = 0x702E,   // 电机PP模式减速度(常态不启用) rad/s^2
};
struct Config{
    explicit Config()
    :reversed(false)
    ,multi_turn_angle_enabled(false){}

    bool reversed;
    bool multi_turn_angle_enabled;

    Config& set_reversed(){ return reversed = true, *this; }
    Config& enable_multi_turn_angle(){ return multi_turn_angle_enabled = true, *this; }
};
struct ExtendedCanId{
    uint32_t can_id : 8;//CAN ID
    uint32_t data : 16;//CAN 数据
    uint32_t mode : 5;//通信类型
};

    Rs01()
    :pos_(0.0)
    ,vel_(0.0)
    ,tor_(0.0)
    ,temp_(0.0)
    ,last_raw_pos_(0.0)
    ,multi_turn_pos_(0.0)
    ,reversed_(false)
    ,multi_turn_angle_enabled_(false)
    ,feedback_seen_(false)
    ,first_update_(true){}

    explicit Rs01(const Config& config)
    :pos_(0.0)
    ,vel_(0.0)
    ,tor_(0.0)
    ,temp_(0.0)
    ,last_raw_pos_(0.0)
    ,multi_turn_pos_(0.0)
    ,reversed_(false)
    ,multi_turn_angle_enabled_(false)
    ,feedback_seen_(false)
    ,first_update_(true){
        configure(config);
    }

    void configure(const Config& config){
        reversed_ = config.reversed;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        first_update_ = true;
        last_raw_pos_ = 0.0f;
        multi_turn_pos_ = 0.0f;
        feedback_seen_ = false;
    }

    //禁用拷贝
    Rs01(const Rs01&) = delete;
    Rs01& operator=(const Rs01&) = delete;

    void store_status(uint64_t can_data){
        can_data_.store(can_data,std::memory_order_release);
        feedback_seen_.store(true, std::memory_order_release);
    }

    void update_status(){
        if(!feedback_seen_.load(std::memory_order_acquire))
            return;
        auto feedback = std::bit_cast<Rs01Feedback>(can_data_.load(std::memory_order_acquire));
        float reversed_sign = reversed_ ? -1.0 : 1.0;
        const float raw_pos = uint_to_float(feedback.Pos(), P_MIN, P_MAX, 16);
        if(first_update_){
            last_raw_pos_ = raw_pos;
            multi_turn_pos_ = raw_pos;
            first_update_ = false;
        }

        if(multi_turn_angle_enabled_){
            multi_turn_pos_ += wrap_delta(raw_pos - last_raw_pos_);
            pos_ = multi_turn_pos_ * reversed_sign;
        }
        else{
            pos_ = raw_pos * reversed_sign;
        }
        last_raw_pos_ = raw_pos;

        vel_ = uint_to_float(feedback.Vel(), V_MIN, V_MAX, 16) * reversed_sign;
        tor_ = uint_to_float(feedback.Tor(), T_MIN, T_MAX, 16) * reversed_sign;
        temp_ = uint_to_float(feedback.Temp(), 0, 100, 16);
    }

    uint32_t generate_extended_can_id(ExtendedMode mode,uint8_t target_id,float data) const{
        ExtendedCanId extended_can_id;
        float reversed_sign = reversed_ ? -1.0 : 1.0;
        switch(mode){
        case ExtendedMode::operational_control:
            extended_can_id.mode = 0x1;
            extended_can_id.data = float_to_uint(data* reversed_sign, T_MIN, T_MAX, 16);
            extended_can_id.can_id = target_id;
            return std::bit_cast<uint32_t>(extended_can_id) & 0x1FFFFFFF;
        break;
        case ExtendedMode::motor_enable:
            extended_can_id.mode = 0x3;
        break;
        case ExtendedMode::motor_stop:
            extended_can_id.mode = 0x4;
        break;
        case ExtendedMode::set_motor_zero:
            extended_can_id.mode = 0x6;
        break;
        case ExtendedMode::set_motor_canid:
            extended_can_id.mode = 0x7;
        break;
        case ExtendedMode::single_param_write:
            extended_can_id.mode = 0x12;
        break;
        case ExtendedMode::error_feedback:
            extended_can_id.mode = 0x15;
        break;
        case ExtendedMode::save_motor_param:
            extended_can_id.mode = 0x16;   
        break;
        case ExtendedMode::param_return:
            extended_can_id.mode = 0x18;
        break;
        }

        extended_can_id.data = static_cast<uint16_t>(data);
        extended_can_id.can_id = target_id;
        return std::bit_cast<uint32_t>(extended_can_id) & 0x1FFFFFFF;
   }
    // 使能电机
    uint64_t enable_motor(){
        uint64_t result_can_data = 0;
        return result_can_data;
    }
    // 设置零位
    uint64_t set_zero_pos(){
        uint64_t result_can_data = 1;
        return result_can_data;
    }
    // 保存电机参数
    uint64_t save_motor_param(){
        uint8_t data[8];
        uint64_t result_can_data;
        data[0] = 1;
        data[1] = 2;
        data[2] = 3;
        data[3] = 4;
        data[4] = 5;
        data[5] = 6;
        data[6] = 7;
        data[7] = 8;
        memcpy(&result_can_data, data, 8);
        return result_can_data;
    }
    // 是否开启参数上报
    uint64_t param_return(bool f_cmd){
        uint8_t data[8];
        uint64_t result_can_data;
        data[0] = 1;
        data[1] = 2;
        data[2] = 3;
        data[3] = 4;
        data[4] = 5;
        data[5] = 6;
        data[6] = f_cmd ? 1 : 0;
        data[7] = 0;
        memcpy(&result_can_data, data, 8);
        return result_can_data;
    }
    // 生成单参数写入命令，index为参数索引，Byte4起写入参数值，小端，与官方例程保持一致。
    uint64_t single_param_write(ParamList index, uint8_t value){
        uint8_t data[8]{};
        uint64_t result_can_data;
        memcpy(&data[0], &index, sizeof(index));
        memcpy(&data[4], &value, sizeof(value));
        memcpy(&result_can_data, data, 8);
        return result_can_data;
   }
    uint64_t single_param_write(ParamList index, float value){
        uint8_t data[8]{};
        uint64_t result_can_data;
        if(index == ParamList::loc_ref)
            value *= reversed_ ? -1.0f : 1.0f;
        memcpy(&data[0], &index, sizeof(index));
        memcpy(&data[4], &value, sizeof(value));
        memcpy(&result_can_data, data, 8);
        return result_can_data;
   }
   // 生成运控命令
    uint64_t operational_control(float target_pos,float target_vel,float target_kp,float target_kd){
        float reversed_sign = reversed_ ? -1.0 : 1.0;
        uint8_t data[8];
        uint64_t result_can_data;

        target_pos = std::clamp(target_pos, P_MIN, P_MAX);
        target_vel = std::clamp(target_vel, V_MIN, V_MAX);
        target_kp = std::clamp(target_kp, Kp_MIN, Kp_MAX);
        target_kd = std::clamp(target_kd, Kd_MIN, Kd_MAX);

        target_pos *= reversed_sign;
        target_vel *= reversed_sign;

        uint16_t target_pos_uint = float_to_uint(target_pos, P_MIN, P_MAX, 16);
        uint16_t target_vel_uint = float_to_uint(target_vel, V_MIN, V_MAX, 16);
        uint16_t target_kp_uint = float_to_uint(target_kp, Kp_MIN, Kp_MAX, 16);
        uint16_t target_kd_uint = float_to_uint(target_kd, Kd_MIN, Kd_MAX, 16);

        data[0] = (target_pos_uint >> 8) & 0xff;
        data[1] = target_pos_uint & 0xff;
        data[2] = (target_vel_uint >> 8) & 0xff;
        data[3] = target_vel_uint & 0xff;
        data[4] = (target_kp_uint >> 8) & 0xff;
        data[5] = target_kp_uint & 0xff;
        data[6] = (target_kd_uint >> 8) & 0xff;
        data[7] = target_kd_uint & 0xff;

        memcpy(&result_can_data, data, 8);
        return result_can_data;
   }

   void set_reversed(bool reversed){
    reversed_ = reversed;
   }
   //使能电机 8byte数据区不做要求
   static uint32_t enable_motor_extend_canid(uint8_t target_id){
    ExtendedCanId extended_can_id;
    extended_can_id.mode = static_cast<uint8_t>(ExtendedMode::motor_enable);
    extended_can_id.data = 0x127;
    extended_can_id.can_id = target_id;
    return std::bit_cast<uint32_t>(extended_can_id) & 0x1FFFFFFF;
   }
   //停止电机 8byte数据区要为0,byte[0]=1时：清除故障
   static uint32_t stop_motor_extend_canid(uint8_t target_id){
    ExtendedCanId extended_can_id;
    extended_can_id.mode = static_cast<uint8_t>(ExtendedMode::motor_stop);
    extended_can_id.data = 0x127;
    extended_can_id.can_id = target_id;
    return std::bit_cast<uint32_t>(extended_can_id) & 0x1FFFFFFF;
   }
   //设置电机零位 8byte数据区 byte[0] = 1;
   static uint32_t set_motor_zero_extend_canid(uint8_t target_id){
    ExtendedCanId extended_can_id;
    extended_can_id.mode = static_cast<uint8_t>(ExtendedMode::set_motor_zero);
    extended_can_id.data = 0x127;
    extended_can_id.can_id = target_id;
    return std::bit_cast<uint32_t>(extended_can_id) & 0x1FFFFFFF;
   }

   float get_pos() const{return pos_;}
   float get_vel() const{return vel_;}
   float get_tor() const{return tor_;}
   float get_temp() const{return temp_;}
   float max_torque() const{return T_MAX;}
private:

static float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
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
static float wrap_delta(float delta){
    if(delta > P_RANGE * 0.5f)
        delta -= P_RANGE;
    else if(delta <= -P_RANGE * 0.5f)
        delta += P_RANGE;
    return delta;
}
struct alignas(uint64_t) Rs01Feedback{
    uint8_t data[8];
    uint16_t Pos() const{return (data[0] << 8 | (data[1] ));}
    uint16_t Vel() const{return (data[2] << 8 | (data[3] ));}
    uint16_t Tor() const{return (data[4] << 8 | (data[5] ));}
    uint16_t Temp() const{return (data[6] << 8| (data[7] ));}
};
    std::atomic<uint64_t> can_data_{0};
    static constexpr float P_MIN = -12.57;
    static constexpr float P_MAX = 12.57;
    static constexpr float P_RANGE = P_MAX - P_MIN;
    static constexpr float V_MIN = -44.0;
    static constexpr float V_MAX = 44.0;
    static constexpr float T_MIN = -17.0;
    static constexpr float T_MAX = 17.0;
    static constexpr float Kp_MAX = 500.0;
    static constexpr float Kd_MAX = 5.0;
    static constexpr float Kp_MIN = 0.0;
    static constexpr float Kd_MIN = 0.0;

    // 插补位置模式的极值限制 随便改
    static constexpr float Value_MIN = -44;
    static constexpr float Value_MAX = 44;

    float pos_;
    float vel_;
    float tor_;
    float temp_;
    float last_raw_pos_;
    float multi_turn_pos_;
    bool reversed_;
    bool multi_turn_angle_enabled_;
    std::atomic_bool feedback_seen_;
    bool first_update_;
};
}
