#pragma once

#include <cmath>
#include <array>
#include <limits>
#include <algorithm>

namespace rmcs_core::controller::pid{

class FuzzyPID{
public:
    enum FuzzyLevel{
        NB = 0, // 负大
        NM = 1, // 负中
        NS = 2, // 负小
        ZO = 3, // 零
        PS = 4, // 正小
        PM = 5, // 正中
        PB = 6  // 正大
    };
    FuzzyPID(double kp_base, double ki_base, double kd_base,
             double ke, double kec,
             double kup, double kui, double kud)
            : kp_base_(kp_base)
            , ki_base_(ki_base)
            , kd_base_(kd_base)
            , ke_(ke)
            , kec_(kec)
            , kup_(kup)
            , kui_(kui)
            , kud_(kud) 
    {
        reset();
    }
    virtual ~FuzzyPID() = default;

    FuzzyPID& reset() {
        last_err_     = nan;
        err_integral_ = 0;

        return *this;
    }
    FuzzyPID& set_base_pid(double kp, double ki, double kd) {
        kp_base_ = kp;
        ki_base_ = ki;
        kd_base_ = kd;
        return *this;
    }
    FuzzyPID& set_fuzzy_params(double ke, double kec, double kup, double kui, double kud) {
        ke_ = ke;
        kec_ = kec;
        kup_ = kup;
        kui_ = kui;
        kud_ = kud;
        return *this;
    }
    FuzzyPID& set_limits(double integral_min, double integral_max, double output_min, double output_max, double dead_zone) {
        integral_min_ = integral_min;
        integral_max_ = integral_max;
        output_min_ = output_min;
        output_max_ = output_max;
        dead_zone_ = dead_zone;
        return *this;
    }
    double update(double error){
        if(!std::isfinite(error)){
            return 0.0;
        }

        double error_change = std::isnan(last_err_) ? 0 : error - last_err_;
        last_err_ = error;

        auto mu_e = fuzzyfy(ke_ * error); // 误差模糊化
        auto mu_ec = fuzzyfy(kec_ * error_change); // 误差变化率模糊化

        double dKp = defuzzy(mu_e, mu_ec, ruleKp) * kup_; // 模糊推理得到的 Kp 调节量
        double dKi = defuzzy(mu_e, mu_ec, ruleKi) * kui_;
        double dKd = defuzzy(mu_e, mu_ec, ruleKd) * kud_;

        double kp = kp_base_ + dKp;
        double ki = ki_base_ + dKi;
        double kd = kd_base_ + dKd;

        kp = std::max(0.0, kp); // Kp 不允许为负
        ki = std::max(0.0, ki); // Ki 不允许为负
        kd = std::max(0.0, kd); // Kd 不允许为负

        if(error > integral_split_min_ && error < integral_split_max_){
            err_integral_ = std::clamp(err_integral_ + error, integral_min_, integral_max_);
        }
        else{
            err_integral_ = 0;
        }
        double control = kp * error + ki * err_integral_ + kd * error_change;
        if (std::abs(control) < dead_zone_) {
            control = 0.0; // 死区处理
        }
        return std::clamp(control, output_min_, output_max_);
    }

    double kp_base_,ki_base_,kd_base_; // 基础PID值
    double ke_,kec_;           // 输入量化因子(把真实误差和误差变化率锁放到模糊论域，再做模糊化)
    double kup_,kui_,kud_;     // 输出比例因子(调节幅度，决定实际调多少)
    double integral_min_ = -inf, integral_max_ = inf;   // 积分限幅，防止积分过大导致系统震荡
    double integral_split_min_ = -inf, integral_split_max_ = inf;// 积分分离阈值，误差过大时不积分
    double output_min_ = -inf, output_max_ = inf;
    double dead_zone_ = 0.0;
    
private:
    static constexpr double inf = std::numeric_limits<double>::infinity(); // 无穷大
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();// 非数值

    double last_err_ = nan;
    double err_integral_ = 0;

    const std::array<double, 7> centers = {-3, -2, -1, 0, 1, 2, 3}; // 模糊集中心
    using RuleTable = std::array<std::array<int, 7>, 7>; // 模糊规则表

    RuleTable ruleKp = {{
        {{PB, PB, PM, PM, PS, ZO, ZO}},
        {{PB, PB, PM, PS, PS, ZO, NS}},
        {{PM, PM, PM, PS, ZO, NS, NS}},
        {{PM, PM, PS, ZO, NS, NM, NM}},
        {{PS, PS, ZO, NS, NS, NM, NM}},
        {{PS, ZO, NS, NM, NM, NM, NB}},
        {{ZO, ZO, NM, NM, NM, NB, NB}}
    }};

    RuleTable ruleKi = {{
        {{NB, NB, NM, NM, NS, ZO, ZO}},
        {{NB, NB, NM, NS, NS, ZO, ZO}},
        {{NM, NM, NS, NS, ZO, PS, PS}},
        {{NM, NS, NS, ZO, PS, PS, PM}},
        {{NS, NS, ZO, PS, PS, PM, PM}},
        {{ZO, ZO, PS, PS, PM, PB, PB}},
        {{ZO, ZO, PS, PM, PM, PB, PB}}
    }};

    RuleTable ruleKd = {{
        {{PS, NS, NB, NB, NB, NM, PS}},
        {{PS, NS, NB, NM, NM, NS, ZO}},
        {{ZO, NS, NM, NM, NS, NS, ZO}},
        {{ZO, NS, NS, NS, NS, NS, ZO}},
        {{ZO, ZO, ZO, ZO, ZO, ZO, ZO}},
        {{PB, NS, PS, PS, PS, PS, PB}},
        {{PB, PM, PM, PM, PS, PS, PB}}
    }};

    /*
    @brief 模糊化函数：将输入误差或误差变化率转换成 7 个模糊等级的隶属度
    @param x 输入值（误差或误差变化率）
    @return 7 个模糊等级的隶属度数组，值域为 [0, 1]，表示输入属于每个模糊等级的程度，越接近 1 表示越属于该等级
    */
    std::array<double, 7> fuzzyfy(double x){
        x = std::clamp(x, -3.0, 3.0);

        std::array<double, 7> mu{};

        for(int i = 0; i < 7; ++i){
            double distance = std::abs(x - centers[i]);
            mu[i] = std::max(0.0, 1.0 - distance); // 三角隶属函数
        }
        return mu;
    }
    /*
    @brief 解模糊函数：根据输入误差和误差变化率的隶属度，以及模糊规则表，计算实际的 PID 调节量
    @param mu_e 输入误差的隶属度数组
    @param mu_ec 输入误差变化率的隶属度数组
    @param rule_table 模糊规则表，定义了每个输入组合对应的输出模糊等级
    @return 实际的 PID 调节量，通过重心法计算得到，值域根据输出模糊等级的中心值和比例因子决定
    */
    double defuzzy(
        const std::array<double, 7>& mu_e,
        const std::array<double, 7>& mu_ec,
        const RuleTable& rule_table)
    {
        double numerator = 0.0;     // 分子
        double denominator = 0.0;   // 分母
        for(int i = 0; i < 7; ++i){
            for(int j = 0; j < 7; ++j){
                double weight = std::min(mu_e[i], mu_ec[j]); // 规则权重，取输入隶属度的最小值
                int outputLevel = rule_table[i][j]; // 模糊规则表查询得到输出模糊集

                numerator += weight * centers[outputLevel]; // 加权求和
                denominator += weight; // 权重总和
            }
        }
        return denominator != 0 ? numerator / denominator : 0; // 重心法解模糊
    }

};

}