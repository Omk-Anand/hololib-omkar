#include "Subsystems/modular_lift.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

ModularLift::ModularLift(const std::vector<LiftMotorConfig>& m_configs, LiftMechanism t, LiftConfig c)
    : type(t), config(c), is_running(false), cancel_request(false), is_settled(false), task(nullptr) {
    for (const auto& m_config : m_configs) {
        motors.emplace_back(m_config.port, m_config.gearset);
        motors.back().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

ModularLift::~ModularLift() {
    cancel();

    const uint32_t start_wait = pros::millis();
    while (is_running && (pros::millis() - start_wait < 500)) {
        pros::delay(5);
    }

    if (task != nullptr) {
        task->remove();
        delete task;
        task = nullptr;
    }
}

float ModularLift::getLiftRadians(float motor_degrees) {
    return (motor_degrees / 360.0f) * config.gear_ratio * (2.0f * kPi);
}

void ModularLift::setPayload(float mass_kg) { config.payload_mass_kg = mass_kg; }

float ModularLift::calculateFeedforward(float current_motor_degrees) {
    const float total_mass = config.arm_mass_kg + config.payload_mass_kg;

    switch (type) {
    case LiftMechanism::CASCADE:
        return config.kG_base * total_mass * 9.81f;
    case LiftMechanism::FOUR_BAR:
    case LiftMechanism::SIX_BAR:
    case LiftMechanism::VIRTUAL: {
        const float current_angle_rad = getLiftRadians(current_motor_degrees);
        return config.kG_base * total_mass * 9.81f * config.arm_length * std::cos(current_angle_rad);
    }
    default:
        return 0.0f;
    }
}

void ModularLift::moveTo(float target_motor_degrees) {
    target_mutex.take(TIMEOUT_MAX);
    target_position = target_motor_degrees;
    target_mutex.give();

    if (is_running) {
        return;
    }

    is_running = true;
    cancel_request = false;
    is_settled = false;

    if (task != nullptr) {
        task->remove();
        delete task;
        task = nullptr;
    }

    TaskParams* params = new TaskParams{this};
    task = new pros::Task(task_trampoline, params, "ModularLiftTask");
}

void ModularLift::task_trampoline(void* params) {
    TaskParams* p = static_cast<TaskParams*>(params);
    if (p && p->instance) {
        p->instance->controlLoopImpl();
    }
    delete p;
}

void ModularLift::controlLoopImpl() {
    uint32_t current_time = pros::millis();
    constexpr uint32_t kLoopDelayMs = 20;

    while (!cancel_request) {
        target_mutex.take(TIMEOUT_MAX);
        const float current_target = target_position;
        target_mutex.give();

        float total_pos = 0.0f;
        float total_vel = 0.0f;
        for (auto& motor : motors) {
            total_pos += static_cast<float>(motor.get_position());
            total_vel += static_cast<float>(motor.get_actual_velocity()) * 6.0f;
        }

        const float pos = motors.empty() ? 0.0f : total_pos / motors.size();
        const float vel_deg_per_sec = motors.empty() ? 0.0f : total_vel / motors.size();

        Eigen::Vector2f x(pos, vel_deg_per_sec);
        Eigen::Vector2f x_ref(current_target, 0.0f);
        const Eigen::Matrix<float, 1, 1> u_feedback = -config.K * (x - x_ref);

        float total_voltage = u_feedback(0, 0) + calculateFeedforward(pos);
        total_voltage = std::clamp(total_voltage, -12000.0f, 12000.0f);

        for (auto& motor : motors) {
            motor.move_voltage(static_cast<int>(total_voltage));
        }

        is_settled = std::abs(pos - current_target) < config.tolerance && std::abs(vel_deg_per_sec) < 10.0f;
        pros::Task::delay_until(&current_time, kLoopDelayMs);
    }

    is_running = false;
    is_settled = false;
}

void ModularLift::cancel() {
    cancel_request = true;
    for (auto& motor : motors) {
        motor.brake();
    }
}

void ModularLift::waitUntilDone() {
    while (is_running && !is_settled) {
        pros::delay(10);
    }
}

bool ModularLift::isRunning() { return is_running; }
