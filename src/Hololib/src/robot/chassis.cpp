#include "chassis.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kPi = 3.14159265358979323846f;

float clamp127(float value) { return std::clamp(value, -127.0f, 127.0f); }

int toMillivolts(float value) { return static_cast<int>(std::clamp(value, -127.0f, 127.0f) * (12000.0f / 127.0f)); }

float applyCurve(float input, const DriveCurve& curve) {
    if (std::abs(input) <= curve.deadzone) {
        return 0.0f;
    }

    const float sign = input >= 0.0f ? 1.0f : -1.0f;
    const float magnitude = std::abs(input);
    float output = magnitude;

    if (curve.curve_multipler != 1.0f) {
        const float normalized = magnitude / 127.0f;
        output = std::pow(normalized, curve.curve_multipler) * 127.0f;
    }

    if (output > 0.0f) {
        output = std::max(output, curve.minimum_output);
    }

    return sign * clamp127(output);
}
} // namespace

void motionHandlerTask(void* param);

MoveParams Chassis::defaultParams{};

PID::PID(double kP, double kI, double kD, double kF, double windupRange, bool signFlipReset, double slew)
    : m_gains{kP, kI, kD, kF, slew}, m_windupRange(windupRange), m_signFlipReset(signFlipReset) {}

PID::PID(const PIDGains& gains, double windupRange, bool signFlipReset)
    : m_gains(gains), m_windupRange(windupRange), m_signFlipReset(signFlipReset) {}

PIDGains PID::getGains() { return m_gains; }

void PID::setGains(PIDGains gains) { m_gains = gains; }

double PID::update(double error) { return update(error, error); }

double PID::update(double error, double measurement) {
    const uint32_t now = pros::millis();
    const double dt = m_initialized ? std::max((now - m_previousTime) / 1000.0, 0.001) : 0.02;

    if (!m_initialized) {
        m_previousError = error;
        m_previousMeasurement = measurement;
        m_previousTime = now;
        m_initialized = true;
    }

    if (m_signFlipReset && sgn(error) != sgn(m_previousError)) {
        m_integral = 0.0;
    }

    if (m_windupRange <= 0.0 || std::abs(error) <= m_windupRange) {
        m_integral += error * dt;
    }
    if (m_integralLimit > 0.0) {
        m_integral = std::clamp(m_integral, -m_integralLimit, m_integralLimit);
    }

    const double derivative = (measurement - m_previousMeasurement) / dt;
    m_filteredDerivative = m_alpha * derivative + (1.0 - m_alpha) * m_filteredDerivative;

    double output = (m_gains.kP * error) + (m_gains.kI * m_integral) - (m_gains.kD * m_filteredDerivative) +
                    (m_gains.kF * sgn(error));

    if (m_gains.slew > 0.0) {
        output = std::clamp(output, m_previousOutput - m_gains.slew, m_previousOutput + m_gains.slew);
    }

    m_previousError = error;
    m_previousMeasurement = measurement;
    m_previousTime = now;
    m_previousOutput = output;
    return output;
}

void PID::reset() {
    m_previousError = 0.0;
    m_previousMeasurement = 0.0;
    m_integral = 0.0;
    m_filteredDerivative = 0.0;
    m_previousTime = pros::millis();
    m_initialized = false;
    m_previousOutput = 0.0;
}

void PID::setSignFlipReset(bool signFlipReset) { m_signFlipReset = signFlipReset; }

bool PID::getSignFlipReset() { return m_signFlipReset; }

void PID::setWindupRange(double windupRange) { m_windupRange = windupRange; }

double PID::getWindupRange() { return m_windupRange; }

EncoderKalmanFilter::EncoderKalmanFilter(float process_noise, float measurement_noise) : R(measurement_noise) {
    x.setZero();
    P.setIdentity();
    Q = Eigen::Matrix2f::Identity() * process_noise;
    H << 1.0f, 0.0f;
}

float EncoderKalmanFilter::update(float measured_position, float) {
    x(0) = measured_position;
    return measured_position;
}

void GainScheduler::addStep(float threshold, float kP, float kI, float kD, float slew) {
    schedules.push_back({
        threshold, {kP, kI, kD, 0.0, slew}
    });
}

PIDGains GainScheduler::getGains(float error) const {
    if (schedules.empty()) {
        return {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    const float absError = std::abs(error);
    const ScheduledGain* best = &schedules.front();
    for (const auto& step : schedules) {
        if (absError <= step.threshold) {
            best = &step;
            break;
        }
        best = &step;
    }
    return best->gains;
}

void GainScheduler::clear() { schedules.clear(); }

MotionHandler::MotionHandler() = default;

void MotionHandler::enqueue(std::function<void()> motion, bool async) {
    lastEnqueuedId++;
    if (async) {
        mutex.take();
        queue.push(std::move(motion));
        mutex.give();
        if (!task) {
            task = new pros::Task(motionHandlerTask, this, "MotionHandler");
        }
        return;
    }
    running = true;
    currentRunningId = lastEnqueuedId;
    motion();
    running = false;
}

void MotionHandler::cancelAll() {
    mutex.take();
    while (!queue.empty()) {
        queue.pop();
    }
    running = false;
    mutex.give();
}

void MotionHandler::waitUntilDone() {
    while (running || !isQueueEmpty()) {
        pros::delay(10);
    }
}

void MotionHandler::cancelMotion() { running = false; }

bool MotionHandler::isInMotion() { return running; }

void MotionHandler::loop() {
    while (true) {
        std::function<void()> next;
        mutex.take();
        if (!queue.empty()) {
            next = queue.front();
            queue.pop();
            currentRunningId++;
            running = true;
        }
        mutex.give();

        if (next) {
            if (onMotionStartCallback) {
                onMotionStartCallback();
            }
            next();
            running = false;
        }
        pros::delay(10);
    }
}

void motionHandlerTask(void* param) { static_cast<MotionHandler*>(param)->loop(); }

void ObstacleManager::setRobotDimensions(float width, float length) {
    robot_width = width;
    robot_length = length;
}

void ObstacleManager::addObstacle(float x, float y, float radius) {
    obstacles.push_back({
        Eigen::Vector2f{x, y},
        radius
    });
}

void ObstacleManager::removeObstacle(size_t index) {
    if (index < obstacles.size()) {
        obstacles.erase(obstacles.begin() + index);
    }
}

void ObstacleManager::clearObstacles() { obstacles.clear(); }

const std::vector<Obstacle>& ObstacleManager::getObstacles() const { return obstacles; }

bool ObstacleManager::checkIntersection(
    const Eigen::Vector2f&, const Eigen::Vector2f&, float, Obstacle&, Eigen::Vector2f&) const {
    return false;
}

Eigen::Vector2f ObstacleManager::getAvoidanceTarget(
    const Eigen::Vector2f&, const Eigen::Vector2f& target_pos, float, float, float, int) const {
    return target_pos;
}

Eigen::Vector2f ObstacleManager::getPotentialFieldTarget(
    const Eigen::Vector2f&, const Eigen::Vector2f& target_pos, float, float, float, float) const {
    return target_pos;
}

Chassis::Chassis(
    pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br, pros::Imu imu_sensor, ChassisConfig chassisConfig)
    : frontLeft(fl), frontRight(fr), backLeft(bl), backRight(br), imu(imu_sensor), config(chassisConfig) {}

void Chassis::calibrate() {
    imu.reset(true);
    setPose(0.0f, 0.0f, 0.0f);
}

void Chassis::setXGains(std::vector<ScheduledGain> steps) {
    xSched.clear();
    for (const auto& step : steps) {
        xSched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD, step.gains.slew);
    }
}

void Chassis::setYGains(std::vector<ScheduledGain> steps) {
    ySched.clear();
    for (const auto& step : steps) {
        ySched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD, step.gains.slew);
    }
}

void Chassis::setThetaGains(std::vector<ScheduledGain> steps) {
    thetaSched.clear();
    for (const auto& step : steps) {
        thetaSched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD, step.gains.slew);
    }
}

void Chassis::setPose(float x, float y, float theta) {
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = theta;
    ekf.setPose(x, y, theta * kPi / 180.0f);
}

void Chassis::setPose(Pose pose) { setPose(pose.x, pose.y, pose.theta); }

Pose Chassis::getPose(bool radians) {
    currentPose.theta = static_cast<float>(imu.get_rotation());
    if (radians) {
        Pose pose = currentPose;
        pose.theta *= kPi / 180.0f;
        return pose;
    }
    return currentPose;
}

XDriveVoltages Chassis::calculateHolonomic(float vx, float vy, float vt) {
    return {
        clamp127(vy + vx + vt),
        clamp127(vy - vx - vt),
        clamp127(vy - vx + vt),
        clamp127(vy + vx - vt),
    };
}

void Chassis::setMotorVoltages(XDriveVoltages v) {
    frontLeft.move_voltage(toMillivolts(v.fl));
    frontRight.move_voltage(toMillivolts(v.fr));
    backLeft.move_voltage(toMillivolts(v.bl));
    backRight.move_voltage(toMillivolts(v.br));
}

void Chassis::brake() {
    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void Chassis::driveControl(float forward,
                           float sideways,
                           float rotation,
                           DriveCurves drivecurves,
                           bool fieldCentric,
                           float headingOffset,
                           DriveCorrection) {
    float y = applyCurve(forward, drivecurves.movement);
    float x = applyCurve(sideways, drivecurves.movement);
    float turn = applyCurve(rotation, drivecurves.rotation);

    if (fieldCentric) {
        const float heading = (static_cast<float>(imu.get_rotation()) - headingOffset) * kPi / 180.0f;
        const float cosH = std::cos(heading);
        const float sinH = std::sin(heading);
        const float rotatedX = x * cosH - y * sinH;
        const float rotatedY = x * sinH + y * cosH;
        x = rotatedX;
        y = rotatedY;
    }

    setMotorVoltages(calculateHolonomic(x, y, turn));
}

void Chassis::followPath(const std::vector<PathPoint>&, float, MoveParams, HeadingMode, float, bool) {}

void Chassis::turnToHeading(float targetDeg, MoveParams params) {
    const float maxSpeed = std::clamp(params.maxRotationSpeed, 0.0f, 127.0f);
    const float exitRange = params.exitRange > 0.0f ? params.exitRange : 1.5f;
    if (params.timeout == 0 || maxSpeed <= 0.0f) {
        brake();
        return;
    }

    const uint32_t start = pros::millis();
    float lastHeading = std::numeric_limits<float>::quiet_NaN();
    uint32_t lastProgressTime = start;
    while (pros::millis() - start < params.timeout) {
        const float heading = static_cast<float>(imu.get_rotation());
        if (!std::isfinite(heading)) {
            brake();
            return;
        }

        const float error = getAngleError(targetDeg, heading);
        if (std::abs(error) <= exitRange) {
            break;
        }
        if (!std::isfinite(lastHeading) || std::abs(getAngleError(heading, lastHeading)) > 0.5f) {
            lastHeading = heading;
            lastProgressTime = pros::millis();
        } else if (pros::millis() - lastProgressTime > 500 && std::abs(error) > 10.0f) {
            break;
        }

        float output = std::clamp(error * 0.8f, -maxSpeed, maxSpeed);
        if (params.minSpeed > 0.0f && std::abs(output) < params.minSpeed) {
            output = params.minSpeed * (output >= 0.0f ? 1.0f : -1.0f);
        }
        setMotorVoltages(calculateHolonomic(0.0f, 0.0f, output));
        pros::delay(10);
    }
    brake();
}

void Chassis::turnToPoint(float, float, MoveParams) {}
void Chassis::moveToPoint(float, float, MoveParams, bool) {}
void Chassis::moveRelative(float, float, MoveParams, bool) {}
void Chassis::moveDistance(float, MoveParams, bool) {}
void Chassis::strafeDistance(float, MoveParams, bool) {}
void Chassis::moveToPose(float, float, float, MoveParams) {}
void Chassis::curveCircle(float, float, MoveParams, CurveDirection) {}
void Chassis::waitUntilDone() { motion.waitUntilDone(); }
void Chassis::cancelAllMotions() { motion.cancelAll(); }
void Chassis::odometryTask() {}
float Chassis::getDistanceTraveled(bool) { return motionDistTraveled; }
void Chassis::cancelMotion() { motion.cancelMotion(); }
void Chassis::waitUntil(float) {}
void Chassis::setEKFGains(float xNoise, float yNoise, float thetaNoise, float measNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
    measurementNoise = measNoise;
    ekf.setProcessNoise(xNoise, yNoise, thetaNoise, measNoise);
}
void Chassis::setVelocityCalculations(bool state) { velocityCalculationsOn = state; }
bool Chassis::detectCollision() { return false; }
void Chassis::openLoop(float forward, float sideways, float rotation) {
    setMotorVoltages(calculateHolonomic(sideways, forward, rotation));
}
void Chassis::addObstacle(float x, float y, float radius) { obstacles.addObstacle(x, y, radius); }
void Chassis::removeObstacle(size_t index) { obstacles.removeObstacle(index); }
void Chassis::clearObstacles() { obstacles.clearObstacles(); }
void Chassis::setAvoidanceMode(AvoidanceMode mode) { avoidanceMode = mode; }
void Chassis::setAvoidanceParams(float safetyMargin, float clearance) {
    avoidanceSafetyMargin = safetyMargin;
    avoidanceClearance = clearance;
}
void Chassis::setPotentialFieldParams(float ka, float kr, float influenceRadius) {
    pf_ka = ka;
    pf_kr = kr;
    pf_influence_radius = influenceRadius;
}
void Chassis::setRobotDimensionsAvoidance(float width, float height) { obstacles.setRobotDimensions(width, height); }
float Chassis::radToDeg(float rad) { return rad * 180.0f / kPi; }
float Chassis::degToRad(float deg) { return deg * kPi / 180.0f; }
void Chassis::setEKFstate(bool state) { config.kfEnabled = state; }
void Chassis::addTrackingWheel(TrackingWheelConfig config) { trackingWheelConfigs.push_back(config); }
void Chassis::clearTrackingWheels() { trackingWheelConfigs.clear(); }
void Chassis::setMoveParams(MoveParams params) { defaultParams = params; }
void Chassis::swingTurn(float, SwingSide, MoveParams) {}
void Chassis::getControllerInput(pros::Controller) {}
void Chassis::logReplayData(pros::Controller, int) {}
void Chassis::disableReplayDataLogging() {}
void Chassis::runDriverReplay(std::vector<PathPoint>, float) {}
