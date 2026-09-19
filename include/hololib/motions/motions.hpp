#pragma once

#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/obstacle_manager.hpp"
#include <cstdint>
#include <functional>
#include <vector>

namespace hololib {
struct MoveParams {
    bool angleCorrection = true;      /**< Whether to correct the angle during movement */
    float maxTranslationSpeed = 127.0f; /**< Maximum speed for translation */
    float maxRotationSpeed = 127.0f;    /**< Maximum speed for rotation */
    float minSpeed = 0.0f;              /**< Minimum speed (avoid stalling) */
    float exitRange = 0.5f;             /**< Acceptable position error to consider motion complete */
    float earlyExitRange = 0.0f;        /**< Distance to exit early before the movement is fully settled */
    uint32_t timeout = 5000;            /**< Maximum time allowed for movement (ms) */
}; 
struct MoveSettings {
    // Every default below MUST be qualified with `::`. In a default member
    // initializer, name lookup searches class scope first, so an unqualified
    // `chassis` resolves to MoveSettings::chassis -- the member being declared --
    // and binds the reference to itself rather than to the global. That is
    // undefined behaviour, it compiles without a warning, and it only shows up at
    // runtime as a data abort the first time a motion calls through it.
    pros::Motor& frontLeft = ::frontLeft;
    pros::Motor& frontRight = ::frontRight;
    pros::Motor& backLeft = ::backLeft;
    pros::Motor& backRight = ::backRight;

    Chassis& chassis = ::chassis;
    EncoderEKFOdometry& odom = ::odom;
    // One reference per line: in a comma-separated declaration each name needs
    // its own `::`, which is easy to miss.
    GainScheduler& xSched = ::xSched;
    GainScheduler& ySched = ::ySched;
    GainScheduler& thetaSched = ::thetaSched;
    ObstacleManager& obstacles = ::obstacles;

    std::function<hololib::Pose(bool)> poseGetter = ::poseGetter;
};

struct PathPoint {
    float x;
    float y;
    float theta;
};

enum class HeadingMode {
    FollowPath,
    HoldAngle,
    CustomAngles,
};

enum class SwingSide {
    Left,
    Right,
};

constexpr float DEG2RAD = M_PI / 180.0f;
constexpr float RAD2DEG = 180.0f / M_PI;

/**
 *@brief Moves the robot to a specific point using PID control.
 *@param tx The x-coordinate of the target point.
 *@param ty The y-coordinate of the target point.
 *@param params The movement parameters.
 *@param settings Shared motion dependencies.
 *@return void
 */
void moveToPoint(float tx, float ty, MoveParams params = {}, MoveSettings settings = {});

void followPath(const std::vector<PathPoint>& path, float lookaheadDistance, HeadingMode headingMode,
                float holdAngleDeg, bool reversed, MoveParams params = {}, MoveSettings settings = {});

void turnToHeading(float targetDeg, MoveParams params = {}, MoveSettings settings = {});

void turnToPoint(float tx, float ty, MoveParams params = {}, MoveSettings settings = {});

void moveRelative(float forward, float sideways, bool holdHeading = true, MoveParams params = {}, MoveSettings settings = {});

void moveDistance(float distance, bool holdHeading = true, MoveParams params = {}, MoveSettings settings = {});

void strafeDistance(float distance, bool holdHeading = true, MoveParams params = {}, MoveSettings settings = {});

void moveToPose(float tx, float ty, float targetThetaDeg, MoveParams params = {}, MoveSettings settings = {});

void swingTurn(float targetThetaDeg, SwingSide lockedSide, MoveParams params = {}, MoveSettings settings = {});

}; // namespace hololib