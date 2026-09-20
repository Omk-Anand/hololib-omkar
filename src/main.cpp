#include "main.h"
#include "Eigen/Core" // IWYU pragma: export
#include "chassis.h"
#define LIFTLIB_NO_GLOBAL_NAMES
#include "liftlib/liftlib.hpp"
// #include "distanceReset.h"
#include "pros/imu.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

constexpr float kPi = 3.14159265358979323846f;

// Initialize motor ports (negative for reversing motor)
int frontLPort = 11;
int frontRPort = -12;
int backLPort = 20;
int backRPort = -17;

// Initialize IMU port
int imuPort = 18;

Eigen::Matrix3f cameraMatrix = (Eigen::Matrix3f() << 383.57019326565296f * 0.5f,
                                0.0f,
                                322.50382974986405f * 0.5f,
                                0.0f,
                                385.93627187500545f * 0.5f,
                                230.00229561364546f * 0.5f,
                                0.0f,
                                0.0f,
                                1.0f)
                                   .finished();

Eigen::Vector<float, 5> distCoeffs = (Eigen::Vector<float, 5>() << -0.047271311979687029f,
                                      0.19589273312693006f,
                                      -0.0052232338824990303f,
                                      0.00058677141664816013f,
                                      -0.14778998923627218f)
                                         .finished();

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(frontLPort, pros::MotorGear::blue);
pros::Motor frontr(frontRPort, pros::MotorGear::blue);
pros::Motor backl(backLPort, pros::MotorGear::blue);
pros::Motor backr(backRPort, pros::MotorGear::blue);
pros::Imu imu(imuPort);

pros::Distance front = pros::Distance(4);
pros::Distance back = pros::Distance(3);
pros::Distance left = pros::Distance(5);
pros::Distance right = pros::Distance(2);
// pros::AIVision visionSensor = pros::AIVision(visionPort);

pros::Motor clawGripper = pros::Motor(10, pros::MotorGear::green); // open/close rollers

pros::Motor intake = pros::Motor(8, pros::MotorGear::blue);

pros::Motor clawRotationMotor(9, pros::MotorGear::green);

liftlib::PID clawRotationPID(/*kP=*/5.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/2.0f);
liftlib::Subsystem clawRotationLift(
    {
        liftlib::MotorConfig{.port = -9,
                             .gear_ratio = 12.0f / 60.0f,
                             .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                             .gearset = pros::MotorGears::green}
},
    clawRotationPID);

constexpr float LIFT_SPOOL_RADIUS_IN = 0.26f; // measured
constexpr float LIFT_GEAR_RATIO = 1.0f;       // direct drive, no external reduction
constexpr float LIFT_INCHES_PER_MOTOR_DEGREE = LIFT_GEAR_RATIO * (kPi / 180.0f) * LIFT_SPOOL_RADIUS_IN;
liftlib::Subsystem liftLift(
    {
        liftlib::MotorConfig{.port = 6,
                             .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                             .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                             .gearset = pros::MotorGears::blue},
        liftlib::MotorConfig{.port = -7,
                             .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                             .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                             .gearset = pros::MotorGears::blue}
},
    std::vector<liftlib::Subsystem::GainPoint>{
        {liftlib::PID(/*kP=*/28.0f, /*kI=*/0.0f, /*kD=*/1.0f, /*threshold=*/1.0f), /*position_in=*/12.0f},
        {liftlib::PID(/*kP=*/20.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/24.0f},
        {liftlib::PID(/*kP=*/0.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/36.0f},
    });
// Initialize Chassis
Chassis chassis(frontl,
                frontr,
                backl,
                backr,
                imu,
                {.drivetrainWidth = 9.1,    // width from wheel to wheel
                 .drivetrainLength = 10.25, // length from wheel to wheel
                 .wheelDiameter = 3.25,     // wheel diameter in inches (should
                                            // be tuned to EFFECTIVE wheel
                                            // diameter)
                 .gearRatio = 0.5,          // gear ratio of the drivetrain
                 .kfEnabled = false});      // Enables ekf, only use if you know
                                            // how to tune the process and
                                            // measurement noise.

void initialize() {
    pros::lcd::initialize();

    // Calibrate the chassis
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    // Set PID gains for chassis
    chassis.setXGains({
        {36.0, {15, 0, 2.4}},
        {0.0,  {25, 0, 0.5}},
    });
    chassis.setYGains({
        {36.0, {15, 0, 1.6}},
        {0.0,  {20, 0, 1.5}},
    });
    chassis.setThetaGains({
        {180.0, {0, 0, 0}   },
        {90.0,  {0, 0, 0}   },
        {45.0,  {3, 0, 0}   },
        {0,     {3, 0, 0.04}},
    });

    // Basically allows you to see the velocity of the chassis (in/s) (helpful for
    // making custom motions)
    chassis.setVelocityCalculations(true);

    // LCD screen task to display chassis data
    pros::Task screen_task([&]() {
        while (true) {
            Pose pose = chassis.getPose(false); // false means degrees, true means radians
            pros::lcd::print(0, "X: %.3f", pose.x);
            pros::lcd::print(1, "Y: %.3f", pose.y);
            pros::lcd::print(2, "Theta: %.3f", pose.theta);
            pros::lcd::print(3, "X Velocity: %.3f", pose.velocity.vx);
            pros::lcd::print(4, "Y Velocity: %.3f", pose.velocity.vy);
            pros::lcd::print(5, "Theta Velocity: %.3f", pose.velocity.w);
            pros::delay(50);
        }
    });
}

void disabled() { chassis.setPose(0, 0, 0); }

void competition_initialize() {}

/*
Run:
python tools/sim_auton.py
then open the file with the browser of your choice.
*/
void simulation() {}

void autonomous() {
    chassis.setPose(0, 0, 0);

    intake.move_voltage(-12000);
    pros::delay(600);
    intake.brake();
    chassis.moveToPoint(0, 5, {});
    

}

void testFunction() { std::cout << "Function called" << std::endl; }

void opcontrol() {
    chassis.setPose(0, 0, 00);
    chassis.setEKFstate(false); // turn off EKF for driver control

    // Example drive curves
    DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
    DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
    int prev_forward = 0;
    int prev_sideways = 0;
    int prev_rotation = 0;

    bool reverseFront = false;
    bool prevUp = false;

    constexpr float LIFT_JOG_POWER = 127.0f; // liftlib full scale, == 12000 mV
    bool liftJogging = false;

    constexpr float CLAW_PERPENDICULAR_DEG = 0.0f;
    constexpr float CLAW_PARALLEL_DEG = 90.0f;

    constexpr float CLAW_FIRST_L1_STEP_DEG = 50.0f;
    constexpr float CLAW_MATCHLOAD_DEG = 100.0f;

    constexpr float CLAW_HOMING_POWER = 40.0f; // -127..127, gentle on purpose
    constexpr std::int32_t CLAW_HOMING_CURRENT_MA = 1200;
    constexpr std::uint32_t CLAW_HOMING_GRACE_MS = 250;
    constexpr std::uint32_t CLAW_HOMING_TIMEOUT_MS = 3000;
    constexpr float CLAW_STALL_EPSILON_DEG = 0.3f;
    constexpr int CLAW_STALL_TICKS = 10; // 10 * 20ms loop = 200ms of no movement

    // Set once L2 has homed and tared, which is what makes the absolute angles
    // (matchload) mean anything.
    bool clawZeroed = false;

    constexpr float CLAW_MANUAL_POWER = 127.0f * 0.6f; // 60%
    bool clawManualJogging = false;
    bool clawManualTared = false; // one tare per press, not once per tick

    // 0 = not homing, -1 = homing down (L2), +1 = homing up (L1)
    int clawHomingDir = 0;
    std::uint32_t clawHomingStartedAt = 0;
    int clawStallTicks = 0;
    float clawLastPosition = 0.0f;

    constexpr std::uint32_t CLAW_CHORD_WINDOW_MS = 100;

    bool prevL1 = false;
    bool prevL2 = false;
    bool l1Pending = false;
    bool l2Pending = false;
    std::uint32_t l1PressedAt = 0;
    std::uint32_t l2PressedAt = 0;

    clawRotationLift.holdActively();
    chassis.logReplayData(master, 100); // allows logging for driver replay

    while (true) {
        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int sideways = controller.get_analog(ANALOG_LEFT_X);
        int rotation = controller.get_analog(ANALOG_RIGHT_X);
        if (prev_forward != forward || prev_sideways != sideways || prev_rotation != rotation) {
            prev_forward = forward;
            prev_sideways = sideways;
            prev_rotation = rotation;
        }

        clawGripper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        const bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (r1 || r2) {
            liftLift.setOutput(r1 ? LIFT_JOG_POWER : -LIFT_JOG_POWER);
            liftJogging = true;
        } else if (liftJogging) {
            liftLift.holdActively();
            liftJogging = false;
        }

        const bool intakeIn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        const bool intakeOut = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        const bool outtakeOnly = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

        // Gripper: B and Y only. LEFT is deliberately absent here.
        if (intakeIn) {
            clawGripper.move_voltage(12000);
        } else if (intakeOut) {
            clawGripper.move_voltage(-12000);
        } else {
            clawGripper.move_voltage(0);
        }

        // Intake: same two buttons, plus LEFT for the outtake direction alone.
        if (intakeIn) {
            intake.move_voltage(-12000);
        } else if (intakeOut || outtakeOnly) {
            intake.move_voltage(12000);
        } else {
            intake.move_voltage(0);
        }

        // Claw pivot. Only rising edges count, so holding a button does not re-issue
        // the move -- one press, one command.
        const bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        const std::uint32_t nowMs = pros::millis();

        if (l1 && !prevL1) {
            l1Pending = true;
            l1PressedAt = nowMs;
        }
        if (l2 && !prevL2) {
            l2Pending = true;
            l2PressedAt = nowMs;
        }
        prevL1 = l1;
        prevL2 = l2;

        if (l1Pending && l2Pending) {
            // Matchload is an absolute angle, so it only means anything once L2 has
            // homed and established the zero.
            clawRotationLift.stop();
            clawHomingDir = 0;
            clawRotationLift.moveTo(CLAW_MATCHLOAD_DEG);
            l1Pending = false;
            l2Pending = false;
        } else if (l1Pending && nowMs - l1PressedAt >= CLAW_CHORD_WINDOW_MS) {
            // L1 aims at an angle -- it does not home. Once L2 has zeroed the claw
            // that angle is absolute; before then there is no frame to measure from,
            // so it steps up relative to wherever the claw is resting.
            clawRotationLift.stop(); // cancel a homing run still in progress
            clawHomingDir = 0;
            clawRotationLift.moveTo(clawZeroed ? CLAW_PARALLEL_DEG
                                               : clawRotationLift.getPosition() + CLAW_FIRST_L1_STEP_DEG);
            l1Pending = false;
        } else if (l2Pending && nowMs - l2PressedAt >= CLAW_CHORD_WINDOW_MS) {
            clawRotationLift.stop();
            clawHomingDir = -1;
            clawHomingStartedAt = nowMs;
            clawStallTicks = 0;
            clawLastPosition = clawRotationLift.getPosition();
            l2Pending = false;
        }

        // Homing runs a tick at a time here rather than in a blocking loop, so the
        // drivetrain and everything else keep responding while the claw seeks.
        if (clawHomingDir != 0) {
            clawRotationLift.setOutput(clawHomingDir > 0 ? CLAW_HOMING_POWER : -CLAW_HOMING_POWER);

            const float clawPosition = clawRotationLift.getPosition();
            const std::uint32_t clawElapsed = nowMs - clawHomingStartedAt;
            bool contact = false;

            if (clawElapsed >= CLAW_HOMING_GRACE_MS) {
                const bool overCurrent = clawRotationMotor.get_current_draw() >= CLAW_HOMING_CURRENT_MA;
                const bool notMoving = std::abs(clawPosition - clawLastPosition) < CLAW_STALL_EPSILON_DEG;
                if (overCurrent || notMoving) {
                    clawStallTicks++;
                } else {
                    clawStallTicks = 0;
                }
                contact = clawStallTicks >= CLAW_STALL_TICKS || clawElapsed >= CLAW_HOMING_TIMEOUT_MS;
            }
            clawLastPosition = clawPosition;

            if (contact) {
                // setOutput(0) first so the claw stops pressing into the stop before
                // anything else happens. Taring under a live output would tare against a
                // position still being driven.
                clawRotationLift.setOutput(0);
                if (clawHomingDir < 0) {
                    clawRotationLift.initialize(); // bottom stop becomes 0
                    clawZeroed = true;
                }
                clawRotationLift.holdActively();
                clawHomingDir = 0;
                clawStallTicks = 0;
            }
        }

        // Manual claw jog. Takes priority over anything else driving the claw:
        // stop() cancels a homing run or an L1 move, because setOutput() on its own
        // would leave those tasks writing to the same motor.
        const bool clawJogUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        const bool clawJogDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (clawJogUp || clawJogDown) {
            if (!clawManualJogging) {
                clawRotationLift.stop();
                clawHomingDir = 0;
                clawStallTicks = 0;
                clawManualTared = false;
                clawLastPosition = clawRotationLift.getPosition();
                clawHomingStartedAt = nowMs;
            }
            clawRotationLift.setOutput(clawJogUp ? CLAW_MANUAL_POWER : -CLAW_MANUAL_POWER);
            clawManualJogging = true;

            // Jogging down into the bottom stop re-zeros the claw, so the driver can
            // recover a slipped reference without using L2. Only downward: the top
            // stop is not the zero. Taring up would be actively wrong.
            if (clawJogDown && !clawManualTared) {
                const float clawPosition = clawRotationLift.getPosition();
                if (nowMs - clawHomingStartedAt >= CLAW_HOMING_GRACE_MS) {
                    const bool overCurrent = clawRotationMotor.get_current_draw() >= CLAW_HOMING_CURRENT_MA;
                    const bool notMoving = std::abs(clawPosition - clawLastPosition) < CLAW_STALL_EPSILON_DEG;
                    if (overCurrent || notMoving) {
                        clawStallTicks++;
                    } else {
                        clawStallTicks = 0;
                    }
                    if (clawStallTicks >= CLAW_STALL_TICKS) {
                        clawRotationLift.setOutput(0); // stop pressing into the stop
                        clawRotationLift.initialize(); // bottom stop becomes 0
                        clawZeroed = true;
                        clawManualTared = true;
                        clawStallTicks = 0;
                    }
                }
                clawLastPosition = clawPosition;
            }
        } else if (clawManualJogging) {
            clawRotationLift.holdActively();
            clawManualJogging = false;
        }

        // Rising edge only, so holding UP does not flip every tick.
        const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        if (up && !prevUp) {
            reverseFront = !reverseFront;
            // One short buzz so the driver knows which way the bot is pointing
            // without looking at the brain.
            controller.rumble(".");
        }
        prevUp = up;

        const int driveForward = reverseFront ? -forward : forward;
        const int driveSideways = reverseFront ? -sideways : sideways;

        // fieldCentric = false: robot-centric driving. "Forward" is whichever end
        // the toggle above currently calls the front.
        chassis.driveControl(driveForward, driveSideways, rotation,
                             {.movement = movement_curve, .rotation = rotation_curve}, false, 90,
                             {.correctionOn = false, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
        pros::delay(20);
    }
}
