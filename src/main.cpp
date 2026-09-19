#include "main.h"
#include "Eigen/Core" // IWYU pragma: export
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/ApriltagLocalization.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motion_handler.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/replay.hpp"
// Keep liftlib's names under liftlib:: -- hololib::PID already exists and
// would collide otherwise.
#define LIFTLIB_NO_GLOBAL_NAMES
#include "liftlib/liftlib.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <print>



// Motor ports (negative for reversing motor)
int frontLPort = 11;
int frontRPort = 12;
int backLPort = 20;
int backRPort = 17;

// IMU port
int imuPort = 18;

// AI Vision sensor -- not on the robot right now, so left undefined.
// config.hpp declares `extern pros::AIVision visionSensor`, and the build links
// fine without it only because nothing references that symbol. Constructing an
// ApriltagLocalization would make it an undefined-reference link error, so
// uncomment this and the definition below (with the real port) before adding
// any AprilTag localization.
// int visionPort = 5;
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



// Initalize motors, IMU, odometry, and chassis
pros::Motor frontLeft = pros::Motor(frontLPort, pros::MotorGear::blue);
pros::Motor frontRight = pros::Motor(frontRPort, pros::MotorGear::blue);
pros::Motor backLeft = pros::Motor(backLPort, pros::MotorGear::blue);
pros::Motor backRight = pros::Motor(backRPort, pros::MotorGear::blue);

pros::Imu imu = pros::Imu(imuPort);
// pros::AIVision visionSensor = pros::AIVision(visionPort);

pros::Motor clawGripper = pros::Motor(10, pros::MotorGear::green); // open/close rollers

pros::Motor intake = pros::Motor(8, pros::MotorGear::blue);


liftlib::PID clawRotationPID(/*kP=*/5.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/2.0f);
liftlib::Subsystem clawRotationLift(
    {liftlib::MotorConfig{.port = -9,
                          .gear_ratio = 12.0f / 60.0f,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::green}},
    clawRotationPID);

constexpr float LIFT_SPOOL_RADIUS_IN = 0.26f;  // measured
constexpr float LIFT_GEAR_RATIO = 1.0f;       // direct drive, no external reduction
constexpr float LIFT_INCHES_PER_MOTOR_DEGREE =
    LIFT_GEAR_RATIO * (M_PI / 180.0f) * LIFT_SPOOL_RADIUS_IN;
liftlib::Subsystem liftLift(
    {liftlib::MotorConfig{.port = 6,
                          .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::blue},
     liftlib::MotorConfig{.port = -7,
                          .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::blue}},
    std::vector<liftlib::Subsystem::GainPoint>{
        {liftlib::PID(/*kP=*/28.0f, /*kI=*/0.0f, /*kD=*/1.0f, /*threshold=*/1.0f), /*position_in=*/12.0f},
        {liftlib::PID(/*kP=*/20.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/24.0f},
        {liftlib::PID(/*kP=*/0.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/36.0f},
    });










hololib::ChassisConfig chassis_config = {
    .drivetrainWidth = 9.1, .drivetrainLength = 10.25, .wheelDiameter = 3.25, .gearRatio = 0.5};
hololib::EncoderEKFOdometry odom =
hololib::EncoderEKFOdometry(frontLeft, frontRight, backLeft, backRight, imu, chassis_config);
const std::function<hololib::Pose(bool)> poseGetter = [](bool radians) { return odom.getPose(radians); };

hololib::Chassis chassis = hololib::Chassis(frontLeft, frontRight, backLeft, backRight, imu, odom);
hololib::GainScheduler xSched = hololib::GainScheduler();
hololib::GainScheduler ySched = hololib::GainScheduler();
hololib::GainScheduler thetaSched = hololib::GainScheduler();

// Initialize obstacle manager
hololib::ObstacleManager obstacles = hololib::ObstacleManager();

void initialize() {
    pros::lcd::initialize();
    // Calibrate the chassis
    chassis.calibrate();
    odom.startTask();

    // Tare both subsystems and seed their position readings. Whatever pose the
    // claw and lift are resting in at boot becomes 0 for each of them, so power
    // the robot on with the lift retracted.
    clawRotationLift.initialize();
    liftLift.initialize();
    clawRotationLift.setFeedforward(liftlib::Feedforward::cosine(/*kG=*/0.0f, /*horizontal=*/0.0f, /*degreesPerUnit=*/1.0f));

    // Required for the R1/R2 lock in opcontrol: holdActively() early-returns to
    // brake() whenever the feedforward is disabled (isEnabled() is
    // `model != None && kG != 0`), so with this commented out the lift only
    // brakes instead of actively holding.
    //
    // Constant, not cosine -- a cascade's load does not change with height. kG is
    // in liftlib's -127..127 output units and is NOT measured: it came from
    // converting ModularLift's old millivolt feedforward. Tune it by watching
    // whether the lift holds height, sags, or climbs when you let go of R1/R2.
    liftLift.setFeedforward(liftlib::Feedforward::constant(/*kG=*/18.5f));

    // Set PID gains for chassis
    xSched.setGains({
        {36.0, {15, 0, 2.4}},
        {0.0,  {25, 0, 0.5}},
    });

    ySched.setGains({
        {36.0, {15, 0, 1.6}},
        {0.0,  {20, 0, 1.5}},
    });

    thetaSched.setGains({
        {90.0, {2.76411f, 0.0116046f, 0.0384008f}},
        {0,    {3, 0, 0.04}                      }
    });



    // Basically allows you to see the velocity of the chassis (in/s) (helpful
    // for making custom motions)
    odom.setVelocityCalculations(true);

    // LCD screen task to display chassis data

    pros::Task screen_task([&]() {
        while (true) {
            hololib::Pose pose = odom.getPose(false); // false means degrees, true means radians
            pros::lcd::print(0, "X: %.3f", pose.x);
            pros::lcd::print(1, "Y: %.3f", pose.y);
            pros::lcd::print(2, "Theta: %.3f", pose.theta);
            pros::lcd::print(3, "X Velocity: %.3f", pose.velocity.vx);
            pros::lcd::print(4, "Y Velocity: %.3f", pose.velocity.vy);
            pros::lcd::print(5, "Theta Velocity: %.3f", pose.velocity.w);
            pros::lcd::print(6, "Claw rot (deg): %.2f", clawRotationLift.getPosition());
            pros::lcd::print(7, "Lift (in): %.2f", liftLift.getPosition());
            pros::delay(50);
        }
    });
}



void disabled() {
    pros::lcd::print(0, "test");
    odom.setPose(0, 0, 0);
}



void competition_initialize() {}



/*

Run:

python tools/sim_auton.py

then open the file with the browser of your choice.

*/

void simulation() {}



void autonomous() {
  liftLift.moveTo(6.0f);

}




void opcontrol() {

  odom.setKalmanFilterEnabled(true);
  odom.setPose(0, 0, 0);
  hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;

  // --- drive direction toggle (UP) ----------------------------------------
  // Swaps which end of the bot counts as the front, so the driver can lead with
  // either end without turning around. Each press flips it.
  //
  // Flipping the front is a 180 degree rotation of the robot frame, which for a
  // holonomic drive is just negating both translation axes -- forward becomes
  // backward and left becomes right together. Negating only one would mirror the
  // bot instead of turning it, and strafing would end up backwards.
  //
  // Rotation is deliberately NOT negated: a clockwise spin is clockwise no
  // matter which end you call the front, because the bot turns about its centre.
  // Flipping it too would make the right stick fight the driver.
  //
  // This is purely a driver-control convenience. Odometry, heading and every
  // autonomous motion still use the real, unflipped front of the robot.
  bool reverseFront = false;
  bool prevUp = false;

  constexpr float LIFT_JOG_POWER = 127.0f; // liftlib full scale, == 12000 mV
  bool liftJogging = false;

  constexpr float CLAW_PERPENDICULAR_DEG = -40.0f;
  constexpr float CLAW_PARALLEL_DEG = 50.0f;
  constexpr float CLAW_MATCHLOAD_DEG = 60.0f;

  constexpr std::uint32_t CLAW_CHORD_WINDOW_MS = 100;

  bool prevL1 = false;
  bool prevL2 = false;
  bool l1Pending = false;
  bool l2Pending = false;
  std::uint32_t l1PressedAt = 0;
  std::uint32_t l2PressedAt = 0;

  clawRotationLift.holdActively();

  while (true) {
    int forward = controller.get_analog(ANALOG_LEFT_Y);
    int sideways = controller.get_analog(ANALOG_LEFT_X);
    int rotation = controller.get_analog(ANALOG_RIGHT_X);
    if (prev_forward != forward || prev_sideways != sideways ||
        prev_rotation != rotation) {
      prev_forward = forward;
      prev_sideways = sideways;
      prev_rotation = rotation;
    }

    clawGripper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // R1/R2 jog the elevator open loop. liftLift (the PID subsystem on the same
    // motors) is only used by moveTo() in autonomous, so the two never fight.
    // R1/R2 jog the lift, and releasing both locks it where it is.
    //
    // Driven through liftLift rather than the raw motor group so the two cannot
    // fight over ports 6/-7: setOutput() bypasses the PID for a straight jog and
    // stops any running hold task itself, and holdActively() then starts a hold
    // at the current height. The bool tracks the release edge so the hold task
    // is started once instead of being torn down and rebuilt every idle tick.
    //
    // LIFT_JOG_POWER is liftlib's full scale (VOLTAGE_OUTPUT_LIMIT = 127), which
    // maps to the same 12000 mV the raw move_voltage call used before.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      liftLift.setOutput(LIFT_JOG_POWER);
      liftJogging = true;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      liftLift.setOutput(-LIFT_JOG_POWER);
      liftJogging = true;
    } else if (liftJogging) {
      liftLift.holdActively();
      liftJogging = false;
    }

    // B and Y drive the intake rollers and the claw gripper together, one
    // button per direction. This replaces the old DOWN/RIGHT intake-only
    // bindings -- they're folded in here, so DOWN and RIGHT are now free.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      std::cout << "Intake in / claw in" << std::endl;
      intake.move_voltage(-6000);
      clawGripper.move_voltage(6000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      std::cout << "Intake out / claw out" << std::endl;
      intake.move_voltage(12000);
      clawGripper.move_voltage(-12000);
    } else {

      intake.move_voltage(0);
      clawGripper.move_voltage(0);
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
      clawRotationLift.moveTo(CLAW_MATCHLOAD_DEG);
      l1Pending = false;
      l2Pending = false;
    } else if (l1Pending && nowMs - l1PressedAt >= CLAW_CHORD_WINDOW_MS) {
      clawRotationLift.moveTo(CLAW_PARALLEL_DEG);
      l1Pending = false;
    } else if (l2Pending && nowMs - l2PressedAt >= CLAW_CHORD_WINDOW_MS) {
      clawRotationLift.moveTo(CLAW_PERPENDICULAR_DEG);
      l2Pending = false;
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
    chassis.driveControl(
        driveForward, driveSideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, false, 90,
        {.correctionOn = false, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
    pros::delay(20);
  }

}