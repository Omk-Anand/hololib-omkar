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

pros::MotorGroup liftMotors = pros::MotorGroup({6, -7});
pros::Motor clawGripper = pros::Motor(10, pros::MotorGear::green); // open/close rollers

pros::Motor intake = pros::Motor(8, pros::MotorGear::blue);

// --- liftlib PID subsystems -------------------------------------------------
// A liftlib::Subsystem is a motor group under a PID. gear_ratio is applied for
// you, so moveTo() targets are in real units at the mechanism.
//
// File scope, not local to initialize(): they own background hold tasks and are
// used from opcontrol(), so they must outlive any one function.

// Claw pivot, port 9 reversed. Targets are degrees at the claw; 12:60 is the
// gearbox (5:1 reduction).
liftlib::PID clawRotationPID(/*kP=*/15.0f, /*kI=*/0.0001f, /*kD=*/16.0f, /*threshold=*/2.0f);
liftlib::Subsystem clawRotationLift(
    {liftlib::MotorConfig{.port = -9,
                          .gear_ratio = 12.0f / 60.0f,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::green}},
    clawRotationPID);

// Elevator, ports 6/-7 -- the same motors liftMotors jogs on R1/R2, here under
// closed-loop control. Targets are INCHES from wherever it sits at boot:
//   inches = motor_degrees * gear_ratio * (pi/180) * spool_radius
// (arc length = radius * angle). Both constants are measured, not tuned.
//
// Gain schedule, not one PID: a cascade is floppier extended than retracted.
// liftlib picks gains by POSITION and interpolates between the two points.
// 10" and 19" are the working heights. Tune each independently -- expect 19" to
// want less kP than 10".
constexpr float LIFT_SPOOL_RADIUS_IN = 0.4f;  // measured
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
        {liftlib::PID(/*kP=*/10.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/10.0f},
        {liftlib::PID(/*kP=*/1.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/19.0f},
    });










// Initialize odometry configuration. This has to be constructed before chassis:
// EncoderEKFOdometry reads the motors and IMU in its constructor, and chassis
// binds a reference to it, so defining chassis first would capture an object
// that has not run its constructor yet.

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

    // The claw pivots, so the torque needed to hold it depends on its angle --
    // cosine, not a constant push. Both numbers are untuned:
    //   kG:         raise from 0 until holdActively() holds the claw level with
    //               no sag and no climb.
    //   horizontal: the clawRotationLift.getPosition() reading (LCD line 6)
    //               where the claw is actually level. 0 assumes it boots level.
    clawRotationLift.setFeedforward(liftlib::Feedforward::cosine(/*kG=*/0.0f, /*horizontal=*/0.0f, /*degreesPerUnit=*/1.0f));

    // Constant, not cosine -- a cascade's load doesn't change with height. kG is
    // in liftlib's -127..127 output units. Verify by watching whether the lift
    // holds height or sags after settling.
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

    chassisAsync(hololib::turnToHeading(90));
    hololib::motion_handler::waitUntilDone();

}



void opcontrol() {
  // Field-centric driving rotates the joystick vector by the heading odom
  // reports every tick, so it's only correct while heading doesn't drift. With
  // the EKF off, heading integrates from wheel encoders alone, which drifts
  // fast on an X-drive with no tracking wheels. Leaving the EKF on fuses in the
  // IMU instead. The fieldCentric flag below does nothing useful without this.
  odom.setKalmanFilterEnabled(true);
  odom.setPose(0, 0, 0);
  hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;

  constexpr float CLAW_ROT_JOG_POWER = 60.0f; // -127..127, untuned -- raise if it jogs too slowly
  bool clawRotJogging = false;

  // Hold from the start, not just after the first L1/L2 press, so the claw
  // fights gravity immediately.
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

    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawGripper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // R1/R2 jog the elevator open loop. liftLift (the PID subsystem on the same
    // motors) is only used by moveTo() in autonomous, so the two never fight.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      std::cout << "Lift up" << std::endl;
      liftMotors.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      std::cout << "Lift down" << std::endl;
      liftMotors.move_voltage(-12000);
    } else {
      liftMotors.move_voltage(0);
    }

    // B and Y drive the intake rollers and the claw gripper together, one
    // button per direction. This replaces the old DOWN/RIGHT intake-only
    // bindings -- they're folded in here, so DOWN and RIGHT are now free.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      std::cout << "Intake in / claw in" << std::endl;
      intake.move_voltage(-12000);
      clawGripper.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      std::cout << "Intake out / claw out" << std::endl;
      intake.move_voltage(12000);
      clawGripper.move_voltage(-12000);
    } else {
      intake.move_voltage(0);
      clawGripper.move_voltage(0);
    }

    // L1/L2 jog the claw pivot. setOutput() bypasses the PID and stops any
    // active hold itself, so jogging can't fight the hold task. The bool only
    // tracks the release edge -- calling holdActively() every idle tick would
    // tear down and restart its background task 50 times a second for nothing.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      clawRotationLift.setOutput(CLAW_ROT_JOG_POWER);
      clawRotJogging = true;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      clawRotationLift.setOutput(-CLAW_ROT_JOG_POWER);
      clawRotJogging = true;
    } else if (clawRotJogging) {
      clawRotationLift.holdActively();
      clawRotJogging = false;
    }

    // fieldCentric = true: "forward" always means the direction the bot faced
    // when opcontrol() started (heading 0, from setPose above), whatever way
    // the bot is currently pointing.
    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, true, 90,
        {.correctionOn = false, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
    pros::delay(20);
  }

}
