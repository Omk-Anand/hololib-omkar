#pragma once

#include "Eigen/Core" // IWYU pragma: export
#include "Eigen/src/Core/Matrix.h"
#include "hololib/localization/odometry.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"
#include <array>
#include <numbers>
#include <utility>

namespace hololib {

/** @brief Estimates the robot's field pose from AprilTag detections. */
class ApriltagLocalization {
public:
    /** @brief The two pose solutions produced by the IPPE square solver. */
    struct IPPEOutput {
        /** Rotation vector for the lower-error solution, in radians. */
        Eigen::Vector3f rvec1;
        /** Rotation vector for the alternate solution, in radians. */
        Eigen::Vector3f rvec2;
        /** Translation vector for the lower-error solution. */
        Eigen::Vector3f tvec1;
        /** Translation vector for the alternate solution. */
        Eigen::Vector3f tvec2;
        /** Reprojection error of the first solution, in pixels. */
        float reprojection_error1;
        /** Reprojection error of the second solution, in pixels. */
        float reprojection_error2;
        /** Whether both solutions contain finite reprojection errors. */
        bool valid;
    };

    /**
     * @brief Constructs an AprilTag localization system.
     * @param visionSensor AI Vision sensor used for tag detections.
     * @param imu IMU associated with the robot.
     * @param odomPoseGetter Callback returning the current odometry pose; its
     * boolean argument selects radians.
     * @param cameraMatrix Camera intrinsic matrix.
     * @param distCoeffs Distortion coefficients ordered as k1, k2, p1, p2, k3.
     * @param visionSensorOffset Camera offset from the robot tracking center as
     * +x forward and +y right, in inches.
     * @param visionSensorYawOffset Camera yaw relative to the robot, in radians.
     *
     * visionSensorOffset is measured from the robot tracking center to the
     * camera optical center in robot coordinates: +x forward and +y right.
     * visionSensorYawOffset and returned Pose angles use the odometry convention:
     * zero faces field +Y and positive rotation is clockwise. Angles passed to
     * this function through Pose are radians.
     * All translations must use the same units as FIELD_TAGS (inches).
     */
    ApriltagLocalization(pros::AIVision& visionSensor,
                         pros::Imu& imu,
                         std::function<hololib::Pose(bool)> odomPoseGetter,
                         const Eigen::Matrix3f& cameraMatrix,
                         const Eigen::Vector<float, 5>& distCoeffs,
                         const Eigen::Vector2f& visionSensorOffset = Eigen::Vector2f::Zero(),
                         float visionSensorYawOffset = 0.0f)
        : visionSensor(visionSensor), imu(imu), odomPoseGetter(odomPoseGetter), cameraMatrix(cameraMatrix),
          distCoeffs(distCoeffs), visionSensorOffset(visionSensorOffset),
          visionSensorYawOffset(visionSensorYawOffset) {};

    /**
     * @brief Continuously processes tag detections and updates the estimated pose.
     * @param period_ms Update period in milliseconds.
     */
    void update(uint32_t period_ms);

    /**
     * @brief Starts the localization task if it is not already running.
     * @param period_ms Update period in milliseconds.
     * 
     * @b Example:
     * @code
     * // Start the AprilTag localization task with a 100 ms update period.
     * initialize() {
     *     aprilTagLocalization.startTask(100);
     * }
     * @endcode
     */
    void startTask(uint32_t period_ms = 10);

    /**
     * @brief Returns the latest tag-based pose estimate.
     * @param useRadians Return heading in radians when true, otherwise degrees.
     * @return The latest estimated field pose.
     *
     * @b Example:
     * @code
     * // Get the latest tag-based pose estimate in radians.
     * tagOdom.startTask(100);
     * while (true) {
     *    hololib::Pose p = tagOdom.getPoseFromTag(false);
     *    std::println("Tag pose: x: {}, y: {}, theta: {}", p.x, p.y, p.theta);
     *    pros::delay(100);
     * }
     * @endcode
     */
    Pose getPoseFromTag(bool useRadians = false);

    /**
     * @brief Estimates the robot's field pose from a 6-DOF camera pose produced by IPPE.
     * @param ippeOutput Candidate camera poses produced by IPPE.
     * @param odometryPose Current odometry pose, with heading in radians.
     * @param tagId Detected field tag ID.
     * @return The best matching field pose, or odometryPose if no valid match exists.
     */
    Pose estimateRobotPose(const IPPEOutput& ippeOutput, const Pose& odometryPose, int tagId) const;

private:
    pros::AIVision& visionSensor;
    pros::Imu& imu;
    std::function<hololib::Pose(bool)> odomPoseGetter;
    const float IPPE_SMALL = 1e-6f;
    /**
     * Tag edge length, in inches. This sets the scale of the IPPE solution, so
     * the translation it returns comes back in these units and must match the
     * units used by FIELD_TAGS. Measure the black square of the printed tag and
     * update this if your tags are a different size.
     */
    const float APRILTAG_SIZE = 2.0f;
    static constexpr float RAD2DEG = 180.0f / M_PI;
    const Eigen::Matrix3f cameraMatrix;
    const Eigen::Vector<float, 5> distCoeffs;
    const Eigen::Vector2f visionSensorOffset;
    const float visionSensorYawOffset;
    Pose currentPose;

    pros::Mutex poseMutex;
    std::optional<pros::Task> m_task = std::nullopt;

    struct SortedPoses {
        Eigen::Matrix4f M1;
        Eigen::Matrix4f M2;
        float err1;
        float err2;
    };

    struct FieldTag {
        int tagId;
        float yawRad;
        std::array<float, 3> position;
    };

    static constexpr float PI = std::numbers::pi_v<float>;

    static constexpr float YAW_POS_X = 0.0f;
    static constexpr float YAW_POS_Y = PI / 2.0f;
    static constexpr float YAW_NEG_X = PI;
    static constexpr float YAW_NEG_Y = 3.0f * PI / 2.0f;

    static constexpr std::array<FieldTag, 36> FIELD_TAGS{
        {
         // goal 1 +x side
            {1, YAW_POS_Y, {46.640f, -21.455f, 4.355f}},
         {1, YAW_NEG_X, {44.775f, -23.320f, 4.355f}},
         {1, YAW_NEG_Y, {46.640f, -25.185f, 4.355f}},
         {1, YAW_POS_X, {48.505f, -23.320f, 4.355f}},

         // goal 2 +x side
            {2, YAW_POS_Y, {46.640f, 25.185f, 4.355f}},
         {2, YAW_NEG_X, {44.775f, 23.320f, 4.355f}},
         {2, YAW_NEG_Y, {46.640f, 21.455f, 4.355f}},
         {2, YAW_POS_X, {48.505f, 23.320f, 4.355f}},

         // goal 3 +y side
            {3, YAW_POS_Y, {23.320f, 48.505f, 4.355f}},
         {3, YAW_NEG_X, {21.455f, 46.640f, 4.355f}},
         {3, YAW_NEG_Y, {23.320f, 44.775f, 4.355f}},
         {3, YAW_POS_X, {25.185f, 46.640f, 4.355f}},

         // goal 4: +y side
            {4, YAW_POS_Y, {-23.320f, 48.505f, 4.355f}},
         {4, YAW_NEG_X, {-25.185f, 46.640f, 4.355f}},
         {4, YAW_NEG_Y, {-23.320f, 44.775f, 4.355f}},
         {4, YAW_POS_X, {-21.455f, 46.640f, 4.355f}},

         // goal 1: -x side
            {1, YAW_POS_Y, {-46.640f, 25.185f, 4.355f}},
         {1, YAW_NEG_X, {-48.505f, 23.320f, 4.355f}},
         {1, YAW_NEG_Y, {-46.640f, 21.455f, 4.355f}},
         {1, YAW_POS_X, {-44.775f, 23.320f, 4.355f}},

         // goal 2: -x side
            {2, YAW_POS_Y, {-46.640f, -21.455f, 4.355f}},
         {2, YAW_NEG_X, {-48.505f, -23.320f, 4.355f}},
         {2, YAW_NEG_Y, {-46.640f, -25.185f, 4.355f}},
         {2, YAW_POS_X, {-44.775f, -23.320f, 4.355f}},

         // goal 3: -y side
            {3, YAW_POS_Y, {-23.320f, -44.775f, 4.355f}},
         {3, YAW_NEG_X, {-25.185f, -46.640f, 4.355f}},
         {3, YAW_NEG_Y, {-23.320f, -48.505f, 4.355f}},
         {3, YAW_POS_X, {-21.455f, -46.640f, 4.355f}},

         // goal 4: -y side
            {4, YAW_POS_Y, {23.320f, -44.775f, 4.355f}},
         {4, YAW_NEG_X, {21.455f, -46.640f, 4.355f}},
         {4, YAW_NEG_Y, {23.320f, -48.505f, 4.355f}},
         {4, YAW_POS_X, {25.185f, -46.640f, 4.355f}},

         // center goal
            {0, YAW_POS_Y, {0.000f, 1.865f, 4.355f}},
         {0, YAW_NEG_X, {-1.865f, 0.000f, 4.355f}},
         {0, YAW_NEG_Y, {0.000f, -1.865f, 4.355f}},
         {0, YAW_POS_X, {1.865f, 0.000f, 4.355f}},
         }
    };

    std::array<Eigen::Vector2f, 4> generateSquareObjectCorners2D(float squareLength);

    std::array<Eigen::Vector3f, 4> generateSquareObjectCorners3D(float squareLength);

    Eigen::Matrix3f rotateVec2ZAxis(const Eigen::Vector3f& a);

    Eigen::Vector3f rot2vec(const Eigen::Matrix3f& R);

    Eigen::Vector3f computeTranslation(const std::array<Eigen::Vector2f, 4>& objectPoints,
                                       const std::array<Eigen::Vector2f, 4>& normalizedImgPoints,
                                       const Eigen::Matrix3f& R);

    std::pair<Eigen::Matrix3f, Eigen::Matrix3f>
    computeRotations(float j00, float j01, float j10, float j11, float p, float q);

    std::pair<Eigen::Matrix4f, Eigen::Matrix4f>
    solveCanonicalForm(const std::array<Eigen::Vector2f, 4>& canonicalObjPoints,
                       const std::array<Eigen::Vector2f, 4>& normalizedInputPoints,
                       const Eigen::Matrix3f& H);

    Eigen::Matrix3f homographyFromSquarePoints(const std::array<Eigen::Vector2f, 4>& targetPoints, float halfLength);

    std::array<Eigen::Vector2f, 4> projectPoints(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                                 const Eigen::Matrix3f& R,
                                                 const Eigen::Vector3f& t,
                                                 const Eigen::Matrix3f& cameraMatrix,
                                                 const Eigen::Vector<float, 5>& distCoeffs);

    float evalReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                          const std::array<Eigen::Vector2f, 4>& imagePoints,
                          const Eigen::Matrix3f& cameraMatrix,
                          const Eigen::Vector<float, 5>& distCoeffs,
                          const Eigen::Matrix4f& M);

    SortedPoses sortPosesByReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                       const std::array<Eigen::Vector2f, 4>& imagePoints,
                                       const Eigen::Matrix3f& cameraMatrix,
                                       const Eigen::Vector<float, 5>& distCoeffs,
                                       const Eigen::Matrix4f& Ma,
                                       const Eigen::Matrix4f& Mb);

    std::array<Eigen::Vector2f, 4> undistortPoints5Coeffs(const std::array<Eigen::Vector2f, 4>& src,
                                                          const Eigen::Matrix3f& cameraMatrix,
                                                          const Eigen::Vector<float, 5>& distCoeffs,
                                                          int maxIterations = 5,
                                                          float epsilon = 0.01f);

    IPPEOutput IPPESquare(const std::array<Eigen::Vector2f, 4>& imagePoints,
                          const Eigen::Matrix3f& cameraMatrix,
                          const Eigen::Vector<float, 5>& distCoeffs,
                          float squareLength);

    std::pair<Eigen::Vector3f, Eigen::Vector3f> selectBestIPPESolution(const IPPEOutput& ippeOutput,
                                                                       const Eigen::Quaternionf& imuQuat,
                                                                       const Eigen::Quaternionf& cameraToImuRotation);
};
} // namespace hololib
