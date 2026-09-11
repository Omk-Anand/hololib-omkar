#include "hololib/localization/ApriltagLocalization.hpp"
#include "Eigen/Core"     // IWYU pragma: export
#include "Eigen/Geometry" // IWYU pragma: export
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <print>

namespace hololib {

std::array<Eigen::Vector2f, 4> ApriltagLocalization::generateSquareObjectCorners2D(float squareLength) {
    std::array<Eigen::Vector2f, 4> corners;
    float halfLength = squareLength / 2.0f;

    corners[0] = {-halfLength, halfLength};
    corners[1] = {halfLength, halfLength};
    corners[2] = {halfLength, -halfLength};
    corners[3] = {-halfLength, -halfLength};

    return corners;
}

std::array<Eigen::Vector3f, 4> ApriltagLocalization::generateSquareObjectCorners3D(float squareLength) {
    std::array<Eigen::Vector3f, 4> corners;
    float halfLength = squareLength / 2.0f;

    corners[0] = {-halfLength, halfLength, 0.0f};
    corners[1] = {halfLength, halfLength, 0.0f};
    corners[2] = {halfLength, -halfLength, 0.0f};
    corners[3] = {-halfLength, -halfLength, 0.0f};

    return corners;
}

Eigen::Matrix3f ApriltagLocalization::rotateVec2ZAxis(const Eigen::Vector3f& a) {
    return Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), a).toRotationMatrix();
}

Eigen::Vector3f ApriltagLocalization::rot2vec(const Eigen::Matrix3f& R) {
    Eigen::AngleAxisf angleAxis(R);
    return angleAxis.axis() * angleAxis.angle();
}

Eigen::Vector3f ApriltagLocalization::computeTranslation(const std::array<Eigen::Vector2f, 4>& objectPoints,
                                                         const std::array<Eigen::Vector2f, 4>& normalizedImgPoints,
                                                         const Eigen::Matrix3f& R) {
    constexpr float n = 4.0f;

    float ATA00 = n;
    float ATA02 = 0.0f;
    float ATA11 = n;
    float ATA12 = 0.0f;
    float ATA20 = 0.0f;
    float ATA21 = 0.0f;
    float ATA22 = 0.0f;

    float ATb0 = 0.0f;
    float ATb1 = 0.0f;
    float ATb2 = 0.0f;

    for (int i = 0; i < 4; i++) {
        float X = objectPoints[i].x();
        float Y = objectPoints[i].y();
        float u = normalizedImgPoints[i].x();
        float v = normalizedImgPoints[i].y();

        float rx = R(0, 0) * X + R(0, 1) * Y;
        float ry = R(1, 0) * X + R(1, 1) * Y;
        float rz = R(2, 0) * X + R(2, 1) * Y;

        float a2 = -u;
        float b2 = -v;

        ATA02 += a2;
        ATA12 += b2;
        ATA20 += a2;
        ATA21 += b2;
        ATA22 += a2 * a2 + b2 * b2;

        float bx = -a2 * rz - rx;
        float by = -b2 * rz - ry;

        ATb0 += bx;
        ATb1 += by;
        ATb2 += a2 * bx + b2 * by;
    }

    const float det = ATA00 * ATA11 * ATA22 - ATA00 * ATA12 * ATA21 - ATA02 * ATA11 * ATA20;
    if (!std::isfinite(det) || std::abs(det) < IPPE_SMALL) {
        return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    const float detAInv = 1.0f / det;

    const float S00 = ATA11 * ATA22 - ATA12 * ATA21;
    const float S01 = ATA02 * ATA21;
    const float S02 = -ATA02 * ATA11;
    const float S10 = ATA12 * ATA20;
    const float S11 = ATA00 * ATA22 - ATA02 * ATA20;
    const float S12 = -ATA00 * ATA12;
    const float S20 = -ATA11 * ATA20;
    const float S21 = -ATA00 * ATA21;
    const float S22 = ATA00 * ATA11;

    return Eigen::Vector3f(detAInv * (S00 * ATb0 + S01 * ATb1 + S02 * ATb2),
                           detAInv * (S10 * ATb0 + S11 * ATb1 + S12 * ATb2),
                           detAInv * (S20 * ATb0 + S21 * ATb1 + S22 * ATb2));
}

std::pair<Eigen::Matrix3f, Eigen::Matrix3f>
ApriltagLocalization::computeRotations(float j00, float j01, float j10, float j11, float p, float q) {
    Eigen::Matrix3f Rv = rotateVec2ZAxis(Eigen::Vector3f(p, q, 1.0f));

    Eigen::Matrix2f B;
    B << Rv(0, 0) - p * Rv(2, 0), Rv(0, 1) - p * Rv(2, 1), Rv(1, 0) - q * Rv(2, 0), Rv(1, 1) - q * Rv(2, 1);

    Eigen::Matrix2f J;
    J << j00, j01, j10, j11;

    const float detB = B.determinant();
    if (!std::isfinite(detB) || std::abs(detB) < IPPE_SMALL) {
        const Eigen::Matrix3f invalid = Eigen::Matrix3f::Constant(std::numeric_limits<float>::quiet_NaN());
        return {invalid, invalid};
    }
    Eigen::Matrix2f A = B.inverse() * J;

    Eigen::JacobiSVD<Eigen::Matrix2f> svd(A);
    float gamma = svd.singularValues()(0);
    if (!std::isfinite(gamma) || std::abs(gamma) < std::numeric_limits<float>::epsilon()) {
        const Eigen::Matrix3f invalid = Eigen::Matrix3f::Constant(std::numeric_limits<float>::quiet_NaN());
        return {invalid, invalid};
    }

    Eigen::Matrix2f Rtilde = A / gamma;

    float b0 = std::sqrt(std::max(0.0f, 1.0f - Rtilde(0, 0) * Rtilde(0, 0) - Rtilde(1, 0) * Rtilde(1, 0)));
    float b1 = std::sqrt(std::max(0.0f, 1.0f - Rtilde(0, 1) * Rtilde(0, 1) - Rtilde(1, 1) * Rtilde(1, 1)));

    float sp = -Rtilde(0, 0) * Rtilde(0, 1) - Rtilde(1, 0) * Rtilde(1, 1);
    if (sp < 0.0f)
        b1 = -b1;

    Eigen::Matrix3f R_tilde_1, R_tilde_2;

    R_tilde_1.col(0) = Eigen::Vector3f(Rtilde(0, 0), Rtilde(1, 0), b0);
    R_tilde_1.col(1) = Eigen::Vector3f(Rtilde(0, 1), Rtilde(1, 1), b1);
    R_tilde_1.col(2) = R_tilde_1.col(0).cross(R_tilde_1.col(1));

    R_tilde_2.col(0) = Eigen::Vector3f(Rtilde(0, 0), Rtilde(1, 0), -b0);
    R_tilde_2.col(1) = Eigen::Vector3f(Rtilde(0, 1), Rtilde(1, 1), -b1);
    R_tilde_2.col(2) = R_tilde_2.col(0).cross(R_tilde_2.col(1));

    return {Rv * R_tilde_1, Rv * R_tilde_2};
}

std::pair<Eigen::Matrix4f, Eigen::Matrix4f>
ApriltagLocalization::solveCanonicalForm(const std::array<Eigen::Vector2f, 4>& canonicalObjPoints,
                                         const std::array<Eigen::Vector2f, 4>& normalizedInputPoints,
                                         const Eigen::Matrix3f& H) {
    Eigen::Matrix4f Ma = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mb = Eigen::Matrix4f::Identity();

    float j00 = H(0, 0) - H(2, 0) * H(0, 2);
    float j01 = H(0, 1) - H(2, 1) * H(0, 2);
    float j10 = H(1, 0) - H(2, 0) * H(1, 2);
    float j11 = H(1, 1) - H(2, 1) * H(1, 2);

    auto [Ra, Rb] = computeRotations(j00, j01, j10, j11, H(0, 2), H(1, 2));

    Ma.block<3, 3>(0, 0) = Ra;
    Mb.block<3, 3>(0, 0) = Rb;

    Ma.block<3, 1>(0, 3) = computeTranslation(canonicalObjPoints, normalizedInputPoints, Ra);
    Mb.block<3, 1>(0, 3) = computeTranslation(canonicalObjPoints, normalizedInputPoints, Rb);

    return {Ma, Mb};
}

Eigen::Matrix3f ApriltagLocalization::homographyFromSquarePoints(const std::array<Eigen::Vector2f, 4>& targetPoints,
                                                                 float halfLength) {
    Eigen::Matrix3f H;

    float p1x = -targetPoints[0].x(), p1y = -targetPoints[0].y();
    float p2x = -targetPoints[1].x(), p2y = -targetPoints[1].y();
    float p3x = -targetPoints[2].x(), p3y = -targetPoints[2].y();
    float p4x = -targetPoints[3].x(), p4y = -targetPoints[3].y();

    const float det =
        halfLength * (p1x * p2y - p2x * p1y - p1x * p4y + p2x * p3y - p3x * p2y + p4x * p1y + p3x * p4y - p4x * p3y);
    if (!std::isfinite(det) || std::abs(det) < 1e-9f) {
        return Eigen::Matrix3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    const float detsInv = -1.0f / det;

    H(0, 0) = detsInv * (p1x * p3x * p2y - p2x * p3x * p1y - p1x * p4x * p2y + p2x * p4x * p1y - p1x * p3x * p4y +
                         p1x * p4x * p3y + p2x * p3x * p4y - p2x * p4x * p3y);
    H(0, 1) = detsInv * (p1x * p2x * p3y - p1x * p3x * p2y - p1x * p2x * p4y + p2x * p4x * p1y + p1x * p3x * p4y -
                         p3x * p4x * p1y - p2x * p4x * p3y + p3x * p4x * p2y);
    H(0, 2) = detsInv * halfLength *
              (p1x * p2x * p3y - p2x * p3x * p1y - p1x * p2x * p4y + p1x * p4x * p2y - p1x * p4x * p3y +
               p3x * p4x * p1y + p2x * p3x * p4y - p3x * p4x * p2y);
    H(1, 0) = detsInv * (p1x * p2y * p3y - p2x * p1y * p3y - p1x * p2y * p4y + p2x * p1y * p4y - p3x * p1y * p4y +
                         p4x * p1y * p3y + p3x * p2y * p4y - p4x * p2y * p3y);
    H(1, 1) = detsInv * (p2x * p1y * p3y - p3x * p1y * p2y - p1x * p2y * p4y + p4x * p1y * p2y + p1x * p3y * p4y -
                         p4x * p1y * p3y - p2x * p3y * p4y + p3x * p2y * p4y);
    H(1, 2) = detsInv * halfLength *
              (p1x * p2y * p3y - p3x * p1y * p2y - p2x * p1y * p4y + p4x * p1y * p2y - p1x * p3y * p4y +
               p3x * p1y * p4y + p2x * p3y * p4y - p4x * p2y * p3y);
    H(2, 0) =
        -detsInv * (p1x * p3y - p3x * p1y - p1x * p4y - p2x * p3y + p3x * p2y + p4x * p1y + p2x * p4y - p4x * p2y);
    H(2, 1) = detsInv * (p1x * p2y - p2x * p1y - p1x * p3y + p3x * p1y + p2x * p4y - p4x * p2y - p3x * p4y + p4x * p3y);
    H(2, 2) = 1.0f;

    return H;
}

std::array<Eigen::Vector2f, 4> ApriltagLocalization::projectPoints(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                                                   const Eigen::Matrix3f& R,
                                                                   const Eigen::Vector3f& t,
                                                                   const Eigen::Matrix3f& cameraMatrix,
                                                                   const Eigen::Vector<float, 5>& distCoeffs) {
    std::array<Eigen::Vector2f, 4> projectedPoints;

    const bool normalizedCoordinates = cameraMatrix.isZero();
    float fx = normalizedCoordinates ? 1.0f : cameraMatrix(0, 0);
    float fy = normalizedCoordinates ? 1.0f : cameraMatrix(1, 1);
    float cx = normalizedCoordinates ? 0.0f : cameraMatrix(0, 2);
    float cy = normalizedCoordinates ? 0.0f : cameraMatrix(1, 2);

    float k1 = normalizedCoordinates ? 0.0f : distCoeffs(0);
    float k2 = normalizedCoordinates ? 0.0f : distCoeffs(1);
    float p1 = normalizedCoordinates ? 0.0f : distCoeffs(2);
    float p2 = normalizedCoordinates ? 0.0f : distCoeffs(3);
    float k3 = normalizedCoordinates ? 0.0f : distCoeffs(4);

    for (size_t i = 0; i < 4; ++i) {
        Eigen::Vector3f p_cam = R * objectPoints[i] + t;

        float x = p_cam.x() / p_cam.z();
        float y = p_cam.y() / p_cam.z();

        float r2 = x * x + y * y;
        float r4 = r2 * r2;
        float r6 = r4 * r2;

        float cdist = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
        float x_dist = x * cdist + 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
        float y_dist = y * cdist + p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;

        projectedPoints[i] = Eigen::Vector2f(fx * x_dist + cx, fy * y_dist + cy);
    }

    return projectedPoints;
}

float ApriltagLocalization::evalReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                            const std::array<Eigen::Vector2f, 4>& imagePoints,
                                            const Eigen::Matrix3f& cameraMatrix,
                                            const Eigen::Vector<float, 5>& distCoeffs,
                                            const Eigen::Matrix4f& M) {
    Eigen::Matrix3f R = M.block<3, 3>(0, 0);
    Eigen::Vector3f t = M.block<3, 1>(0, 3);

    std::array<Eigen::Vector2f, 4> projectedPoints = projectPoints(objectPoints, R, t, cameraMatrix, distCoeffs);

    float errSqSum = 0.0f;
    constexpr size_t n = 4;

    for (size_t i = 0; i < n; i++) {
        Eigen::Vector2f diff = projectedPoints[i] - imagePoints[i];
        errSqSum += diff.squaredNorm();
    }

    return std::sqrt(errSqSum / (2.0f * static_cast<float>(n)));
}

ApriltagLocalization::SortedPoses
ApriltagLocalization::sortPosesByReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                             const std::array<Eigen::Vector2f, 4>& imagePoints,
                                             const Eigen::Matrix3f& cameraMatrix,
                                             const Eigen::Vector<float, 5>& distCoeffs,
                                             const Eigen::Matrix4f& Ma,
                                             const Eigen::Matrix4f& Mb) {
    float erra = evalReprojError(objectPoints, imagePoints, cameraMatrix, distCoeffs, Ma);
    float errb = evalReprojError(objectPoints, imagePoints, cameraMatrix, distCoeffs, Mb);

    if (erra < errb) {
        return {Ma, Mb, erra, errb};
    } else {
        return {Mb, Ma, errb, erra};
    }
}

std::array<Eigen::Vector2f, 4> ApriltagLocalization::undistortPoints5Coeffs(const std::array<Eigen::Vector2f, 4>& src,
                                                                            const Eigen::Matrix3f& cameraMatrix,
                                                                            const Eigen::Vector<float, 5>& distCoeffs,
                                                                            int maxIterations,
                                                                            float epsilon) {
    const float fx = cameraMatrix(0, 0);
    const float fy = cameraMatrix(1, 1);
    const float ifx = 1.0f / fx;
    const float ify = 1.0f / fy;
    const float cx = cameraMatrix(0, 2);
    const float cy = cameraMatrix(1, 2);

    const float k1 = distCoeffs(0);
    const float k2 = distCoeffs(1);
    const float p1 = distCoeffs(2);
    const float p2 = distCoeffs(3);
    const float k3 = distCoeffs(4);

    std::array<Eigen::Vector2f, 4> dst;
    for (size_t i = 0; i < dst.size(); ++i) {
        const float u = src[i].x();
        const float v = src[i].y();

        float x = (u - cx) * ifx;
        float y = (v - cy) * ify;
        const float x0 = x;
        const float y0 = y;

        float error = std::numeric_limits<float>::max();
        float prevError = std::numeric_limits<float>::max();
        float alpha = 1.0f;

        for (int j = 0; j < maxIterations && error >= epsilon; ++j) {
            float r2 = x * x + y * y;
            const float icdist = 1.0f / (1.0f + ((k3 * r2 + k2) * r2 + k1) * r2);

            if (icdist < 0.0f) {
                x = (u - cx) * ifx;
                y = (v - cy) * ify;
                break;
            }

            const float deltaX = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
            const float deltaY = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;
            const float newX = (1.0f - alpha) * x + alpha * (x0 - deltaX) * icdist;
            const float newY = (1.0f - alpha) * y + alpha * (y0 - deltaY) * icdist;

            if (epsilon > 0.0f) {
                r2 = newX * newX + newY * newY;
                const float r4 = r2 * r2;
                const float r6 = r4 * r2;
                const float a1 = 2.0f * newX * newY;
                const float a2 = r2 + 2.0f * newX * newX;
                const float a3 = r2 + 2.0f * newY * newY;
                const float cdist = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
                const float xProj = (newX * cdist + p1 * a1 + p2 * a2) * fx + cx;
                const float yProj = (newY * cdist + p1 * a3 + p2 * a1) * fy + cy;
                const float dx = xProj - u;
                const float dy = yProj - v;
                error = std::sqrt(dx * dx + dy * dy);
            }

            if (error > prevError) {
                alpha *= 0.5f;
            } else {
                x = newX;
                y = newY;
            }
            prevError = error;
        }

        dst[i] = Eigen::Vector2f(x, y);
    }

    return dst;
}

ApriltagLocalization::IPPEOutput ApriltagLocalization::IPPESquare(const std::array<Eigen::Vector2f, 4>& imagePoints,
                                                                  const Eigen::Matrix3f& cameraMatrix,
                                                                  const Eigen::Vector<float, 5>& distCoeffs,
                                                                  float squareLength) {

    IPPEOutput output;

    // allocate outputs (zero-initialization):
    output.rvec1.setZero();
    output.tvec1.setZero();
    output.rvec2.setZero();
    output.tvec2.setZero();
    output.reprojection_error1 = std::numeric_limits<float>::infinity();
    output.reprojection_error2 = std::numeric_limits<float>::infinity();
    output.valid = false;

    if (!std::isfinite(squareLength) || squareLength <= 0.0f) {
        return output;
    }
    for (const Eigen::Vector2f& point : imagePoints) {
        if (!point.allFinite()) {
            return output;
        }
    }

    const bool normalizedCoordinates = cameraMatrix.isZero();
    if (!normalizedCoordinates &&
        (!cameraMatrix.allFinite() || !distCoeffs.allFinite() || std::abs(cameraMatrix(0, 0)) < IPPE_SMALL ||
         std::abs(cameraMatrix(1, 1)) < IPPE_SMALL)) {
        return output;
    }

    std::array<Eigen::Vector2f, 4> normalizedInputPoints; // undistorted version of imagePoints
    std::array<Eigen::Vector2f, 4> objectPoints2D;

    // generate the object points:
    objectPoints2D = generateSquareObjectCorners2D(squareLength);

    // handle normalized pixel coordinates vs camera matrix distortion
    if (normalizedCoordinates) {
        // this means imagePoints are defined in normalized pixel coordinates, so just copy it:
        normalizedInputPoints = imagePoints;
    } else {
        // undistort the image points (i.e. put them in normalized pixel coordinates).
        normalizedInputPoints = undistortPoints5Coeffs(imagePoints, cameraMatrix, distCoeffs);
    }

    // compute H directly via return value
    Eigen::Matrix3f H = homographyFromSquarePoints(normalizedInputPoints, squareLength / 2.0f);
    if (!H.allFinite()) {
        return output;
    }

    // now solve using structured bindings for the dual pose solutions
    auto [Ma, Mb] = solveCanonicalForm(objectPoints2D, normalizedInputPoints, H);
    if (!Ma.allFinite() || !Mb.allFinite()) {
        return output;
    }

    // sort poses according to reprojection error:
    std::array<Eigen::Vector3f, 4> objectPoints3D = generateSquareObjectCorners3D(squareLength);

    auto sortedPoses = sortPosesByReprojError(objectPoints3D, imagePoints, cameraMatrix, distCoeffs, Ma, Mb);

    // fill outputs
    output.rvec1 = rot2vec(sortedPoses.M1.block<3, 3>(0, 0));
    output.rvec2 = rot2vec(sortedPoses.M2.block<3, 3>(0, 0));

    output.tvec1 = sortedPoses.M1.block<3, 1>(0, 3);
    output.tvec2 = sortedPoses.M2.block<3, 1>(0, 3);

    output.reprojection_error1 = sortedPoses.err1;
    output.reprojection_error2 = sortedPoses.err2;
    output.valid = std::isfinite(output.reprojection_error1) && std::isfinite(output.reprojection_error2);

    return output;
}

Eigen::Matrix3f rvecToRotMatrix(const Eigen::Vector3f& rvec) {
    float angle = rvec.norm();
    if (angle < 1e-8f) {
        return Eigen::Matrix3f::Identity();
    }
    Eigen::Vector3f axis = rvec / angle;
    return Eigen::AngleAxisf(angle, axis).toRotationMatrix();
}

Pose ApriltagLocalization::estimateRobotPose(const IPPEOutput& ippeOutput, const Pose& odometryPose, int tagId) const {
    if (!ippeOutput.valid) {
        return odometryPose;
    }

    struct Candidate {
        const FieldTag* tag;
        float distanceSquared;
    };

    std::array<Candidate, 4> closestTags{};
    size_t closestTagCount = 0;
    for (const FieldTag& tag : FIELD_TAGS) {
        if (tag.tagId != tagId) {
            continue;
        }

        const float dx = tag.position[0] - odometryPose.x;
        const float dy = tag.position[1] - odometryPose.y;
        const float distanceSquared = dx * dx + dy * dy;
        const Candidate candidate{&tag, distanceSquared};
        if (closestTagCount < closestTags.size()) {
            closestTags[closestTagCount++] = candidate;
            continue;
        }

        size_t farthestIndex = 0;
        for (size_t i = 1; i < closestTags.size(); ++i) {
            if (closestTags[i].distanceSquared > closestTags[farthestIndex].distanceSquared) {
                farthestIndex = i;
            }
        }
        if (distanceSquared < closestTags[farthestIndex].distanceSquared) {
            closestTags[farthestIndex] = candidate;
        }
    }

    if (closestTagCount == 0) {
        return odometryPose;
    }

    struct PoseSolution {
        Eigen::Matrix3f tagFromCamera;
        Eigen::Vector3f tvec;
        bool valid;
    };

    const std::array<Eigen::Vector3f, 2> rvecs{ippeOutput.rvec1, ippeOutput.rvec2};
    const std::array<Eigen::Vector3f, 2> tvecs{ippeOutput.tvec1, ippeOutput.tvec2};
    std::array<PoseSolution, 2> solutions;
    for (size_t i = 0; i < solutions.size(); ++i) {
        solutions[i].valid = rvecs[i].allFinite() && tvecs[i].allFinite();
        solutions[i].tvec = tvecs[i];
        if (solutions[i].valid) {
            solutions[i].tagFromCamera = rvecToRotMatrix(rvecs[i]).transpose();
        }
    }

    const auto wrapAngle = [](float angle) { return std::remainder(angle, 2.0f * PI); };
    const float odometryMathYaw = wrapAngle(PI / 2.0f - odometryPose.theta);
    constexpr float minimumFacingCosine = 0.3420201433f; // sin(20 deg) == cos(70 deg)
    constexpr float minimumFacingCosineSquared = minimumFacingCosine * minimumFacingCosine;

    Pose bestPose = odometryPose;
    float bestScore = std::numeric_limits<float>::infinity();

    for (size_t tagIndex = 0; tagIndex < closestTagCount; ++tagIndex) {
        const FieldTag& tag = *closestTags[tagIndex].tag;
        const float sinYaw = std::sin(tag.yawRad);
        const float cosYaw = std::cos(tag.yawRad);

        const float dx = tag.position[0] - odometryPose.x;
        const float dy = tag.position[1] - odometryPose.y;
        const float distanceSquared = closestTags[tagIndex].distanceSquared;
        const float facingProjection = -dx * cosYaw - dy * sinYaw;
        if (distanceSquared < IPPE_SMALL * IPPE_SMALL || facingProjection <= 0.0f ||
            facingProjection * facingProjection < minimumFacingCosineSquared * distanceSquared) {
            continue;
        }

        // tag frame in field coordinates
        // +x is tag-right, +y is tag-up, and +z is the outward normal
        Eigen::Matrix3f fieldFromTag;
        fieldFromTag << -sinYaw, 0.0f, cosYaw, cosYaw, 0.0f, sinYaw, 0.0f, 1.0f, 0.0f;
        const Eigen::Vector3f tagPosition(tag.position[0], tag.position[1], tag.position[2]);

        for (const PoseSolution& solution : solutions) {
            if (!solution.valid) {
                continue;
            }

            const Eigen::Matrix3f fieldFromCamera = fieldFromTag * solution.tagFromCamera;
            const Eigen::Vector3f cameraForward = fieldFromCamera.col(2);
            const float cameraMathYaw = std::atan2(cameraForward.y(), cameraForward.x());
            const float robotMathYaw = wrapAngle(cameraMathYaw + visionSensorYawOffset);
            const float headingError = wrapAngle(robotMathYaw - odometryMathYaw);
            const float score = std::abs(headingError);
            if (score < bestScore) {
                const Eigen::Vector3f cameraPosition = tagPosition - fieldFromCamera * solution.tvec;
                const float cosRobotYaw = std::cos(robotMathYaw);
                const float sinRobotYaw = std::sin(robotMathYaw);
                const Eigen::Vector2f fieldSensorOffset(
                    cosRobotYaw * visionSensorOffset.x() - sinRobotYaw * visionSensorOffset.y(),
                    sinRobotYaw * visionSensorOffset.x() + cosRobotYaw * visionSensorOffset.y());

                Pose estimatedPose = odometryPose;
                estimatedPose.x = cameraPosition.x() - fieldSensorOffset.x();
                estimatedPose.y = cameraPosition.y() - fieldSensorOffset.y();
                estimatedPose.theta = wrapAngle(PI / 2.0f - robotMathYaw);

                bestScore = score;
                bestPose = estimatedPose;
            }
        }
    }

    return bestPose;
}

void ApriltagLocalization::update(uint32_t period_ms) {
    u_int32_t now = pros::millis();
    visionSensor.enable_detection_types(pros::AivisionModeType::tags, pros::AivisionModeType::colors);
    visionSensor.disable_detection_types(pros::AivisionModeType::objects, pros::AivisionModeType::color_merge);
    visionSensor.set_tag_family(pros::AivisionTagFamily::tag_21H7, true);

    while (true) {
        auto objects = visionSensor.get_all_objects();
        for (const auto& object : objects) {
            if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag))
                continue;
            std::array<Eigen::Vector2f, 4> imagePoints = {
                Eigen::Vector2f(static_cast<float>(object.object.tag.x3), static_cast<float>(object.object.tag.y3)),
                Eigen::Vector2f(static_cast<float>(object.object.tag.x2), static_cast<float>(object.object.tag.y2)),
                Eigen::Vector2f(static_cast<float>(object.object.tag.x1), static_cast<float>(object.object.tag.y1)),
                Eigen::Vector2f(static_cast<float>(object.object.tag.x0), static_cast<float>(object.object.tag.y0))
            };

            auto vecsPair = IPPESquare(imagePoints, cameraMatrix, distCoeffs, APRILTAG_SIZE);
            if (!vecsPair.valid) {
                std::println("[ApriltagLocalization] IPPE failed for a degenerate tag detection.");
                continue;
            }

            poseMutex.take();
            currentPose = estimateRobotPose(vecsPair, odomPoseGetter(true), object.id);
            poseMutex.give();
        }
        if (objects.empty()) {
            std::println("[ApriltagLocalization] No tags detected!");
        }
        pros::Task::delay_until(&now, period_ms);
    }
}

void ApriltagLocalization::startTask(uint32_t period_ms) {
    if (!m_task.has_value()) {
        m_task = pros::Task([this, period_ms] { this->update(period_ms); });
        std::println("[ApriltagLocalization] Tracking task started!");
    } else {
        std::println("[ApriltagLocalization] WARNING: Tried to start tracking task, but it has already been started!");
    }
}

Pose ApriltagLocalization::getPoseFromTag(bool useRadians) {
    poseMutex.take();
    Pose p = currentPose;
    poseMutex.give();
    if (!useRadians) {
        p.theta *= RAD2DEG;
    }
    return p;
}

} // namespace hololib
