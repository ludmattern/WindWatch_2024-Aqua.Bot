// src/geometry_utils.cpp

#include "mission_manager/geometry_utils.hpp"
#include <cmath>

namespace mission_manager {

double calculateDistance(const Point& p1, const Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double crossProduct(const Point& D, const Point& A, const Point& B)
{
    double DAx = A.x - D.x;
    double DAy = A.y - D.y;
    double DBx = B.x - D.x;
    double DBy = B.y - D.y;
    return DAx * DBy - DAy * DBx;
}

Point calculatePerpendicularProjection(const Point& A, const Point& D, const Point& B)
{
    double DAx = A.x - D.x;
    double DAy = A.y - D.y;
    double DBx = B.x - D.x;
    double DBy = B.y - D.y;

    double dotProduct = DAx * DBx + DAy * DBy;
    double lengthSquared = DAx * DAx + DAy * DAy;

    double t = (lengthSquared != 0) ? (dotProduct / lengthSquared) : 0.0;

    Point projection;
    projection.x = D.x + t * DAx;
    projection.y = D.y + t * DAy;

    return projection;
}

double calculateTrajectoryDeviation(const Point& projection, const Point& B)
{
    return calculateDistance(projection, B);
}

Point calculateCorrectedEndpoint(const Point& projection, const Point& A, const Point& D, const Point& B)
{
    double distanceBR = calculateDistance(B, projection);

    double DAx = A.x - D.x;
    double DAy = A.y - D.y;

    double lengthDA = std::sqrt(DAx * DAx + DAy * DAy);

    double orthogonalDx = -DAy / lengthDA;
    double orthogonalDy = DAx / lengthDA;

    double cross = crossProduct(D, A, B);

    if (cross > 0) {
        orthogonalDx = -orthogonalDx;
        orthogonalDy = -orthogonalDy;
    }

    static constexpr double CORRECTION_FACTOR = 3.0;
    Point correctedEndpoint;
    correctedEndpoint.x = A.x + CORRECTION_FACTOR * (orthogonalDx * distanceBR);
    correctedEndpoint.y = A.y + CORRECTION_FACTOR * (orthogonalDy * distanceBR);

    return correctedEndpoint;
}

} // namespace mission_manager
