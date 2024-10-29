// src/trajectory_manager.cpp

#include "mission_manager/trajectory_manager.hpp"

namespace mission_manager {

Point TrajectoryManager::calculatePerpendicularProjection(const Point& A, const Point& D, const Point& B)
{
    return ::mission_manager::calculatePerpendicularProjection(A, D, B);
}

double TrajectoryManager::calculateTrajectoryDeviation(const Point& projection, const Point& B)
{
    return ::mission_manager::calculateTrajectoryDeviation(projection, B);
}

Point TrajectoryManager::calculateCorrectedEndpoint(const Point& projection, const Point& A, const Point& D, const Point& B)
{
    return ::mission_manager::calculateCorrectedEndpoint(projection, A, D, B);
}

} // namespace mission_manager
