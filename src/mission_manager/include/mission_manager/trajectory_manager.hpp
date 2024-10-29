// include/mission_manager/trajectory_manager.hpp

#ifndef MISSION_MANAGER__TRAJECTORY_MANAGER_HPP_
#define MISSION_MANAGER__TRAJECTORY_MANAGER_HPP_

#include "mission_manager/geometry_utils.hpp"

namespace mission_manager {

/**
 * @brief Classe pour gérer les trajectoires et les projections.
 */
class TrajectoryManager {
public:
    TrajectoryManager() = default;

    /**
     * @brief Calcule la projection perpendiculaire.
     * @param A Position du waypoint.
     * @param D Position de départ.
     * @param B Position actuelle.
     * @return Le point de projection.
     */
    Point calculatePerpendicularProjection(const Point& A, const Point& D, const Point& B);

    /**
     * @brief Calcule la déviation de la trajectoire.
     * @param projection Point de projection.
     * @param B Position actuelle.
     * @return La déviation.
     */
    double calculateTrajectoryDeviation(const Point& projection, const Point& B);

    /**
     * @brief Calcule le point final corrigé.
     * @param projection Point de projection.
     * @param A Position du waypoint.
     * @param D Position de départ.
     * @param B Position actuelle.
     * @return Le point final corrigé.
     */
    Point calculateCorrectedEndpoint(const Point& projection, const Point& A, const Point& D, const Point& B);

    // Constantes pour les distances d'accélération et de décélération
    static constexpr double ACCEL_DISTANCE_METERS = 50.0;
    static constexpr double DECEL_DISTANCE_METERS = 50.0;
    static constexpr double ANGULAR_ERROR_THRESHOLD_RAD = 0.5;
};

} // namespace mission_manager

#endif  // MISSION_MANAGER__TRAJECTORY_MANAGER_HPP_
