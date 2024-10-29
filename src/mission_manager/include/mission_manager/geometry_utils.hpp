// include/mission_manager/geometry_utils.hpp

#ifndef MISSION_MANAGER__GEOMETRY_UTILS_HPP_
#define MISSION_MANAGER__GEOMETRY_UTILS_HPP_

#include "mission_manager/Point.hpp"

namespace mission_manager {

/**
 * @brief Calcule la distance euclidienne entre deux points.
 * @param p1 Premier point.
 * @param p2 Deuxième point.
 * @return La distance euclidienne.
 */
double calculateDistance(const Point& p1, const Point& p2);

/**
 * @brief Calcule le produit vectoriel de trois points.
 * @param D Point de référence.
 * @param A Premier point.
 * @param B Deuxième point.
 * @return Le produit vectoriel.
 */
double crossProduct(const Point& D, const Point& A, const Point& B);

/**
 * @brief Calcule la projection perpendiculaire du point B sur la ligne définie par les points A et D.
 * @param A Point A sur la ligne.
 * @param D Point D sur la ligne.
 * @param B Point à projeter.
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
 * @brief Calcule le point final corrigé en fonction de la projection.
 * @param projection Point de projection.
 * @param A Point A sur la trajectoire.
 * @param D Point de départ.
 * @param B Position actuelle.
 * @return Le point final corrigé.
 */
Point calculateCorrectedEndpoint(const Point& projection, const Point& A, const Point& D, const Point& B);

} // namespace mission_manager

#endif  // MISSION_MANAGER__GEOMETRY_UTILS_HPP_
