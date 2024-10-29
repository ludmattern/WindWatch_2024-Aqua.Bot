// include/mission_manager/Point.hpp

#ifndef MISSION_MANAGER__POINT_HPP_
#define MISSION_MANAGER__POINT_HPP_

namespace mission_manager {

/**
 * @brief Structure représentant un point dans l'espace 2D.
 */
struct Point {
    double x;
    double y;
    bool initialized = false;
};

} // namespace mission_manager

#endif  // MISSION_MANAGER__POINT_HPP_
