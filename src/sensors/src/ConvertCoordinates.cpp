#include <cmath>

void latLonToENU(double latitude, double longitude, double altitude, double& x, double& y, double& z)
{
	double reference_latitude_ = 48.046300;
	double reference_longitude_ = -4.976320;
	double reference_altitude_ = 0; // Altitude de référence en mètres
	double geoid_height_ = 51.7976;     // Hauteur du géoïde à la position de référence
	
    constexpr double a = 6378137.0;                // Demi-grand axe de l'ellipsoïde WGS84 en mètres
    constexpr double f = 1 / 298.257223563;        // Aplatissement de l'ellipsoïde WGS84
    constexpr double e_sq = f * (2 - f);           // Carré de l'excentricité

    // Conversion de la latitude et de la longitude en radians
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    double ref_lat_rad = reference_latitude_ * M_PI / 180.0;
    double ref_lon_rad = reference_longitude_ * M_PI / 180.0;

    // Utiliser l'altitude GPS sans ajustement
    double adjusted_altitude = altitude;

    // Calcul du rayon de courbure en prime verticale
    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    // Conversion du point actuel GPS en ECEF
    double ecef_x = (N + adjusted_altitude) * cos(lat_rad) * cos(lon_rad);
    double ecef_y = (N + adjusted_altitude) * cos(lat_rad) * sin(lon_rad);
    double ecef_z = (N * (1 - e_sq) + adjusted_altitude) * sin(lat_rad);

    // Conversion du point de référence en ECEF
    N = a / sqrt(1 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));
    double ref_ecef_x = (N + reference_altitude_) * cos(ref_lat_rad) * cos(ref_lon_rad);
    double ref_ecef_y = (N + reference_altitude_) * cos(ref_lat_rad) * sin(ref_lon_rad);
    double ref_ecef_z = (N * (1 - e_sq) + reference_altitude_) * sin(ref_lat_rad);

    // Calcul de delta ECEF
    double dx = ecef_x - ref_ecef_x;
    double dy = ecef_y - ref_ecef_y;
    double dz = ecef_z - ref_ecef_z;

    // Conversion de ECEF à ENU
    double sin_lat = sin(ref_lat_rad);
    double cos_lat = cos(ref_lat_rad);
    double sin_lon = sin(ref_lon_rad);
    double cos_lon = cos(ref_lon_rad);

    x = -sin_lon * dx + cos_lon * dy;
    y = -cos_lon * sin_lat * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    //RCLCPP_DEBUG(this->get_logger(), "Converted lat/lon to ENU: (%f, %f, %f) -> (%f, %f, %f)",
                 //latitude, longitude, altitude, x, y, z);
}