// include/mission_manager/pid_controller.hpp

#ifndef MISSION_MANAGER__PID_CONTROLLER_HPP_
#define MISSION_MANAGER__PID_CONTROLLER_HPP_

namespace mission_manager {

/**
 * @brief Classe pour gérer un contrôleur PID.
 */
class PIDController {
public:
    /**
     * @brief Constructeur de PIDController.
     * @param kp Gain proportionnel.
     * @param ki Gain intégral.
     * @param kd Gain dérivatif.
     */
    PIDController(double kp, double ki, double kd);

    /**
     * @brief Définit les paramètres du PID.
     * @param kp Gain proportionnel.
     * @param ki Gain intégral.
     * @param kd Gain dérivatif.
     */
    void set_parameters(double kp, double ki, double kd);

    /**
     * @brief Calcule la sortie du PID.
     * @param setpoint Consigne désirée.
     * @param measured Valeur mesurée.
     * @param dt Intervalle de temps.
     * @return La sortie calculée par le PID.
     */
    double compute(double setpoint, double measured, double dt);

    /**
     * @brief Réinitialise le contrôleur PID.
     */
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double previous_error_;
    double integral_;
};

} // namespace mission_manager

#endif  // MISSION_MANAGER__PID_CONTROLLER_HPP_
