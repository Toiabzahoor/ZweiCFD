#include "ZweiFoil/inviscid_lvpm.hpp"
#include "ZweiFoil/solver.hpp"
#include <iostream>
#include <cmath>

namespace zweifoil {

InviscidLVPM::InviscidLVPM(const Airfoil& airfoil) : targetAirfoil(airfoil) {}

Coefficients InviscidLVPM::solve(const Flowconditions& conditions) {
    const auto& panels = targetAirfoil.getPanels();
    int N = panels.size();
    
    if (N == 0) return {0.0, 0.0, 0.0};

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N + 1, N + 1);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(N + 1);

    calculateInfluenceCoefficients(A, b, conditions);

    gamma = A.colPivHouseholderQr().solve(b);

    double total_circulation = 0.0;
    for (int i = 0; i < N; ++i) {
        double gamma_avg = 0.5 * (gamma(i) + gamma(i + 1));
        total_circulation += gamma_avg * panels[i].length;
    }

    double chord = 1.0;
    double cl = (2.0 * total_circulation) / (conditions.V_inf * chord);
    
    double cd = 0.0;     
    double cm = 0.0; 

    return {cl, cd, cm};
}

void InviscidLVPM::calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions) {
    const auto& panels = targetAirfoil.getPanels();
    int N = panels.size();
    
    double alpha_rad = conditions.alpha * (M_PI / 180.0);
    double V_x = conditions.V_inf * std::cos(alpha_rad);
    double V_y = conditions.V_inf * std::sin(alpha_rad);

    for (int i = 0; i < N; ++i) {
        b(i) = -(V_x * panels[i].tangent.x + V_y * panels[i].tangent.y);

        for (int j = 0; j < N; ++j) {
            double inf_node1 = 0.0; 
            double inf_node2 = 0.0;
            
            if (i == j) {
                inf_node1 = -0.25;
                inf_node2 = -0.25;
            } else {
                double xi = panels[i].cp.x;
                double yi = panels[i].cp.y;
                double xj = panels[j].p1.x;
                double yj = panels[j].p1.y;
                
                double S = panels[j].length;
                double thetaj = panels[j].theta;

                double x = (xi - xj) * std::cos(thetaj) + (yi - yj) * std::sin(thetaj);
                double z = -(xi - xj) * std::sin(thetaj) + (yi - yj) * std::cos(thetaj);

                double r1 = std::sqrt(x * x + z * z);
                double r2 = std::sqrt((x - S) * (x - S) + z * z);
                
                double theta1 = std::atan2(z, x);
                double theta2 = std::atan2(z, x - S);
                
                double d_theta = theta2 - theta1;
                
                while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
                while (d_theta < -M_PI) d_theta += 2.0 * M_PI;

                double term1 = 0.5 * std::log(r1 / r2);

                double u1 = (z * term1 + x * d_theta) / (2.0 * M_PI);
                double w1 = (z * d_theta - x * term1 - S) / (2.0 * M_PI);
                
                double u2 = (z * term1 + (x - S) * d_theta) / (2.0 * M_PI);
                double w2 = (z * d_theta - (x - S) * term1 + S) / (2.0 * M_PI);

                double u_node1 = -u1 + u2 * (x / S) + w2 * (z / S);
                double w_node1 = -w1 + u2 * (z / S) - w2 * (x / S);
                
                double u_node2 = -u2 * (x / S) - w2 * (z / S);
                double w_node2 = -u2 * (z / S) + w2 * (x / S);

                double U_node1 = u_node1 * std::cos(thetaj) - w_node1 * std::sin(thetaj);
                double W_node1 = u_node1 * std::sin(thetaj) + w_node1 * std::cos(thetaj);
                
                double U_node2 = u_node2 * std::cos(thetaj) - w_node2 * std::sin(thetaj);
                double W_node2 = u_node2 * std::sin(thetaj) + w_node2 * std::cos(thetaj);

                inf_node1 = U_node1 * panels[i].tangent.x + W_node1 * panels[i].tangent.y;
                inf_node2 = U_node2 * panels[i].tangent.x + W_node2 * panels[i].tangent.y;
            }

            A(i, j) += inf_node1;
            A(i, j + 1) += inf_node2;
        }
    }

    A(N, 0) = 1.0;
    A(N, N) = 1.0;
    b(N) = 0.0;
}

}