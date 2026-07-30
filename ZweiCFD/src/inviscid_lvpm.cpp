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

// Helper: get freestream velocity vector based on wind direction and AoA
static void getFreestreamVelocity(double V_inf, double alpha_deg, int windDir, double& Vx, double& Vy) {
    double alpha_rad = alpha_deg * (M_PI / 180.0);
    Vx = V_inf * std::cos(alpha_rad);
    Vy = V_inf * std::sin(alpha_rad);

    // Apply wind direction rotation
    switch (windDir) {
        case 1: // From Right -> blows left
            Vx = -Vx;
            Vy = -Vy;
            break;
        case 2: // From Top -> blows downward (in sim coords: -y)
            { double temp = Vx; Vx = Vy; Vy = -temp; }
            break;
        case 3: // From Bottom -> blows upward (in sim coords: +y)
            { double temp = Vx; Vx = -Vy; Vy = temp; }
            break;
        default: // case 0: From Left -> blows right (default)
            break;
    }
}

void InviscidLVPM::calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions) { 
    const auto& panels = targetAirfoil.getPanels(); 
    int N = panels.size(); 
         
    double V_x, V_y;
    getFreestreamVelocity(conditions.V_inf, conditions.alpha, conditions.windDirection, V_x, V_y);

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

                double term1 = std::log(r1 / r2); 

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

Point2D InviscidLVPM::getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const {
    double V_x, V_y;
    getFreestreamVelocity(conditions.V_inf, conditions.alpha, conditions.windDirection, V_x, V_y);

    double U = V_x;
    double W = V_y;

    if (gamma.size() == 0) return {U, W};

    const auto& panels = targetAirfoil.getPanels();
    
    for (size_t j = 0; j < panels.size(); ++j) {
        double xi = pos.x;
        double yi = pos.y;
        double xj = panels[j].p1.x;
        double yj = panels[j].p1.y;
        
        double S = panels[j].length;
        double thetaj = panels[j].theta;
        
        double x = (xi - xj) * std::cos(thetaj) + (yi - yj) * std::sin(thetaj);
        double z = -(xi - xj) * std::sin(thetaj) + (yi - yj) * std::cos(thetaj);
        
        // Avoid singularity very close to panel surface
        double minDist = 1e-4;
        if (std::abs(z) < minDist && x >= 0.0 && x <= S) {
            if (std::abs(z) < 1e-6) continue; // skip exactly on panel
            z = (z < 0) ? -minDist : minDist; // nudge away
        }

        double r1 = std::sqrt(x * x + z * z);
        double r2 = std::sqrt((x - S) * (x - S) + z * z);
        
        double theta1 = std::atan2(z, x);
        double theta2 = std::atan2(z, x - S);
        double d_theta = theta2 - theta1;
        
        while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
        while (d_theta < -M_PI) d_theta += 2.0 * M_PI;
        
        double term1 = std::log(r1 / r2);
        
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
        
        U += U_node1 * gamma(j) + U_node2 * gamma(j + 1);
        W += W_node1 * gamma(j) + W_node2 * gamma(j + 1);
    }
    
    return {U, W};
}

// Ray casting algorithm to check if a point is inside the airfoil polygon
bool InviscidLVPM::isInsideAirfoil(const Point2D& pos) const {
    const auto& coords = targetAirfoil.getCoordinates();
    if (coords.size() < 3) return false;
    
    bool inside = false;
    size_t n = coords.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((coords[i].y > pos.y) != (coords[j].y > pos.y)) &&
            (pos.x < (coords[j].x - coords[i].x) * (pos.y - coords[i].y) / (coords[j].y - coords[i].y) + coords[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

} // namespace zweifoil