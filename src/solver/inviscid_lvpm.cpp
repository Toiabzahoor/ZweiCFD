#include "ZweiCFD/solver/inviscid_lvpm.hpp" 
#include "ZweiCFD/solver/solver.hpp" 
#include <iostream> 
#include <cmath> 
#include <omp.h>

namespace zweicfd { 

InviscidLVPM::InviscidLVPM(const Airfoil& airfoil) : targetAirfoil(airfoil) {} 

InviscidLVPM::~InviscidLVPM() {
}

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
    
    double cx = 0.0;
    double cy = 0.0;
    double cm_c4 = 0.0;
    
    for (int i = 0; i < N; ++i) {
        double V_t = 0.5 * (gamma(i) + gamma(i + 1));
        double Cp = 1.0 - (V_t * V_t) / (conditions.V_inf * conditions.V_inf);
        
        double dFx = -Cp * panels[i].normal.x * panels[i].length;
        double dFy = -Cp * panels[i].normal.y * panels[i].length;
        
        cx += dFx;
        cy += dFy;
        
        
        double dx = panels[i].cp.x - 0.25;
        double dy = panels[i].cp.y - 0.0;
        cm_c4 += (dFy * dx - dFx * dy);
    }
    
    
    double alpha_rad = conditions.alpha * (M_PI / 180.0);
    double cd = cx * std::cos(alpha_rad) + cy * std::sin(alpha_rad);
    double cl = cy * std::cos(alpha_rad) - cx * std::sin(alpha_rad);
    double cm = cm_c4;

    precomputeVelocityGrid(conditions);
    return {cl, cd, cm}; 
} 

static void getFreestreamVelocity(double V_inf, double alpha_deg, int windDir, double& Vx, double& Vy) {
    double alpha_rad = alpha_deg * (M_PI / 180.0);
    Vx = V_inf * std::cos(alpha_rad);
    Vy = V_inf * std::sin(alpha_rad);

    switch (windDir) {
        case 1: 
            Vx = -Vx;
            Vy = -Vy;
            break;
        case 2: 
            { double temp = Vx; Vx = Vy; Vy = -temp; }
            break;
        case 3: 
            { double temp = Vx; Vx = -Vy; Vy = temp; }
            break;
        default: 
            break;
    }
}

void InviscidLVPM::calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions) { 
    const auto& panels = targetAirfoil.getPanels(); 
    int N = panels.size(); 
         
    double V_x, V_y;
    getFreestreamVelocity(conditions.V_inf, conditions.alpha, conditions.windDirection, V_x, V_y);

    #pragma omp parallel for
    for (int i = 0; i < N; ++i) { 
        b(i) = -(V_x * panels[i].normal.x + V_y * panels[i].normal.y); 

        for (int j = 0; j < N; ++j) { 
            double inf_node1 = 0.0;  
            double inf_node2 = 0.0;  
                         
            if (i == j) { 
                inf_node1 = -1.0 / (2.0 * M_PI);
                inf_node2 =  1.0 / (2.0 * M_PI);
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
                                 
                if (std::isnan(d_theta)) d_theta = 0.0;
                else if (d_theta > M_PI) d_theta -= 2.0 * M_PI; 
                else if (d_theta < -M_PI) d_theta += 2.0 * M_PI; 

                double term1 = std::log(r1 / r2); 

                double u_node1 = ((S - x) * d_theta + z * term1) / (2.0 * M_PI * S);
                double w_node1 = -((S - x) * term1 + S - z * d_theta) / (2.0 * M_PI * S);

                double u_node2 = (x * d_theta - z * term1) / (2.0 * M_PI * S);
                double w_node2 = -(x * term1 - S + z * d_theta) / (2.0 * M_PI * S); 

                double U_node1 = u_node1 * std::cos(thetaj) - w_node1 * std::sin(thetaj); 
                double W_node1 = u_node1 * std::sin(thetaj) + w_node1 * std::cos(thetaj); 
                                 
                double U_node2 = u_node2 * std::cos(thetaj) - w_node2 * std::sin(thetaj); 
                double W_node2 = u_node2 * std::sin(thetaj) + w_node2 * std::cos(thetaj); 

                inf_node1 = U_node1 * panels[i].normal.x + W_node1 * panels[i].normal.y; 
                inf_node2 = U_node2 * panels[i].normal.x + W_node2 * panels[i].normal.y; 
            } 
            A(i, j) += inf_node1; 
            A(i, j + 1) += inf_node2; 
        } 
    } 
    A(N, 0) = 1.0; 
    A(N, N) = 1.0; 
    b(N) = 0.0; 
} 

Point2D InviscidLVPM::getExactVelocityAt(const Point2D& pos, const Flowconditions& conditions) const {
    double V_x, V_y;
    getFreestreamVelocity(conditions.V_inf, conditions.alpha, conditions.windDirection, V_x, V_y);

    double U = V_x;
    double W = V_y;

    if (gamma.size() == 0) return {U, W};

    const auto& panels = targetAirfoil.getPanels();
    
    for (int j = 0; j < static_cast<int>(panels.size()); ++j) {
        double xi = pos.x;
        double yi = pos.y;
        double xj = panels[j].p1.x;
        double yj = panels[j].p1.y;
        
        double S = panels[j].length;
        double thetaj = panels[j].theta;
        
        double x = (xi - xj) * std::cos(thetaj) + (yi - yj) * std::sin(thetaj);
        double z = -(xi - xj) * std::sin(thetaj) + (yi - yj) * std::cos(thetaj);
        
        double minDist = 1e-4;
        if (std::abs(z) < minDist && x >= 0.0 && x <= S) {
            if (std::abs(z) < 1e-6) continue;
            z = (z < 0) ? -minDist : minDist;
        }

        double r1 = std::sqrt(x * x + z * z);
        double r2 = std::sqrt((x - S) * (x - S) + z * z);
        
        double theta1 = std::atan2(z, x);
        double theta2 = std::atan2(z, x - S);
        double d_theta = theta2 - theta1;
        
        if (std::isnan(d_theta)) d_theta = 0.0;
        else if (d_theta > M_PI) d_theta -= 2.0 * M_PI;
        else if (d_theta < -M_PI) d_theta += 2.0 * M_PI;
        
        double term1 = std::log(r1 / r2);
        
        double u_node1 = ((S - x) * d_theta + z * term1) / (2.0 * M_PI * S);
        double w_node1 = -((S - x) * term1 + S - z * d_theta) / (2.0 * M_PI * S);

        double u_node2 = (x * d_theta - z * term1) / (2.0 * M_PI * S);
        double w_node2 = -(x * term1 - S + z * d_theta) / (2.0 * M_PI * S);
        
        double U_node1 = u_node1 * std::cos(thetaj) - w_node1 * std::sin(thetaj);
        double W_node1 = u_node1 * std::sin(thetaj) + w_node1 * std::cos(thetaj);
        double U_node2 = u_node2 * std::cos(thetaj) - w_node2 * std::sin(thetaj);
        double W_node2 = u_node2 * std::sin(thetaj) + w_node2 * std::cos(thetaj);
        
        U += U_node1 * gamma(j) + U_node2 * gamma(j + 1);
        W += W_node1 * gamma(j) + W_node2 * gamma(j + 1);
    }
    
    return {U, W};
}

Point2D InviscidLVPM::getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const {
    if (pos.x >= cachedGrid.minX && pos.x <= cachedGrid.maxX &&
        pos.y >= cachedGrid.minY && pos.y <= cachedGrid.maxY) {
        return cachedGrid.interpolate(pos);
    }
    
    return getExactVelocityAt(pos, conditions);
}

void InviscidLVPM::precomputeVelocityGrid(const Flowconditions& conditions) {
    std::cout << "Precomputing velocity grid (" << cachedGrid.nx << "x" << cachedGrid.ny << ")...\n";
    cachedGrid.grid.resize(cachedGrid.nx * cachedGrid.ny);
    
    #pragma omp parallel for
    for (int j = 0; j < cachedGrid.ny; ++j) {
        double y = cachedGrid.minY + j * cachedGrid.dy;
        for (int i = 0; i < cachedGrid.nx; ++i) {
            double x = cachedGrid.minX + i * cachedGrid.dx;
            cachedGrid.grid[j * cachedGrid.nx + i] = getExactVelocityAt(Point2D{x, y}, conditions);
        }
    }
}


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

} 
