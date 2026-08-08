#pragma once 
#include "ZweiCFD/solver/airfoil.hpp" 
#include <Eigen/Dense> 
#include "ZweiCFD/solver/flowconditions.hpp"
namespace zweicfd { 

struct Coefficients; 

struct VelocityGrid {
    double minX = -3.0, maxX = 4.0;
    double minY = -3.0, maxY = 3.0;
    int nx = 200, ny = 200;
    double dx = (maxX - minX) / (nx - 1);
    double dy = (maxY - minY) / (ny - 1);
    std::vector<Point2D> grid;

    Point2D interpolate(const Point2D& pos) const {
        if (grid.empty()) return {0.0, 0.0};
        
        double x_idx = (pos.x - minX) / dx;
        double y_idx = (pos.y - minY) / dy;

        int i0 = std::floor(x_idx);
        int j0 = std::floor(y_idx);
        
        if (i0 < 0) i0 = 0; if (i0 >= nx - 1) i0 = nx - 2;
        if (j0 < 0) j0 = 0; if (j0 >= ny - 1) j0 = ny - 2;

        int i1 = i0 + 1;
        int j1 = j0 + 1;

        double tx = x_idx - i0;
        double ty = y_idx - j0;

        const Point2D& v00 = grid[j0 * nx + i0];
        const Point2D& v10 = grid[j0 * nx + i1];
        const Point2D& v01 = grid[j1 * nx + i0];
        const Point2D& v11 = grid[j1 * nx + i1];

        return {
            (1 - tx) * (1 - ty) * v00.x + tx * (1 - ty) * v10.x + (1 - tx) * ty * v01.x + tx * ty * v11.x,
            (1 - tx) * (1 - ty) * v00.y + tx * (1 - ty) * v10.y + (1 - tx) * ty * v01.y + tx * ty * v11.y
        };
    }
};

class InviscidLVPM { 
public: 
    InviscidLVPM(const Airfoil& airfoil); 
    ~InviscidLVPM();
         
    Coefficients solve(const Flowconditions& conditions); 
    const Eigen::VectorXd& getGammaDistribution() const { return gamma; } 

    Point2D getVelocityAt(const Point2D& pos, const Flowconditions& conditions) const;
    Point2D getExactVelocityAt(const Point2D& pos, const Flowconditions& conditions) const;
    
    bool isInsideAirfoil(const Point2D& pos) const;

    VelocityGrid cachedGrid;

private: 
    const Airfoil& targetAirfoil; 
    Eigen::VectorXd gamma; 

    void calculateInfluenceCoefficients(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Flowconditions& conditions); 
    void precomputeVelocityGrid(const Flowconditions& conditions);
};

}  
