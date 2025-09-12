#include "non_uniform_bspline.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>

// Helper function to write a matrix of points to a CSV file
void write_csv(const std::string& filename, const Eigen::MatrixXd& data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // Assuming 2D points for simplicity (x, y)
        file << "x,y\n";
        for (int i = 0; i < data.rows(); ++i) {
            file << std::fixed << std::setprecision(4) << data(i, 0) << "," << data(i, 1) << "\n";
        }
        file.close();
    } else {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
    }
}

int main() {
    using namespace fast_planner;

    // 1. Define input conditions
    // Waypoints that the trajectory must pass through
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back({0.0, 0.0, 0.0});
    waypoints.push_back({10.0, 5.0, 0.0});
    waypoints.push_back({20.0, -5.0, 0.0});
    waypoints.push_back({30.0, 8.0, 0.0});
    waypoints.push_back({40.0, 2.0, 0.0});
    waypoints.push_back({50.0, 0.0, 0.0});
    
    // Boundary derivatives
    std::vector<Eigen::Vector3d> derivatives;
    derivatives.push_back({5.0, 2.0, 0.0});   // Start velocity
    derivatives.push_back({5.0, -2.0, 0.0});  // End velocity
    derivatives.push_back({0.0, 0.0, 0.0});   // Start acceleration
    derivatives.push_back({0.0, 0.0, 0.0});   // End acceleration

    // Time step for the uniform B-spline
    double ts = 0.5;

    // 2. Call the B-spline parameterization function
    Eigen::MatrixXd control_points_matrix;
    NonUniformBspline::parameterizeToBspline(ts, waypoints, derivatives, control_points_matrix);
    
    std::cout << "Successfully generated " << control_points_matrix.rows() << " control points." << std::endl;

    // 3. Create a B-spline object from the control points
    // Note: The original code uses Eigen::MatrixXd. We convert to 2D for this example.
    Eigen::MatrixXd control_points_2d = control_points_matrix.block(0, 0, control_points_matrix.rows(), 2);
    
    NonUniformBspline spline(control_points_2d, 3, ts); // 3rd order B-spline

    // 4. Sample the trajectory and its derivatives
    std::ofstream traj_file("trajectory.csv");
    if (!traj_file.is_open()) {
        std::cerr << "Error: Unable to open trajectory.csv" << std::endl;
        return -1;
    }
    traj_file << "t,px,py,vx,vy,ax,ay\n";

    auto vel_spline = spline.getDerivative();
    auto acc_spline = vel_spline.getDerivative();
    
    double t_start, t_end;
    spline.getTimeSpan(t_start, t_end);

    const int num_samples = 200;
    for (int i = 0; i <= num_samples; ++i) {
        double t = t_start + (t_end - t_start) * i / num_samples;
        Eigen::VectorXd pos = spline.evaluateDeBoor(t);
        Eigen::VectorXd vel = vel_spline.evaluateDeBoor(t);
        Eigen::VectorXd acc = acc_spline.evaluateDeBoor(t);

        traj_file << std::fixed << std::setprecision(4)
                  << t << ","
                  << pos(0) << "," << pos(1) << ","
                  << vel(0) << "," << vel(1) << ","
                  << acc(0) << "," << acc(1) << "\n";
    }
    traj_file.close();

    // 5. Write waypoints and control points to CSV
    Eigen::MatrixXd waypoints_matrix(waypoints.size(), 2);
    for (size_t i = 0; i < waypoints.size(); ++i) {
        waypoints_matrix.row(i) = waypoints[i].head<2>();
    }
    
    write_csv("waypoints.csv", waypoints_matrix);
    write_csv("control_points.csv", control_points_2d);

    std::cout << "Generated waypoints.csv, control_points.csv, and trajectory.csv" << std::endl;

    return 0;
}
