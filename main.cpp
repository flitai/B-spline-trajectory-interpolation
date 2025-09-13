#include "non_uniform_bspline.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>

// Helper function to write a matrix of points to a CSV file
void write_csv(const std::string& filename, const Eigen::MatrixXd& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Assuming 2D points for simplicity (x, y)
    file << "x,y\n";
    for (int i = 0; i < data.rows(); ++i) {
        file << std::fixed << std::setprecision(4) << data(i, 0) << "," << data(i, 1) << "\n";
    }
    file.close();
}

int main() {
    using namespace fast_planner;

    // 1. Define input conditions
    // Waypoints that the trajectory must pass through
    const std::vector<Eigen::Vector3d> waypoints = {
        {0.0, 0.0, 0.0},
        {10.0, 5.0, 0.0},
        {20.0, -5.0, 0.0},
        {30.0, 8.0, 0.0},
        {40.0, 2.0, 0.0},
        {50.0, 0.0, 0.0}
    };
    
    // Boundary derivatives (start_vel, end_vel, start_acc, end_acc)
    const std::vector<Eigen::Vector3d> derivatives = {
        {5.0, 2.0, 0.0},   // Start velocity
        {5.0, -2.0, 0.0},  // End velocity
        {0.0, 0.0, 0.0},   // Start acceleration
        {0.0, 0.0, 0.0}    // End acceleration
    };

    // Time step for the uniform B-spline parameterization
    const double ts = 0.5;

    // 2. Call the B-spline parameterization function to get control points
    Eigen::MatrixXd control_points_matrix;
    NonUniformBspline::parameterizeToBspline(ts, waypoints, derivatives, control_points_matrix);
    
    std::cout << "Successfully generated " << control_points_matrix.rows() << " control points." << std::endl;

    // 3. Create a B-spline object from the control points (using 2D data for this example)
    const Eigen::MatrixXd control_points_2d = control_points_matrix.block(0, 0, control_points_matrix.rows(), 2);
    
    // Create a 3rd order B-spline
    NonUniformBspline spline(control_points_2d, 3, ts);

    // 4. Sample the trajectory and its derivatives for output
    std::ofstream traj_file("trajectory.csv");
    if (!traj_file.is_open()) {
        std::cerr << "Error: Unable to open trajectory.csv" << std::endl;
        return -1;
    }
    traj_file << "t,px,py,vx,vy,ax,ay\n";

    const auto vel_spline = spline.getDerivative();
    const auto acc_spline = vel_spline.getDerivative();
    
    const double duration = spline.getTimeSum(); // Get total trajectory time
    const int num_samples = 200;

    for (int i = 0; i <= num_samples; ++i) {
        // Use time from 0 to duration for sampling
        const double t = duration * static_cast<double>(i) / num_samples;
        const Eigen::VectorXd pos = spline.evaluateDeBoorT(t);
        const Eigen::VectorXd vel = vel_spline.evaluateDeBoorT(t);
        const Eigen::VectorXd acc = acc_spline.evaluateDeBoorT(t);

        traj_file << std::fixed << std::setprecision(4)
                  << t << ","
                  << pos(0) << "," << pos(1) << ","
                  << vel(0) << "," << vel(1) << ","
                  << acc(0) << "," << acc(1) << "\n";
    }
    traj_file.close();

    // 5. Write waypoints and control points to CSV for visualization
    Eigen::MatrixXd waypoints_matrix(waypoints.size(), 2);
    for (size_t i = 0; i < waypoints.size(); ++i) {
        waypoints_matrix.row(i) = waypoints[i].head<2>();
    }
    
    write_csv("waypoints.csv", waypoints_matrix);
    write_csv("control_points.csv", control_points_2d);

    std::cout << "Generated waypoints.csv, control_points.csv, and trajectory.csv" << std::endl;

    return 0;
}
