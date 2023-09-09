
#include <iostream>
#include <cmath>
#include <vector>

// Define a function to simulate a discrete system
std::pair<double, double> simulate_discrete_system(double u1, double u2) {
    // Wheel radius and separation distance
    double r = 0.05;
    double L = 0.2;

    // Define the state-space matrices for differential drive kinematics
    double A[3][3] = {
        {0.0, 0.0, -r / 2 * (u1 + u2) * std::sin(0.0)}, // θ is set to 0 here
        {0.0, 0.0, r / 2 * (u1 + u2) * std::cos(0.0)},
        {0.0, 0.0, 0.0}
    };
    double B[3][2] = {
        {r / 2, r / 2},
        {0.0, 0.0},
        {r / L, -r / L}
    };
    double C[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double D[3][2] = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}
    };

    // Time values for simulation
    double dt = 0.01; // Time step
    int num_samples = 100; // Number of samples

    // Input signal (u1 and u2) for simulation
    std::vector<std::vector<double>> U(num_samples, std::vector<double>(2, 0.0));
    for (int k = 0; k < num_samples; ++k) {
        U[k][0] = u1;
        U[k][1] = u2;
    }

    // Simulate the system
    int num_states = 3;
    double X[num_samples][3] = {0.0};
    double Y[num_samples][3] = {0.0};

    for (int k = 0; k < num_samples; ++k) {
        if (k == 0) {
            // Initial state
            for (int i = 0; i < num_states; ++i) {
                X[k][i] = 0.0;
            }
        } else {
            // Calculate new state
            for (int i = 0; i < num_states; ++i) {
                X[k][i] = 0.0;
                for (int j = 0; j < num_states; ++j) {
                    X[k][i] += A[i][j] * X[k - 1][j];
                }
                for (int j = 0; j < 2; ++j) {
                    X[k][i] += B[i][j] * U[k - 1][j];
                }
            }
        }

        // Calculate system response
        for (int i = 0; i < num_states; ++i) {
            Y[k][i] = 0.0;
            for (int j = 0; j < num_states; ++j) {
                Y[k][i] += C[i][j] * X[k][j];
            }
            for (int j = 0; j < 2; ++j) {
                Y[k][i] += D[i][j] * U[k][j];
            }
        }
    }

    // Extract the target speed and yaw from the output vector
    double target_speed_m = Y[num_samples - 1][0];
    double target_yaw_rad = Y[num_samples - 1][2];

    // Convert target_yaw_rad to target_yaw_deg
    double target_yaw_deg = target_yaw_rad * 180.0 / M_PI;

    return std::make_pair(target_speed_m, target_yaw_deg);
}

int main() {
    double u1 = 1.0;
    double u2 = 2.0;

    // Simulate the system and get the results
    auto result = simulate_discrete_system(u1, u2);

    // Print calculated values
    std::cout << "Calculated Target Speed: " << result.first << std::endl;
    std::cout << "Calculated Target Yaw: " << result.second << std::endl;

    return 0;
}
