#include <tuple>
#include <iostream>
#include <cmath>
#include "raylib.h"
#include <chrono>
#include <thread>
#include <vector>


/*
* =================================================================================================================================================================
*                                      IMPLEMENTATION: PLEASE READ THE FOLLOWING DESCRIPTION CAREFULLY
* =================================================================================================================================================================
* DESCRIPTION:
* This file containts the solution to the problem requested.
* It includes the implementation of a simple bicycle kinematic model and accomodating positional controller for reaching target point from start point.
* The code is written in C++17 standard and uses basic STL libraries for mathematical computations and data handling.
* The code involves a simulation loop to vsiualize the results, please refer to lines 181 ~ 191 in this file for more information
* Documentation, explanation and more Information available and problem definition in the following file:
* https://drive.google.com/file/d/1lNE8f9Vd01Y9ZGF4MjQedWCq2j9y7GMb/view?usp=sharing
* 
* =========================================================================
*/


// Helper functions for the Bicycle Model and Controllers, marked as inline for potential performance benefits.
    inline double wrapAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        
        return angle;
    }

    inline void set_Delta_Limits(double &delta) { // A simple helper function to set limits on values of steering angle delta mimic mechanical steering constraints.
        if (delta > M_PI / 4) delta = M_PI / 4; 
        if (delta < -M_PI / 4) delta = -M_PI / 4;
    }

    enum class ControlMode { // Different control modes that can be implemented for the position controller of the bicycle model.
    PID,
    PURE_PURSUIT
    };

    double dt_global = 0.1; // Global time step variable to be used in controllers when needed.


/**
 * @brief This class implements a simple bicycle kinematic model.
 * 
 * The bicycle model is a simplified representation of a vehicle's motion, 
 * where the vehicle is modeled as a two-wheeled bicycle. This model is commonly 
 * used in robotics and autonomous vehicle simulations to approximate the behavior 
 * of a car-like vehicle.
 */
class BicycleModel {
private:
    double L; // Distance between front and rear axle

public:
    BicycleModel(double L) : L(L) {}

    double getL() const {
        return L;
    }
    std::tuple<double, double, double> update(double x, double y, double theta, double v, double delta , double dt = dt_global) {
       
    	// TODO: implement the kinematics update , Note that delta is the steering angle only, side slip angle beta is not considered.
        double x_dot = v * std::cos(theta); 
        double y_dot = v * std::sin(theta); 
        double theta_dot = (v / L) * std::tan(delta);
    	
        // Calculate new state based on current state and derivatives.
        x += x_dot * dt;
        y += y_dot * dt; 
        theta += theta_dot * dt; 

        theta = wrapAngle(theta); // Ensure theta stays within -pi to pi

        return std::make_tuple(x, y, theta);
    }
};


/**
 * @brief This class implements two types of controller for a bicycle model.
 * 1 - The first controller is a simple PID controller that adjusts the steering angle (delta) based on the angle error between the current heading and the desired heading.
 * 2 - The second controller is a Pure Pursuit controller that calculates the steering angle (delta) needed to reach a target point (x_d, y_d) from the current position (x, y) and orientation (theta).
 * 
 * The choice of controller can be made by setting the ControlMode enum, I wanted to make it extensible for future controllers, to accomodate different driving scenarios.
 * @note The PID controller is a basic implementation and may require tuning of the Kp and Kd parameters for optimal performance.
 * An Idea to make this implementation more modular and OOP friendly would be creating a base controller class and two child controller classes to inherit from it, 
 * But I didn't want to make the single file code to be too cluttered with classes and overloading.
 * 
 */
class PositionController {
private:
    // PID controller gains
    double Kp;
    double Ki;
    double Kd; 
    // Pure Pursuit controller parameters
    double lookahead_distance; // This should be a positive value representing the lookahead distance for the next point to pursue if you have a path of points.
    double dt = dt_global;
    ControlMode mode;
public:
    PositionController(ControlMode mode = ControlMode::PID, double Kp= 0.1, double Kd= 0.0 , double Ki= 0.0, double lookahead= 0.0 , double dt = dt_global)
        : mode(mode), Kp(Kp), Ki(Ki), Kd(Kd), lookahead_distance(lookahead) {}

    ControlMode GetControlMode() const {
        return mode;
    }

    void setControlMode(ControlMode new_mode = ControlMode::PID) {
        std:: cout << "Please note that PID control mode is ON by default." << std::endl;

        if (new_mode != ControlMode::PID && new_mode != ControlMode::PURE_PURSUIT) {
            std::cerr << "Invalid control mode. Defaulting to PID." << std::endl;
            mode = ControlMode::PID;
            return;
        }
        mode = new_mode;
    }

    void setControlGains(double new_Kp, double new_Kd, double new_Ki) {
        Kp = new_Kp;
        Kd = new_Kd;
        Ki = new_Ki;
    }

    void setLookaheadDistance(double new_lookahead) { // For setting up lookahead distance dynamically in case we pass a list pf points to follow.
        lookahead_distance = new_lookahead;
    }

    double compute(double x, double y, double theta, double x_d, double y_d , double L) {
        
        double delta = 0.0;
        static double prev_error = 0.0;
        if (mode == ControlMode::PID)
        {
            double requested_angle = std::atan2(y_d - y, x_d - x);
            double angle_error = wrapAngle(requested_angle - theta);
            double derivative = (angle_error - prev_error) / dt;
            delta = Kp * angle_error + Kd * derivative + Ki * 0.0 ; // Integral term is set to zero for simplicity.
            prev_error = angle_error;
            set_Delta_Limits(delta);
            return delta;    
        }

        // PURE PURSUIT CONTROL
        if(mode == ControlMode::PURE_PURSUIT){
            double dx = x_d - x;
            double dy = y_d - y;
            // Calucalte lookahead distance based on current position to target, and
            //  use this distance to generate delta steering angle that will follow the circular arc to the target point.
            double lookahead_dist_calculated = std::sqrt(dx * dx + dy * dy); 
            

            if (lookahead_dist_calculated < 1e-6) {
                return 0.0; // Already at the target
            }

            double alpha = std::atan2(dy, dx) - theta;
            alpha = wrapAngle(alpha);

            // Uncomment the below line if you want to enforce a fixed lookahead distance, this will apply controller number 3 in the documentation.
            // double Ld = std::min(lookahead_distance, lookahead_dist_calculated); // Lookahead distance should not exceed the distance to the target when we're close to the goal.

            double Ld = lookahead_dist_calculated;

            delta = std::atan2(2 * L * std::sin(alpha), Ld);

            set_Delta_Limits(delta);

            return delta;
        }

        return delta;

    }
};


// uncomment the below line to enable visualization if you're interested in running the visualizations and you have raylib library installed on your native machine.
// #define ENABLE_VISUALIZATION

int main() {
    BicycleModel bicycle(1.0);
    PositionController controller(ControlMode::PURE_PURSUIT , 1.0 , 0.5 , 0.0); // Using PID controller with some gains.

    double x = 12.0, y = 2.0, theta = 30.0;
    double x_d = 1.0, y_d = 6.0;
    double v = 1.0;

    if(x == x_d && y == y_d){
        std::cout << "The vehicle is already at the target position." << std::endl;
        return 0;
    }

    if (controller.GetControlMode() == ControlMode::PID) {
        std::cout << "Using PID Controller" << std::endl;
    } else {
        std::cout << "Using Pure Pursuit Controller" << std::endl;
    }

// Setting up visualization necessary parameters, if the ENABLE_VISUALIZATION flag is enabled.
#ifdef ENABLE_VISUALIZATION
    char* title = "Bicycle Model Visualization - PID Controller";
    if (controller.GetControlMode() == ControlMode::PURE_PURSUIT) {
        title = "Bicycle Model Visualization - Pure Pursuit Controller";
    }

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1200, 1200, title);
    SetTargetFPS(80);
    std::vector<Vector2> path;
#endif
    // Main execution loop
    for (int i = 0; i < 600; i++) {
        double delta = controller.compute(x, y, theta, x_d, y_d , bicycle.getL());
        std::tie(x, y, theta) = bicycle.update(x, y, theta, v, delta);
        std::cout << "Position: (" << x << ", " << y << ", " << theta << ")" << std::endl;

        double dx = x_d - x;
        double dy = y_d - y;

    if (std::sqrt((dx) * (dx) + (dy) * (dy)) <= 0.05) {
            std::cout << " Reached the target After Exactly " << i << " epochs " << std::endl;
            break;
        }
   

/* 

* The following code is responsible for setting up visualization the bicycle model and its movement using the Raylib library. 
   It sets up a window, draws the bicycle as a rectangle, and updates its position based on the kinematic model.

* Note: This visualization code is optional and it depends on the raylib library. The raylib library must be properly installed and
   linked in your development environment to compile and run this code successfully.

If you wish to use the visualization part, make sure to uncomment the relevant sections and do the following:
    1 - install raylib from https://www.raylib.com/ , just download the installer and follow the instructions.
    2 - Link the raylib library in your project settings by running this command :
        g++ File_Name.cpp -o File_Name.exe -I"path_to_Library" -L" path_to_Library_folder" -lraylib -lopengl32 -lgdi32 -lwinmm
    3 - Uncomment the visualization code sections below.
*/
    #ifdef ENABLE_VISUALIZATION

    if(WindowShouldClose()) break;

    BeginDrawing();
    ClearBackground(WHITE);

    Vector2 carPos = { (x*50 + 50), (600 - (y*50 + 50)) }; // Transforming model coordinates to window coordinates , scaling from meters to pixels. 1 meter = 50 pixels.
    Vector2 targetPos = { (x_d*50 + 50),(600 - (y_d*50 + 50)) };

    path.push_back(carPos); // To store the path of the vehicle for past positions visualization.

    DrawCircleV(targetPos, 5, RED); // Draw target point

    DrawCircleV(carPos, 5, BLACK); // Draw bicycle as a circle for simplicity.

    DrawLineEx(carPos, targetPos, 4.0f, BLUE); // Draw line from bicycle to target

    for(auto& point : path){
        DrawCircleV(point, 2, DARKGRAY); // Draw past path
    }
    
    DrawText(TextFormat("x=%.2f y=%.2f theta=%.2f", x, y, theta), 10, 10, 20, BLACK);
    
    DrawText(TextFormat("delta=%.2f", delta), 10, 40, 20, BLACK);
    
    EndDrawing();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Control the speed of visualization

#endif
}

#ifdef ENABLE_VISUALIZATION
    CloseWindow();
#endif

return 0 ;
}
