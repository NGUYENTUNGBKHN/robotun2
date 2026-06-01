/**
 * main.cpp
 * 
 * C++ Robot Simulator - Inverted Pendulum on a Wheel
 * Built with GLFW, OpenGL 2.0, Dear ImGui, and ImPlot.
 */

#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <iomanip>

// Constants
const double PI = 3.14159265358979323846;
const double g = 9.81;

// Robot State
double theta = 0.25;       // Pendulum angle (rad) - Initial tilt ~14 degrees
double theta_dot = 0.0;   // Pendulum angular velocity (rad/s)
double phi = 0.0;         // Wheel angle (rad)
double phi_dot = 0.0;     // Wheel angular velocity (rad/s)

// Control Variables
double Kp = 35.0;
double Ki = 0.1;
double Kd = 8.0;
double target_theta = 0.0; // Target balance angle (0.0 = straight up)

// Physics Parameters (Adjustable via UI)
double weight_length = 0.5;   // Length of pendulum rod (meters)
double wheel_radius = 0.15;   // Wheel radius (meters)
double damping = 0.15;        // Joint damping
double motor_tau = 0.04;      // Motor time constant (s)

// Simulation settings
bool is_running = true;
double sim_time = 0.0;
double time_step = 0.001;     // Integration timestep (1ms)
int steps_per_frame = 16;     // 16 steps of 1ms = 16ms per frame (~60 FPS)

// PID Internal State
double integral_error = 0.0;
double prev_error = 0.0;

// Rolling Buffers for plotting
struct RollingBuffer {
    int max_size = 2000;
    std::vector<double> time_data;
    std::vector<double> val_data;

    void add(double t, double v) {
        time_data.push_back(t);
        val_data.push_back(v);
        if (time_data.size() > max_size) {
            time_data.erase(time_data.begin());
            val_data.erase(val_data.begin());
        }
    }

    void clear() {
        time_data.clear();
        val_data.clear();
    }
};

RollingBuffer theta_history;
RollingBuffer position_history;

// Keyboard input callback
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (key == GLFW_KEY_RIGHT) {
            theta_dot += 0.8; // Push pendulum to the right
        }
        if (key == GLFW_KEY_LEFT) {
            theta_dot -= 0.8; // Push pendulum to the left
        }
    }
}

// Reset Simulation helper
void reset_simulation(double init_theta) {
    theta = init_theta;
    theta_dot = 0.0;
    phi = 0.0;
    phi_dot = 0.0;
    integral_error = 0.0;
    prev_error = 0.0;
    sim_time = 0.0;
    theta_history.clear();
    position_history.clear();
}

// Physics Step (Runge-Kutta 4th Order)
void physics_step(double dt) {
    // 1. PID controller to calculate control input u (desired wheel velocity)
    double error = target_theta - theta;
    integral_error += error * dt;
    
    // Anti-windup limit for integral error
    integral_error = std::max(-10.0, std::min(10.0, integral_error));
    
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    
    double u = Kp * error + Ki * integral_error + Kd * derivative;
    u = std::max(-30.0, std::min(30.0, u)); // Saturated speed boundary (rad/s)

    // 2. Dynamics ODE function: dy/dt = f(y)
    // State vector S = [theta, theta_dot, phi, phi_dot]
    auto get_derivatives = [&](double th_s, double th_dot_s, double phi_s, double phi_dot_s) {
        // Motor tracking dynamics (first order tracking)
        double ddphi = (u - phi_dot_s) / motor_tau;
        
        // Acceleration of wheel center
        double a_w = ddphi * wheel_radius;
        
        // Pendulum angular acceleration equation:
        // ddtheta = (g/l) * sin(theta) - (a_w/l) * cos(theta) - damping * theta_dot
        double ddth = (g / weight_length) * sin(th_s) - (a_w / weight_length) * cos(th_s) - damping * th_dot_s;
        
        return std::make_tuple(th_dot_s, ddth, phi_dot_s, ddphi);
    };

    // RK4 integration steps
    // K1
    auto [dth1, ddth1, dphi1, ddphi1] = get_derivatives(theta, theta_dot, phi, phi_dot);

    // K2
    double th_k2 = theta + dth1 * dt * 0.5;
    double th_dot_k2 = theta_dot + ddth1 * dt * 0.5;
    double phi_k2 = phi + dphi1 * dt * 0.5;
    double phi_dot_k2 = phi_dot + ddphi1 * dt * 0.5;
    auto [dth2, ddth2, dphi2, ddphi2] = get_derivatives(th_k2, th_dot_k2, phi_k2, phi_dot_k2);

    // K3
    double th_k3 = theta + dth2 * dt * 0.5;
    double th_dot_k3 = theta_dot + ddth2 * dt * 0.5;
    double phi_k3 = phi + dphi2 * dt * 0.5;
    double phi_dot_k3 = phi_dot + ddphi2 * dt * 0.5;
    auto [dth3, ddth3, dphi3, ddphi3] = get_derivatives(th_k3, th_dot_k3, phi_k3, phi_dot_k3);

    // K4
    double th_k4 = theta + dth3 * dt;
    double th_dot_k4 = theta_dot + ddth3 * dt;
    double phi_k4 = phi + dphi3 * dt;
    double phi_dot_k4 = phi_dot + ddphi3 * dt;
    auto [dth4, ddth4, dphi4, ddphi4] = get_derivatives(th_k4, th_dot_k4, phi_k4, phi_dot_k4);

    // Update state vector
    theta += (dth1 + 2.0 * dth2 + 2.0 * dth3 + dth4) * dt / 6.0;
    theta_dot += (ddth1 + 2.0 * ddth2 + 2.0 * ddth3 + ddth4) * dt / 6.0;
    phi += (dphi1 + 2.0 * dphi2 + 2.0 * dphi3 + dphi4) * dt / 6.0;
    phi_dot += (ddphi1 + 2.0 * ddphi2 + 2.0 * ddphi3 + ddphi4) * dt / 6.0;
}

// Draw a circle using modern legacy OpenGL
void draw_circle(double cx, double cy, double r_val, int segments, bool fill, float r, float g, float b, float alpha = 1.0f) {
    glColor4f(r, g, b, alpha);
    if (fill) {
        glBegin(GL_TRIANGLE_FAN);
        glVertex2d(cx, cy);
        for (int i = 0; i <= segments; ++i) {
            double angle = 2.0 * PI * i / segments;
            glVertex2d(cx + r_val * cos(angle), cy + r_val * sin(angle));
        }
        glEnd();
    } else {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < segments; ++i) {
            double angle = 2.0 * PI * i / segments;
            glVertex2d(cx + r_val * cos(angle), cy + r_val * sin(angle));
        }
        glEnd();
    }
}

// Draw a line
void draw_line(double x1, double y1, double x2, double y2, double width, float r, float g, float b, float alpha = 1.0f) {
    glLineWidth((float)width);
    glColor4f(r, g, b, alpha);
    glBegin(GL_LINES);
    glVertex2d(x1, y1);
    glVertex2d(x2, y2);
    glEnd();
}

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "C++ Inverted Pendulum Robot Simulator", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable v-sync

    // Setup input callback
    glfwSetKeyCallback(window, key_callback);

    // Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

    // Modern clean dark theme
    ImGui::StyleColorsDark();
    
    // Customize UI Style for a premium feel
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.GrabRounding = 4.0f;
    style.WindowBorderSize = 0.0f;

    // Initialize ImGui Backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    // Reset simulator initially
    reset_simulation(0.25);

    // Main Loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // 1. Run physical simulation sub-steps (constant time steps)
        if (is_running) {
            for (int i = 0; i < steps_per_frame; ++i) {
                physics_step(time_step);
                sim_time += time_step;
            }
            // Save current state to histories for real-time graphs
            theta_history.add(sim_time, theta);
            position_history.add(sim_time, phi * wheel_radius);
        }

        // 2. Prepare ImGui frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 3. ImGui UI Control Panel Window
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(380, 390), ImGuiCond_Always);
        ImGui::Begin("Simulator Controls", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::TextColored(ImVec4(0.3f, 0.7f, 1.0f, 1.0f), "Simulation Status");
        ImGui::Checkbox(is_running ? "Running (Physics On)" : "Paused (Physics Off)", &is_running);
        
        static float init_angle_deg = 15.0f;
        ImGui::SliderFloat("Init Tilt (deg)", &init_angle_deg, -90.0f, 90.0f);
        if (ImGui::Button("Reset Simulation", ImVec2(-1, 30))) {
            reset_simulation(init_angle_deg * PI / 180.0);
        }

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.6f, 1.0f), "Live PID Parameter Tuning");
        
        // Double input sliders for precise PID tuning
        float kp_f = (float)Kp;
        float ki_f = (float)Ki;
        float kd_f = (float)Kd;
        ImGui::SliderFloat("Kp (Proportional)", &kp_f, 0.0f, 150.0f, "%.1f");
        ImGui::SliderFloat("Ki (Integral)", &ki_f, 0.0f, 5.0f, "%.2f");
        ImGui::SliderFloat("Kd (Derivative)", &kd_f, 0.0f, 30.0f, "%.1f");
        Kp = kp_f; Ki = ki_f; Kd = kd_f;

        float target_th_deg = (float)(target_theta * 180.0 / PI);
        ImGui::SliderFloat("Target Angle (deg)", &target_th_deg, -20.0f, 20.0f, "%.1f");
        target_theta = target_th_deg * PI / 180.0;

        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "Physical Parameter Configuration");
        float len = (float)weight_length;
        float rad = (float)wheel_radius;
        float damp = (float)damping;
        ImGui::SliderFloat("Rod Length (m)", &len, 0.1f, 1.5f, "%.2f");
        ImGui::SliderFloat("Wheel Radius (m)", &rad, 0.05f, 0.4f, "%.2f");
        ImGui::SliderFloat("Joint Damping", &damp, 0.0f, 1.0f, "%.2f");
        weight_length = len; wheel_radius = rad; damping = damp;

        ImGui::End();

        // 4. ImGui UI Real-time Plotting Window
        ImGui::SetNextWindowPos(ImVec2(10, 410), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(380, 300), ImGuiCond_Always);
        ImGui::Begin("Live Graphs", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        if (ImPlot::BeginPlot("State Trajectories", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, std::max(0.0, sim_time - 5.0), sim_time + 0.1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5);
            
            ImPlot::PlotLine("Theta (rad)", theta_history.time_data.data(), theta_history.val_data.data(), (int)theta_history.time_data.size());
            ImPlot::PlotLine("Wheel X (m)", position_history.time_data.data(), position_history.val_data.data(), (int)position_history.time_data.size());
            
            ImPlot::EndPlot();
        }

        ImGui::End();

        // 5. Help window overlaying instructions
        ImGui::SetNextWindowPos(ImVec2(400, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(350, 75), ImGuiCond_Always);
        ImGui::Begin("Interactive Tips", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
        ImGui::Text("Controls:");
        ImGui::BulletText("Press LEFT/RIGHT Arrow Keys to push the pendulum!");
        ImGui::BulletText("The camera automatically tracks the rolling robot!");
        ImGui::End();

        // 6. OpenGL Rendering (Visualizer Animation)
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        
        // Deep premium space-dark blue background
        glClearColor(0.06f, 0.07f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Calculate geometric offsets
        double wheel_x = phi * wheel_radius;
        double wheel_y = wheel_radius;

        // Wheel spoke rotations (to visually show wheel spinning)
        double spoke1_x = wheel_x + wheel_radius * cos(phi);
        double spoke1_y = wheel_y + wheel_radius * sin(phi);
        double spoke2_x = wheel_x + wheel_radius * cos(phi + PI / 2.0);
        double spoke2_y = wheel_y + wheel_radius * sin(phi + PI / 2.0);
        double spoke3_x = wheel_x + wheel_radius * cos(phi + PI);
        double spoke3_y = wheel_y + wheel_radius * sin(phi + PI);
        double spoke4_x = wheel_x + wheel_radius * cos(phi + 3.0 * PI / 2.0);
        double spoke4_y = wheel_y + wheel_radius * sin(phi + 3.0 * PI / 2.0);

        // Pendulum mass coordinates (inverted trigonometry, matching example.py)
        // theta = 0 means straight up.
        double mass_x = wheel_x + weight_length * cos(theta - PI / 2.0);
        double mass_y = wheel_y - weight_length * sin(theta - PI / 2.0);

        // Setup 2D Orthographic Camera following the wheel
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double aspect = (double)display_w / (double)display_h;
        double half_height = 1.4; // Shows exactly 2.8 meters vertically
        double half_width = half_height * aspect;
        
        // We offset the horizontal center by +0.8 to shift the robot to the right
        // since the ImGui panel is on the left, making the robot perfectly visible!
        double camera_x = wheel_x - 0.8;
        glOrtho(camera_x - half_width, camera_x + half_width, -0.4, 2.4, -1.0, 1.0);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // A. Draw Scrolling Coordinate Grid
        int start_grid_x = (int)floor(camera_x - half_width) - 1;
        int end_grid_x = (int)ceil(camera_x + half_width) + 1;
        
        // Vertical grid lines
        for (int gx = start_grid_x; gx <= end_grid_x; ++gx) {
            draw_line(gx, -0.4, gx, 2.4, 1.0, 0.16f, 0.20f, 0.28f, 0.4f);
        }
        // Horizontal grid lines
        for (double gy = -0.2; gy <= 2.2; gy += 0.5) {
            draw_line(camera_x - half_width, gy, camera_x + half_width, gy, 1.0, 0.16f, 0.20f, 0.28f, 0.4f);
        }

        // B. Draw Solid Ground Line
        draw_line(camera_x - half_width, 0.0, camera_x + half_width, 0.0, 3.0, 0.35f, 0.40f, 0.50f, 1.0f);

        // C. Draw Wheel Spoke Lines (rotating indicators)
        draw_line(wheel_x, wheel_y, spoke1_x, spoke1_y, 2.5, 0.30f, 0.65f, 1.0f, 0.8f);
        draw_line(wheel_x, wheel_y, spoke2_x, spoke2_y, 2.5, 0.30f, 0.65f, 1.0f, 0.8f);
        draw_line(wheel_x, wheel_y, spoke3_x, spoke3_y, 2.5, 0.30f, 0.65f, 1.0f, 0.8f);
        draw_line(wheel_x, wheel_y, spoke4_x, spoke4_y, 2.5, 0.30f, 0.65f, 1.0f, 0.8f);

        // D. Draw Wheel Outer Ring
        draw_circle(wheel_x, wheel_y, wheel_radius, 40, false, 0.30f, 0.65f, 1.0f, 1.0f);

        // E. Draw Pendulum Rod
        draw_line(wheel_x, wheel_y, mass_x, mass_y, 5.0, 0.85f, 0.87f, 0.90f, 1.0f);

        // F. Draw Wheel Center Axis Joint
        draw_circle(wheel_x, wheel_y, 0.02, 12, true, 1.0f, 1.0f, 1.0f, 1.0f);

        // G. Draw Pendulum Mass Ball
        // Glow layer (semi-transparent large circle)
        draw_circle(mass_x, mass_y, 0.09, 20, true, 1.0f, 0.35f, 0.35f, 0.3f);
        // Solid core mass
        draw_circle(mass_x, mass_y, 0.06, 20, true, 1.0f, 0.35f, 0.35f, 1.0f);

        // 7. Render ImGui overlay on top
        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        // Swap buffers
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
