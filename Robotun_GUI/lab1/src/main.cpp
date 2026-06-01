#pragma comment(linker, "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // GLFW library for windowing and input handling

#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

// Define Pi locally to be perfectly robust
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Error callback to capture and print GLFW issues
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// ============================================================================
// --- STEP 2.1: PHYSICS MODEL DEFINITIONS ---
// ============================================================================

// State vector holding the dynamic coordinates of the system
struct State {
    double dth;   // Angular velocity of the pendulum (rad/s)
    double th;    // Pendulum angle (rad, 0 is the vertical upright position)
    double dphi;  // Angular velocity of the rolling wheel (rad/s)
    double phi;   // Wheel rotation angle (rad)
};

// Physical parameters of the self-balancing system
struct Params {
    double M = 1.5;     // Wheel mass (kg)
    double R = 0.3;     // Wheel radius (m)
    double m = 0.5;     // Pendulum mass (kg)
    double l = 0.8;     // Distance from wheel center to pendulum Center of Mass (m)
    double g = 9.81;    // Gravity acceleration (m/s^2)
    double cw = 0.1;    // Wheel damping/friction coefficient
    double cp = 0.05;   // Pendulum joint damping/friction coefficient
};

// Selection of numerical integration solver
enum class IntegratorType {
    Euler,
    RK4
};

// ============================================================================
// --- STEP 2.2: CUSTOM UI DESIGN SYSTEM ---
// ============================================================================

// Apply a highly polished, modern dark theme with distinct orange-red accents
void ApplyPremiumStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.ScrollbarRounding = 6.0f;
    style.ChildRounding = 6.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                   = ImVec4(0.95f, 0.96f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.11f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.14f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.14f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.25f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.24f, 0.24f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.11f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.14f, 0.14f, 0.17f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.11f, 0.11f, 0.14f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.40f, 0.40f, 0.45f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.50f, 0.50f, 0.55f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(1.00f, 0.43f, 0.15f, 1.00f); // Sleek Accent color
    colors[ImGuiCol_SliderGrab]             = ImVec4(1.00f, 0.43f, 0.15f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]        = ImVec4(1.00f, 0.55f, 0.25f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(1.00f, 0.43f, 0.15f, 0.80f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(1.00f, 0.43f, 0.15f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.85f, 0.33f, 0.08f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(1.00f, 0.43f, 0.15f, 0.20f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(1.00f, 0.43f, 0.15f, 0.40f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(1.00f, 0.43f, 0.15f, 0.60f);
    colors[ImGuiCol_Separator]              = ImVec4(0.25f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.40f, 0.40f, 0.45f, 1.00f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(1.00f, 0.43f, 0.15f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(1.00f, 0.43f, 0.15f, 0.20f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(1.00f, 0.43f, 0.15f, 0.60f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(1.00f, 0.43f, 0.15f, 0.90f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.18f, 0.18f, 0.22f, 0.80f);
    colors[ImGuiCol_TabHovered]             = ImVec4(1.00f, 0.43f, 0.15f, 0.80f);
    colors[ImGuiCol_TabActive]              = ImVec4(1.00f, 0.43f, 0.15f, 1.00f);
}

int main()
{
    // --- STEP 1.1: GLFW LIBRARY INITIALIZATION ---
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // --- STEP 1.2: GRAPHICS WINDOW CREATION ---
    float main_dpi_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    GLFWwindow* window = glfwCreateWindow(
        (int)(1280 * main_dpi_scale), 
        (int)(720 * main_dpi_scale), 
        "Lab 1: Step-by-Step Simulation Tutorial", 
        NULL, NULL
    );
    if (window == NULL)
        return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable V-Sync

    // --- STEP 1.3: DEAR IMGUI CONTEXT SETUP ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_dpi_scale);        
    style.FontScaleDpi = main_dpi_scale;        

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // --- STEP 2.2: APPLY THE HIGH-END CUSTOM STYLE ---
    ApplyPremiumStyle();

    // --- STEP 2.3: SIMULATION INITIAL STATE & PARAMETERS VARIABLES ---
    Params params;
    State state = { 0.0, M_PI / 4.0, 0.0, 0.0 }; // Initial pose: th = 45 degrees (M_PI/4)

    bool paused = true;
    float sim_speed = 1.0f;
    IntegratorType integrator = IntegratorType::RK4;

    bool track_wheel = true;
    bool enable_3d = true;
    float init_theta_deg = 45.0f; // Slider configuration for starting tilt angle

    // Dummy reset lambda function (to be fully integrated with physics later)
    auto reset_sim = [&]() {
        state.dth = 0.0;
        state.th = init_theta_deg * M_PI / 180.0;
        state.dphi = 0.0;
        state.phi = 0.0;
    };

    ImVec4 clear_color = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);

    // --- STEP 1.4: MAIN FRAME LOOP ---
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
            continue;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // --- STEP 2.4: CREATING FULLSCREEN VIEWPORT DASHBOARD ---
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)display_w, (float)display_h));
        
        ImGui::Begin("Dashboard", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_MenuBar);

        // Pre-calculate physical values for layout scopes
        double current_wheel_x = state.phi * params.R;

        // Dashboard Menu Bar
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("Simulation")) {
                if (ImGui::MenuItem("Play/Pause", "Space")) { paused = !paused; }
                if (ImGui::MenuItem("Reset", "R")) { reset_sim(); }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        // --- STEP 2.5: MODERN 2-COLUMN TABLE LAYOUT ---
        if (ImGui::BeginTable("MainColumnsTable", 2, ImGuiTableFlags_Resizable)) {
            // Setup static controls column and expandable canvas/plots column
            ImGui::TableSetupColumn("ControlsColumn", ImGuiTableColumnFlags_WidthFixed, 340.0f * main_dpi_scale);
            ImGui::TableSetupColumn("CanvasColumn", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();

            // ==========================================
            // --- COLUMN 0: CONTROLS & PARAMETERS PANEL ---
            // ==========================================
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("ControlsPanel", ImVec2(0, 0), true);

            ImGui::TextColored(ImVec4(1.00f, 0.43f, 0.15f, 1.00f), "CONTROLS & SETTINGS");
            ImGui::Separator();
            ImGui::Spacing();

            // Playback controls buttons
            if (paused) {
                if (ImGui::Button("PLAY", ImVec2(90, 30))) paused = false;
            } else {
                if (ImGui::Button("PAUSE", ImVec2(90, 30))) paused = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("STEP", ImVec2(75, 30))) {
                // Step physics once (will be integrated with numerical solvers in step 3)
            }
            ImGui::SameLine();
            if (ImGui::Button("RESET", ImVec2(75, 30))) {
                reset_sim();
            }

            ImGui::Spacing();
            ImGui::SliderFloat("Sim Speed", &sim_speed, 0.1f, 5.0f, "%.1fx");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Starting state configurations
            ImGui::TextColored(ImVec4(1.00f, 0.43f, 0.15f, 1.00f), "INITIAL STATE");
            ImGui::Spacing();
            
            if (ImGui::SliderFloat("Init Theta (deg)", &init_theta_deg, -180.0f, 180.0f, "%.1f deg")) {
                if (paused) {
                    state.th = init_theta_deg * M_PI / 180.0;
                }
            }
            if (paused) {
                ImGui::TextColored(ImVec4(0.40f, 0.80f, 0.40f, 1.00f), "Drag slider to rotate starting pose.");
            } else {
                ImGui::TextDisabled("Pause to adjust initial angle.");
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Real-time physical constants sliders (uses SliderScalar for double support)
            ImGui::TextColored(ImVec4(1.00f, 0.43f, 0.15f, 1.00f), "PHYSICAL PARAMETERS");
            ImGui::Spacing();

            double min_M = 0.1, max_M = 10.0;
            ImGui::SliderScalar("Wheel Mass M (kg)", ImGuiDataType_Double, &params.M, &min_M, &max_M, "%.2f");
            
            double min_R = 0.05, max_R = 1.5;
            ImGui::SliderScalar("Wheel Radius R (m)", ImGuiDataType_Double, &params.R, &min_R, &max_R, "%.2f");

            double min_m = 0.05, max_m = 10.0;
            ImGui::SliderScalar("Pend. Mass m (kg)", ImGuiDataType_Double, &params.m, &min_m, &max_m, "%.2f");

            double min_l = 0.1, max_l = 3.0;
            ImGui::SliderScalar("Pend. Length l (m)", ImGuiDataType_Double, &params.l, &min_l, &max_l, "%.2f");

            double min_g = 0.0, max_g = 25.0;
            ImGui::SliderScalar("Gravity g (m/s2)", ImGuiDataType_Double, &params.g, &min_g, &max_g, "%.2f");
            
            ImGui::Spacing();
            ImGui::Text("Friction & Joint Damping");
            double min_cw = 0.0, max_cw = 2.0;
            ImGui::SliderScalar("Wheel Damping", ImGuiDataType_Double, &params.cw, &min_cw, &max_cw, "%.3f");

            double min_cp = 0.0, max_cp = 1.0;
            ImGui::SliderScalar("Pend Damping", ImGuiDataType_Double, &params.cp, &min_cp, &max_cp, "%.3f");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Numerical solvers and interactive camera setups
            ImGui::TextColored(ImVec4(1.00f, 0.43f, 0.15f, 1.00f), "SIMULATOR SETTINGS");
            ImGui::Spacing();

            const char* integrators[] = { "Euler (Fast)", "RK4 (Accurate)" };
            int current_int = (int)integrator;
            if (ImGui::Combo("Integrator", &current_int, integrators, IM_ARRAYSIZE(integrators))) {
                integrator = (IntegratorType)current_int;
            }

            ImGui::Checkbox("Track Wheel (Camera)", &track_wheel);
            ImGui::Checkbox("Enable 3D Mode", &enable_3d);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Live metrics telemetry
            ImGui::Text("LIVE telemetry:");
            if (ImGui::BeginTable("StateTelemetryTable", 2)) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Theta: %.3f rad", state.th);
                ImGui::Text("       %.1f deg", state.th * 180.0 / M_PI);
                
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("dTheta: %.3f r/s", state.dth);
                ImGui::Text("Wheel X: %.3f m", current_wheel_x);
                ImGui::EndTable();
            }

            ImGui::EndChild();

            // ==========================================
            // --- COLUMN 1: CANVAS & PLOTS LAYOUT ---
            // ==========================================
            ImGui::TableSetColumnIndex(1);
            ImVec2 col1_avail = ImGui::GetContentRegionAvail();
            
            // Subdivide Column 1 height: 65% for Canvas, 35% for Plots
            float canvas_h = col1_avail.y * 0.65f;
            float plot_h = col1_avail.y - canvas_h - 10.0f;

            // --- SUB-PANEL 1.1: GRAPHICS CANVAS PLACEHOLDER ---
            ImGui::BeginChild("CanvasPanel", ImVec2(col1_avail.x, canvas_h), true);
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size = ImGui::GetContentRegionAvail();

            // Render simple canvas solid dark background
            draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(20, 20, 24, 255));
            draw_list->AddText(ImVec2(canvas_pos.x + 15, canvas_pos.y + 15), IM_COL32(200, 200, 210, 255), "PHYSICS CANVAS PLACEHOLDER");
            draw_list->AddText(ImVec2(canvas_pos.x + 15, canvas_pos.y + 35), IM_COL32(140, 140, 150, 180), "Graphics rendering pipeline will be integrated here in Step 4.");
            ImGui::EndChild();

            // --- SUB-PANEL 1.2: REAL-TIME PLOTS PLACEHOLDER ---
            ImGui::BeginChild("PlotsPanel", ImVec2(col1_avail.x, plot_h), true);
            ImVec2 plot_pos = ImGui::GetCursorScreenPos();
            draw_list->AddText(ImVec2(plot_pos.x + 15, plot_pos.y + 15), IM_COL32(200, 200, 210, 255), "REAL-TIME PLOTS PLACEHOLDER");
            draw_list->AddText(ImVec2(plot_pos.x + 15, plot_pos.y + 35), IM_COL32(140, 140, 150, 180), "Telemetry graphs (theta and dtheta) will be drawn here in Step 5.");
            ImGui::EndChild();

            ImGui::EndTable();
        }
        ImGui::End();

        // --- STEP 1.5: SYSTEM RENDERING PIPELINE ---
        ImGui::Render();
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // --- STEP 1.6: RESOURCE CLEANUP AND SHUTDOWN ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
