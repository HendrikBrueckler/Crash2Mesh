#include <crash2mesh/viewer/imgui_viewer.hpp>

#include <crash2mesh/io/c2m/c2m_writer.hpp>
#include <crash2mesh/util/par_for.hpp>

#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/random.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/util/file_system.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/drawable_lines.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/manipulated_camera_frame.h>
#include <easy3d/viewer/opengl_text.h>
#include <easy3d/viewer/opengl_timer.h>
#include <easy3d/viewer/setting.h>

#include <easy3d/gui/picker_model.h>
#include <easy3d/util/dialogs.h>

#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/imgui_internal.h>
#include <3rd_party/imgui/impl/imgui_impl_glfw.h>
#include <3rd_party/imgui/impl/imgui_impl_opengl3.h>
#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>

#include <3rd_party/glfw/include/GLFW/glfw3.h>

#include <cmath>
#include <iostream>

namespace c2m
{
using std::map;
using std::vector;

ImGuiContext* ImGuiViewer::context_ = nullptr;

ImGuiViewer::ImGuiViewer(const std::string& title /* = "Easy3D ImGui Viewer" */,
                         int samples /* = 4 */,
                         int gl_major /* = 3 */,
                         int gl_minor /* = 2 */,
                         bool full_screen /* = false */,
                         bool resizable /* = true */,
                         int depth_bits /* = 24 */,
                         int stencil_bits /* = 8 */
                         )
    : AnimationViewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits),
      alpha_(0.8f), movable_(true)
{
    background_color_ = easy3d::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    deciParts.useQuadric = true;
    deciParts.quadricExcludeOnly = false;
    deciParts.framesQuadric = 15;
    deciParts.maxQuadricError = 10;
    deciParts.quadricAreaWeighting = false;
    deciParts.quadricPositionOptimization = false;
    deciParts.quadricPostProcessOptimize = false;
    deciParts.useNormalDeviation = true;
    deciParts.quadricExcludeOnly = false;
    deciParts.framesNormalDeviation = deciParts.framesQuadric;
    deciParts.maxNormalDeviation = 5;
    deciParts.normalExcludeOnly = false;
    deciParts.combineQuadricNormal = true;
    deciParts.useBoundaryDeviation = true;
    deciParts.framesBoundaryDeviation = 5;
    deciParts.maxBoundaryDeviation = 5;
    deciParts.useAspectRatio = true;
    deciParts.maxAspectRatio = 20;
    deciParts.maxVLog = 0;
    deciParts.maxVRender = 0;
    deciParts.minVLog = 1;
    deciParts.minVRender = 1;
}

static inline double get_seconds()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

easy3d::vec3 ImGuiViewer::getStrainColor(float strain)
{
    float t = strain / maxStrain;
    t = std::clamp(t, 0.0f, 1.0f);
    hsv colHSV{t * maxColorHSV.h + (1.0f - t) * minColorHSV.h,
               t * maxColorHSV.s + (1.0f - t) * minColorHSV.s,
               t * maxColorHSV.v + (1.0f - t) * minColorHSV.v};
    rgb colRGB;
    ImGui::ColorConvertHSVtoRGB(colHSV.h, colHSV.s, colHSV.v, colRGB.r, colRGB.g, colRGB.b);
    return easy3d::vec3(colRGB.r, colRGB.g, colRGB.b);
}

void ImGuiViewer::run()
{
    // initialize before showing the window because it can be slow
    init();

    // make sure scene fits the screen when the window appears
    fit_screen();

    // show the window
    glfwShowWindow(window_);

    try
    {
        // Rendering loop
        const int num_extra_frames = 5;
        const double animation_max_fps = 30;
        int frame_counter = 0;

        while (!glfwWindowShouldClose(window_))
        {
            if (!glfwGetWindowAttrib(window_, GLFW_VISIBLE)) // not visible
                continue;

            double tic = get_seconds();
            pre_draw();

            gpu_timer_->start();
            draw();
            gpu_timer_->stop();
            gpu_time_ = gpu_timer_->time();

            post_draw();
            glfwSwapBuffers(window_);

            if (animating || frame_counter++ < num_extra_frames)
            {
                glfwPollEvents();
                // In microseconds
                double duration = 1000000. * (get_seconds() - tic);
                const double min_duration = 1000000. / animation_max_fps;
                if (duration < min_duration)
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(min_duration - duration)));
            }
            else
            {
                /* Wait for mouse/keyboard or empty refresh events */
                glfwWaitEvents();
                frame_counter = 0;
            }
        }

        /* Process events once more */
        glfwPollEvents();
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "Caught exception in main loop: " << e.what();
    }

    cleanup();
}

void ImGuiViewer::init()
{
    AnimationViewer::init();

    if (!context_)
    {
        // Setup ImGui binding
        IMGUI_CHECKVERSION();

        context_ = ImGui::CreateContext();

        const char* glsl_version = "#version 150";
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init(glsl_version);
        ImGuiIO& io = ImGui::GetIO();
        io.WantCaptureKeyboard = true;
        io.WantTextInput = true;
        io.IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;

        // load ui font
        reload_font();
    }
}

double ImGuiViewer::pixel_ratio()
{
    // Computes pixel ratio for hidpi devices
    int fbo_size[2], win_size[2];
    glfwGetFramebufferSize(window_, &fbo_size[0], &fbo_size[1]);
    glfwGetWindowSize(window_, &win_size[0], &win_size[1]);
    return static_cast<double>(fbo_size[0]) / static_cast<double>(win_size[0]);
}

void ImGuiViewer::reload_font(int font_size)
{
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    io.Fonts->AddFontFromMemoryCompressedTTF(
        droid_sans_compressed_data, droid_sans_compressed_size, (float)font_size * dpi_scaling());
    io.FontGlobalScale = 1.0f / pixel_ratio();
    ImGui_ImplOpenGL3_DestroyDeviceObjects();
}

void ImGuiViewer::post_resize(int w, int h)
{
    AnimationViewer::post_resize(w, h);
    if (context_)
    {
        ImGui::GetIO().DisplaySize.x = float(w);
        ImGui::GetIO().DisplaySize.y = float(h);
    }
}

bool ImGuiViewer::callback_event_cursor_pos(double x, double y)
{
    if (ImGui::GetIO().WantCaptureMouse)
        return true;
    else
        return AnimationViewer::callback_event_cursor_pos(x, y);
}

bool ImGuiViewer::callback_event_mouse_button(int button, int action, int modifiers)
{
    if (ImGui::GetIO().WantCaptureMouse)
        return true;
    else
        return AnimationViewer::callback_event_mouse_button(button, action, modifiers);
}

bool ImGuiViewer::callback_event_keyboard(int key, int action, int modifiers)
{
    if (ImGui::GetIO().WantCaptureKeyboard)
        return true;
    else
        return AnimationViewer::callback_event_keyboard(key, action, modifiers);
}

bool ImGuiViewer::callback_event_character(unsigned int codepoint)
{
    if (ImGui::GetIO().WantCaptureKeyboard)
        return true;
    else
        return AnimationViewer::callback_event_character(codepoint);
}

bool ImGuiViewer::callback_event_scroll(double dx, double dy)
{
    if (ImGui::GetIO().WantCaptureMouse)
        return true;
    else
        return AnimationViewer::callback_event_scroll(dx, dy);
}

bool ImGuiViewer::key_press_event(int key, int modifiers)
{
    if (key == GLFW_KEY_LEFT && modifiers == 0)
    {
        currentFrame = currentFrame <= 0 ? (int)visFrames.size() - 1 : currentFrame - 1;
        updateFrame();
    }
    else if (key == GLFW_KEY_RIGHT && modifiers == 0)
    {
        currentFrame = currentFrame >= (int)visFrames.size() - 1 ? 0 : currentFrame + 1;
        updateFrame();
    }
    else if (key == GLFW_KEY_W && modifiers == 0)
    {
        drawWireframes = !drawWireframes;
        updateWireframeVisibility();
    }
    else if (key == GLFW_KEY_V && modifiers == 0)
    {
        drawVertices = !drawVertices;
        updateVertexVisibility();
    }
    else if (key == GLFW_KEY_B && modifiers == 0)
    {
        drawBoundaries = !drawBoundaries;
        updateBoundaryVisibility();
    }
    else if (key == GLFW_KEY_F8 && modifiers == 0)
    {
        drawFaces = !drawFaces;
        updateFaceVisibility();
    }
    else if (key == GLFW_KEY_DELETE && modifiers == 0)
    {
        if (current_model())
            removeCurrentPartAndModel();
    }
    else if (key == GLFW_KEY_DELETE && modifiers == GLFW_MOD_SHIFT)
    {
        if (current_model())
            removeAllElse();
    }
    else
    {
        return AnimationViewer::key_press_event(key, modifiers);
    }
    return false;
}

bool ImGuiViewer::mouse_release_event(int x, int y, int button, int modifiers)
{
    return AnimationViewer::mouse_release_event(x, y, button, modifiers);
}

bool ImGuiViewer::mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers)
{
    if (modifiers != GLFW_MOD_ALT)
    { // GLFW_MOD_ALT is reserved for zoom on region
        switch (button)
        {
        case GLFW_MOUSE_BUTTON_RIGHT:
            camera_->frame()->action_rotate(x, y, dx, dy, camera_, false);
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            camera_->frame()->action_translate(x, y, dx, dy, camera_, false);
            break;
        }
    }
    else
    {
        return AnimationViewer::mouse_drag_event(x, y, dx, dy, button, modifiers);
    }
    return false;
}

bool ImGuiViewer::mouse_press_event(int x, int y, int button, int modifiers)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        easy3d::ModelPicker picker(camera());
        auto model = picker.pick(models(), x, y);
        if (current_model() && dynamic_cast<easy3d::SurfaceMesh*>(current_model()))
        {
            for (auto drawable : current_model()->triangles_drawables())
            {
                drawable->set_per_vertex_color(true);
            }
        }
        else if (current_model() && current_model()->name() == "epicenter")
        {
            for (auto drawable : current_model()->lines_drawables())
            {
                drawable->set_default_color(easy3d::vec3(1.0, 1.0, 1.0) - drawable->default_color());
            }
        }
        if (model && (model != current_model() || modifiers == GLFW_MOD_SHIFT))
        {
            for (auto drawable : model->triangles_drawables())
            {
                if (!strainColors)
                {
                    drawable->set_per_vertex_color(false);
                    drawable->set_default_color(easy3d::vec3(1, 1, 1));
                }
            }
            if (model && model->name() == "epicenter")
            {
                for (auto drawable : model->lines_drawables())
                {
                    drawable->set_default_color(easy3d::vec3(1.0, 1.0, 1.0) - drawable->default_color());
                }
            }
            auto pos = std::find(models_.begin(), models_.end(), model);
            if (pos != models_.end())
            {
                model_idx_ = pos - models_.begin();
            }
        }
        else
        {
            model_idx_ = -1;
            if (targetCam)
            {
                fit_screen();
                targetCam = false;
            }
        }
    }

    return AnimationViewer::mouse_press_event(x, y, button, modifiers);
}

void ImGuiViewer::cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImGui::DestroyContext(context_);

    AnimationViewer::cleanup();
}

void ImGuiViewer::pre_draw()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (animating)
    {
        currentFrame = currentFrame >= (int)visFrames.size() - 1 ? 0 : currentFrame + 1;
        updateFrame();
    }
    AnimationViewer::pre_draw();
}

void ImGuiViewer::draw_overlay(bool* /*visible*/)
{
    drawInfoPanel();
    drawDecimationPanel();
}

void ImGuiViewer::post_draw()
{
    if (show_overlay)
        draw_overlay(&show_overlay);

    static bool show_about = false;
    if (show_about)
    {
        ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::Begin("Crash2Mesh ImGui Viewer", &show_about, ImGuiWindowFlags_NoResize);
        ImGui::Text("Adapted from Easy3D ImGui Viewer by:");
        ImGui::Separator();
        ImGui::Text("\n"
                    "Liangliang Nan\n"
                    "liangliang.nan@gmail.com\n"
                    "https://3d.bk.tudelft.nl/liangliang/\n");
        ImGui::End();
    }

    static bool show_manual = false;
    if (show_manual)
    {
        int w, h;
        glfwGetWindowSize(window_, &w, &h);
        ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
        ImGui::Begin("Easy3D Manual", &show_manual, ImGuiWindowFlags_NoResize);
        ImGui::Text("%s", usage().c_str());
        ImGui::End();
    }

    // ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
    // if (ImGui::BeginMainMenuBar())
    // {
    //     draw_menu_file();

    //     draw_menu_view();

    //     if (ImGui::BeginMenu("Help"))
    //     {
    //         ImGui::MenuItem("Manual", nullptr, &show_manual);
    //         ImGui::Separator();
    //         ImGui::MenuItem("About", nullptr, &show_about);
    //         ImGui::EndMenu();
    //     }
    //     menu_height_ = ImGui::GetWindowHeight();
    //     ImGui::EndMainMenuBar();
    // }
    // ImGui::PopStyleVar();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // draw Easy3D logo
    if (text_renderer_)
    {
        const float font_size = 20.0f;
        const float offsetX = 0.5f * width() - 120.0f * dpi_scaling();
        const float offsetY = 20.0f * dpi_scaling();
        text_renderer_->draw("Crash2Mesh", offsetX, offsetY, font_size, 0);
    }
}

void ImGuiViewer::draw_menu_file()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open", "Ctrl+O"))
            openDialog();
        if (ImGui::MenuItem("Save As...", "Ctrl+S"))
            save();

        // ImGui::Separator();
        // if (ImGui::BeginMenu("Recent Files...")) {
        //	std::string file_name;
        //	std::vector<Model*>::const_reverse_iterator it = models_.rbegin();
        //	for (; it != models_.rend(); ++it) {
        //		if (ImGui::MenuItem((*it)->name().c_str())) {
        //			file_name = (*it)->name();
        //		}
        //	}
        //	if (!file_name.empty())
        //		open(file_name);
        //	ImGui::EndMenu();
        //}

        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4"))
            glfwSetWindowShouldClose(window_, GLFW_TRUE);

        ImGui::EndMenu();
    }
}

void ImGuiViewer::draw_menu_view()
{
    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Snapshot", nullptr))
            snapshot();

        ImGui::Separator();
        if (ImGui::BeginMenu("Options"))
        {
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.50f);

            static int style_idx = 1;
            if (ImGui::Combo("Style", &style_idx, "Classic\0Dark\0Light\0"))
            {
                switch (style_idx)
                {
                case 0:
                    ImGui::StyleColorsClassic();
                    break;
                case 1:
                    ImGui::StyleColorsDark();
                    break;
                case 2:
                    ImGui::StyleColorsLight();
                    break;
                }
            }

            ImGui::Checkbox("Panel Movable", &movable_);
            ImGui::ColorEdit3("Background Color", (float*)background_color_, ImGuiColorEditFlags_NoInputs);
            ImGui::DragFloat("Transparency", &alpha_, 0.005f, 0.0f, 1.0f, "%.1f");
            ImGui::PopItemWidth();
            ImGui::EndMenu();
        }

        ImGui::EndMenu();
    }
}

bool ImGuiViewer::removeCurrentPartAndModel()
{
    if (!current_model())
        return false;

    auto model = current_model();
    std::string name = model->name();
    if (name.substr(0, 4) == "part")
    {
        int userID = atoi(name.substr(4).c_str());
        if (stage == 1)
        {
            for (auto partptr : parts)
            {
                if (partptr->userID == userID)
                {
                    partptr->mesh.clear();
                            // TODO remove
                            partptr->elements1D.clear();
                            partptr->elements2D.clear();
                            partptr->elements3D.clear();
                            partptr->surfaceElements.clear();
                }
            }
        }
    }
    else if (name == "epicenter")
    {
        epicenters.resize(0, 3);
        meanDists.resize(0, 0);
        deciParts.epicenters.resize(0, 3);
        deciParts.meanDistsFromEpicenters.resize(0, 0);
        deciScene.epicenters.resize(0, 3);
        deciScene.meanDistsFromEpicenters.resize(0, 0);
    }
    delete_model(current_model());
    updateGlobalStats();

    model_idx_ = -1;

    return true;
}

bool ImGuiViewer::removeAllElse()
{
    easy3d::Model* curr = current_model();
    if (!curr)
        return false;

    auto models = models_;
    for (auto model : models)
    {
        if (model != curr)
        {
            if (model->name().substr(0, 4) == "part")
            {
                int userID = atoi(model->name().substr(4).c_str());
                if (stage == 1)
                {
                    for (auto partptr : parts)
                    {
                        if (partptr->userID == userID)
                        {
                            partptr->mesh.clear();
                            // TODO remove
                            partptr->elements1D.clear();
                            partptr->elements2D.clear();
                            partptr->elements3D.clear();
                            partptr->surfaceElements.clear();
                        }
                    }
                }
            }
            delete_model(model);
        }
    }
    model_idx_ = 0;
    updateGlobalStats();
    return true;
}

bool ImGuiViewer::updateWireframeVisibility()
{
    for (auto model : models_)
    {
        easy3d::SurfaceMesh* m = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (m)
        {
            easy3d::LinesDrawable* wireframe = m->lines_drawable("wireframe");
            if (!wireframe)
            {
                wireframe = m->add_lines_drawable("wireframe");
                std::vector<unsigned int> indices;
                for (auto e : m->edges())
                {
                    easy3d::SurfaceMesh::Vertex s = m->vertex(e, 0);
                    easy3d::SurfaceMesh::Vertex t = m->vertex(e, 1);
                    indices.push_back(s.idx());
                    indices.push_back(t.idx());
                }
                auto points = m->get_vertex_property<easy3d::vec3>("v:point");
                wireframe->update_vertex_buffer(points.vector());
                wireframe->update_index_buffer(indices);
                wireframe->set_default_color(easy3d::vec3(0.0f, 0.0f, 0.0f));
                wireframe->set_per_vertex_color(false);
                wireframe->set_visible(true);
            }
            wireframe->set_visible(drawWireframes);
        }
    }
    return true;
}

bool ImGuiViewer::updateBoundaryVisibility()
{
    for (auto model : models_)
    {
        easy3d::SurfaceMesh* mesh = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (mesh)
        {
            auto drawable = mesh->lines_drawable("borders");
            if (!drawable)
            {
                auto prop = mesh->get_vertex_property<easy3d::vec3>("v:point");
                std::vector<easy3d::vec3> points;
                for (auto e : mesh->edges())
                {
                    if (mesh->is_boundary(e))
                    {
                        points.push_back(prop[mesh->vertex(e, 0)]);
                        points.push_back(prop[mesh->vertex(e, 1)]);
                    }
                }
                if (!points.empty())
                {
                    drawable = mesh->add_lines_drawable("borders");
                    drawable->update_vertex_buffer(points);
                    drawable->set_default_color(easy3d::setting::surface_mesh_borders_color);
                    drawable->set_per_vertex_color(false);
                    drawable->set_impostor_type(easy3d::LinesDrawable::CYLINDER);
                    drawable->set_line_width(easy3d::setting::surface_mesh_borders_line_width * 3.0f);
                }
            }
            if (drawable)
                drawable->set_visible(drawBoundaries);
        }
    }
    return true;
}

bool ImGuiViewer::updateVertexVisibility()
{
    for (auto model : models_)
        for (auto drawable : model->points_drawables())
            drawable->set_visible(drawVertices);
    return true;
}

bool ImGuiViewer::updateFaceVisibility()
{
    for (auto model : models_)
        for (auto drawable : model->triangles_drawables())
            drawable->set_visible(drawFaces);
    return true;
}

bool ImGuiViewer::openDialog()
{
    const std::string& title = "Please choose a file";
    const std::string& defaultPath = "./";
    const std::vector<std::string>& filters
        = {"ESI PAM-CRASH (VPS) files (*.erfh5)", "*.erfh5", "All Files (*.*)", "*"};
    const std::vector<std::string> fileNames = easy3d::dialog::open(title, defaultPath, filters, false);

    if (!fileNames.empty())
        return openFile(fileNames[0]);
    else
        return false;
}

bool ImGuiViewer::openFile(const std::string& fileName_)
{
    if (fileName_ == "")
        return false;

    stage = 0;

    fileName = fileName_;
    erfh5::Reader reader(fileName, 150);

    numFrames = std::max(1, (int)reader.getNumStates());

    nVisFrames = 1;
    visFrames = {0};
    currentFrame = 0;

    modelToFrameToVertexbuffer.clear();
    modelToFrameToColorbuffer.clear();
    parts.clear();
    scene.reset();

    if (!reader.readParts(parts))
    {
        Logger::lout(Logger::ERROR) << "Could not read file" << std::endl;
        fileName = "";
        return false;
    }
    if (!SurfaceExtractor::extract(parts, true))
    {
        Logger::lout(Logger::ERROR) << "Could not extract surface elements" << std::endl;
        fileName = "";
        return false;
    }
    fullReload = false;

    buildParts();
    MeshAnalyzer::calcPartCenters(parts);

    fit_screen();
    return true;
}

bool ImGuiViewer::buildParts()
{
    epicenters.resize(0, 3);
    meanDists.resize(0, 0);
    deciParts.epicenters.resize(0, 3);
    deciParts.meanDistsFromEpicenters.resize(0, 0);
    deciScene.epicenters.resize(0, 3);
    deciScene.meanDistsFromEpicenters.resize(0, 0);
    if (!MeshBuilder::build(parts, false))
    {
        Logger::lout(Logger::ERROR) << "Mesh building failed!" << std::endl;
        fileName = "";
        return false;
    }

    updateMaxStrains();

    if (createDrawableParts())
        stage = 1;

    updateGlobalStats();

    return true;
}

bool ImGuiViewer::createDrawableParts()
{
    for (auto model : models_)
        delete model;
    models_.clear();
    modelToFrameToVertexbuffer.clear();
    modelToFrameToColorbuffer.clear();

    int preModelIdx = model_idx_;

    std::vector<easy3d::SurfaceMesh*> drawablesMeshes(parts.size());
    std::vector<easy3d::PointsDrawable*> drawablesPoints(parts.size());
    std::vector<easy3d::TrianglesDrawable*> drawablesTriangles(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawablePointsVB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawablePointsCB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawableTrianglesVB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawableTrianglesCB(parts.size());
    std::vector<std::vector<uint>> drawableTrianglesIB(parts.size());

    for (uint index = 0; index < parts.size(); index++)
    {
        Part::Ptr partPtr = parts[index];
        const CMesh& mesh = partPtr->mesh;

        if (mesh.n_faces() == 0)
            continue;

        // Create a drawable mesh
        drawablesMeshes[index] = new easy3d::SurfaceMesh;
        drawablesMeshes[index]->set_name("part " + std::to_string(partPtr->userID));
        drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:marking");
        for (uint i = 0; i < visFrames.size(); i++)
        {
            drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
        }
        modelToFrameToVertexbuffer[drawablesMeshes[index]] = std::vector<std::vector<easy3d::vec3>>(numFrames);
        modelToFrameToColorbuffer[drawablesMeshes[index]] = std::vector<std::vector<easy3d::vec3>>(numFrames);
        // Visualize points as billboards
        drawablesPoints[index] = drawablesMeshes[index]->add_points_drawable("vertices");
        drawablesPoints[index]->set_point_size(5);
        drawablesPoints[index]->set_per_vertex_color(true);
        drawablesPoints[index]->set_visible(false);
        // Visualize triangles with face normals
        drawablesTriangles[index] = drawablesMeshes[index]->add_triangles_drawable("faces");
        drawablesTriangles[index]->set_distinct_back_color(false);
        drawablesTriangles[index]->set_lighting_two_sides(true);
        drawablesTriangles[index]->set_smooth_shading(false);
        drawablesTriangles[index]->set_per_vertex_color(true);
        easy3d::Material m;
        m.specular = easy3d::vec3(0.0f, 0.0f, 0.0f);
        drawablesTriangles[index]->set_material(m);
    }

    auto buildDrawable = [&](size_t index) {
        Part::Ptr partPtr = parts[index];
        const CMesh& mesh = partPtr->mesh;
        if (mesh.n_faces() == 0)
            return;

        easy3d::SurfaceMesh* drawableMesh = drawablesMeshes[index];
        easy3d::SurfaceMesh::VertexProperty colors = drawableMesh->get_vertex_property<easy3d::vec3>("v:marking");

        // Copy over vertices and their properties to drawable model
        vector<easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex(mesh.n_vertices());
        for (VHandle v : mesh.vertices())
        {
            OMVec3 position = mesh.point(v);
            vertexToDrawableVertex[v.idx()]
                = drawableMesh->add_vertex(easy3d::vec3(position[0], position[1], position[2]));
            const MatX3& positions = mesh.data(v).node->positions;
            for (uint i = 0; i < visFrames.size(); i++)
            {
                auto drawablePositions
                    = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
                easy3d::vec3 pos = easy3d::vec3(positions.coeff(visFrames[i], 0),
                                                positions.coeff(visFrames[i], 1),
                                                positions.coeff(visFrames[i], 2));
                drawablePositions[vertexToDrawableVertex[v.idx()]] = pos;
            }
            if (mesh.status(v).fixed_nonmanifold())
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.8f, 0.0f, 0.0f);
            else if (mesh.data(v).node->referencingParts > 1)
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.9f, 0.6f, 0.0f);
            else if (mesh.is_boundary(v))
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.8f, 0.8f, 0.0f);
            else
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.0f, 0.6f, 0.0f);
        }

        auto currentPositions
            = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[currentFrame]));
        auto pos = drawableMesh->get_vertex_property<easy3d::vec3>("v:point");
        pos.vector() = currentPositions.vector();

        // Visualize points as billboards
        drawablePointsVB[index] = drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector();
        drawablePointsCB[index] = drawableMesh->get_vertex_property<easy3d::vec3>("v:marking").vector();

        // Copy over faces and their properties to drawable models
        std::vector<std::vector<easy3d::vec3>>& frameToVertexbuffer = modelToFrameToVertexbuffer[drawableMesh];
        std::vector<std::vector<easy3d::vec3>>& frameToColorbuffer = modelToFrameToColorbuffer[drawableMesh];
        for (FHandle f : mesh.faces())
        {
            auto strains = mesh.data(f).element->plasticStrains;
            vector<easy3d::SurfaceMesh::Vertex> vs;
            for (VHandle v : mesh.fv_range(f))
            {
                auto drawableVertex = vertexToDrawableVertex[v.idx()];
                vs.emplace_back(drawableVertex);
                for (uint i = 0; i < visFrames.size(); i++)
                {
                    auto drawablePositions
                        = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
                    easy3d::vec3 col = getStrainColor(strains(visFrames[i]));
                    frameToColorbuffer[visFrames[i]].emplace_back(col);
                    frameToVertexbuffer[visFrames[i]].emplace_back(drawablePositions[drawableVertex]);
                }
            }
            drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
        }

        vector<easy3d::vec3>& points = drawableTrianglesVB[index];
        vector<easy3d::vec3>& faceColors = drawableTrianglesCB[index];
        vector<uint>& indices = drawableTrianglesIB[index];
        if (strainColors)
        {
            points = frameToVertexbuffer[visFrames[currentFrame]];
            faceColors = frameToColorbuffer[visFrames[currentFrame]];
            indices = std::vector<uint>(3 * drawableMesh->faces_size());
            std::iota(indices.begin(), indices.end(), 0);
        }
        else
        {
            points = drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector();
            std::srand(partPtr->userID);
            faceColors = std::vector<easy3d::vec3>(drawableMesh->n_vertices(), easy3d::random_color());

            for (auto f : drawableMesh->faces())
            {
                easy3d::SurfaceMesh::Halfedge he = drawableMesh->halfedge(f);
                for (int j = 0; j < 3; j++)
                {
                    indices.emplace_back(drawableMesh->to_vertex(he).idx());
                    he = drawableMesh->next_halfedge(he);
                }
            }
        }
    };

    std::vector<size_t> partIndices(parts.size());
    std::iota(partIndices.begin(), partIndices.end(), 0);

    parallel_for_each(partIndices, buildDrawable);

    for (uint index = 0; index < parts.size(); index++)
    {
        easy3d::SurfaceMesh* drawableMesh = drawablesMeshes[index];
        if (!drawableMesh)
            continue;

        easy3d::PointsDrawable* drawablePoints = drawablesPoints[index];
        drawablePoints->update_vertex_buffer(drawablePointsVB[index]);
        drawablePoints->update_color_buffer(drawablePointsCB[index]);
        easy3d::TrianglesDrawable* drawableTriangles = drawablesTriangles[index];
        drawableTriangles->update_vertex_buffer(drawableTrianglesVB[index]);
        drawableTriangles->update_color_buffer(drawableTrianglesCB[index]);
        drawableTriangles->update_index_buffer(drawableTrianglesIB[index]);

        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
        add_model(drawableMesh, false);
        std::cout.clear();
        std::cerr.clear();
    }

    updateFaceVisibility();
    updateVertexVisibility();
    updateBoundaryVisibility();
    updateWireframeVisibility();

    model_idx_ = preModelIdx;
    if (partsExpanded)
    {
        partsExpanded = false;
        toggleExpandParts();
    }
    if (epicenters.rows() > 0)
    {
        createDrawableEpicenters();
    }

    return true;
}

bool ImGuiViewer::toggleStrainColors()
{
    strainColors = !strainColors;

    for (auto model : models_)
    {
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (!surface)
            continue;
        std::string name = surface->name();
        if (name.substr(0, 4) != "part")
            continue;

        auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");

        if (partsExpanded)
        {
            if (strainColors)
            {
                vector<easy3d::vec3> points;
                for (easy3d::SurfaceMesh::Face f : surface->faces())
                {
                    easy3d::SurfaceMesh::Halfedge he = surface->halfedge(f);
                    for (int j = 0; j < 3; j++)
                    {
                        points.emplace_back(pos[surface->to_vertex(he)]);
                        he = surface->next_halfedge(he);
                    }
                }
            }
        }

        for (auto drawableTriangles : surface->triangles_drawables())
        {
            if (strainColors)
            {
                drawableTriangles->update_color_buffer(modelToFrameToColorbuffer[surface][visFrames[currentFrame]]);
                if (partsExpanded)
                {
                    vector<easy3d::vec3> points;
                    for (easy3d::SurfaceMesh::Face f : surface->faces())
                    {
                        easy3d::SurfaceMesh::Halfedge he = surface->halfedge(f);
                        for (int j = 0; j < 3; j++)
                        {
                            points.emplace_back(pos[surface->to_vertex(he)]);
                            he = surface->next_halfedge(he);
                        }
                    }
                    drawableTriangles->update_vertex_buffer(points);
                }
                else
                    drawableTriangles->update_vertex_buffer(
                        modelToFrameToVertexbuffer[surface][visFrames[currentFrame]]);
                std::vector<uint> indices = std::vector<uint>(3 * surface->faces_size());
                std::iota(indices.begin(), indices.end(), 0);
                drawableTriangles->update_index_buffer(indices);
            }
            else
            {
                int userID = atoi(name.substr(4).c_str());
                std::srand(userID);
                drawableTriangles->update_color_buffer(
                    std::vector<easy3d::vec3>(surface->n_vertices(), easy3d::random_color()));
                drawableTriangles->update_vertex_buffer(pos.vector());
                std::vector<uint> indices;
                for (auto f : surface->faces())
                {
                    easy3d::SurfaceMesh::Halfedge he = surface->halfedge(f);
                    for (int j = 0; j < 3; j++)
                    {
                        indices.emplace_back(surface->to_vertex(he).idx());
                        he = surface->next_halfedge(he);
                    }
                }
                drawableTriangles->update_index_buffer(indices);
            }
        }
    }

    return true;
}

bool ImGuiViewer::updateFrame()
{
    for (auto model : models_)
    {
        if (model->name() == "epicenter")
        {
            auto graph = dynamic_cast<easy3d::Graph*>(model);
            auto drawablePositions
                = graph->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[currentFrame]));

            auto pos = graph->get_vertex_property<easy3d::vec3>("v:point");
            pos.vector() = drawablePositions.vector();
            graph->points_drawable("vertices")->update_vertex_buffer(pos.vector());
            graph->lines_drawable("edges")->update_vertex_buffer(pos.vector());
            continue;
        }
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (!surface)
            continue;

        easy3d::SurfaceMesh::VertexProperty drawablePositions
            = surface->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[currentFrame]));

        auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");
        pos.vector() = drawablePositions.vector();

        if (partsExpanded)
        {
            partid_t userID = -1;
            if (model->name().substr(0, 4) == "part")
                userID = atoi(model->name().substr(4).c_str());

            Part::Ptr part;
            for (auto partPtr : parts)
            {
                if (partPtr->userID == userID)
                {
                    part = partPtr;
                    break;
                }
            }
            if (part)
            {
                Vec3 c = part->centers.row(visFrames[currentFrame]).transpose();
                easy3d::vec3 center(c(0), c(1), c(2));
                for (auto v : surface->vertices())
                {
                    pos[v] += (1.3f) * center;
                }
            }
        }

        for (auto drawableTriangles : surface->triangles_drawables())
        {
            if (strainColors && surface->name() != "epicenter")
            {
                if (partsExpanded)
                {
                    vector<easy3d::vec3> points;
                    for (easy3d::SurfaceMesh::Face f : surface->faces())
                    {
                        easy3d::SurfaceMesh::Halfedge he = surface->halfedge(f);
                        for (int j = 0; j < 3; j++)
                        {
                            points.emplace_back(pos[surface->to_vertex(he)]);
                            he = surface->next_halfedge(he);
                        }
                    }
                    drawableTriangles->update_vertex_buffer(points);
                }
                else
                {
                    drawableTriangles->update_vertex_buffer(
                        modelToFrameToVertexbuffer[surface][visFrames[currentFrame]]);
                }
                drawableTriangles->update_color_buffer(modelToFrameToColorbuffer[surface][visFrames[currentFrame]]);
            }
            else
                drawableTriangles->update_vertex_buffer(pos.vector());
        }
        for (auto drawablePoints : surface->points_drawables())
        {
            drawablePoints->update_vertex_buffer(pos.vector());
        }
        auto drawable = surface->lines_drawable("borders");
        if (drawable)
        {
            std::vector<easy3d::vec3> borderPoints;
            for (auto e : surface->edges())
            {
                if (surface->is_boundary(e))
                {
                    borderPoints.push_back(pos[surface->vertex(e, 0)]);
                    borderPoints.push_back(pos[surface->vertex(e, 1)]);
                }
            }
            if (!borderPoints.empty())
            {
                drawable->update_vertex_buffer(borderPoints);
            }
        }
        for (auto drawableLines : surface->lines_drawables())
        {
            if (drawableLines != surface->lines_drawable("borders"))
            {
                drawableLines->update_vertex_buffer(pos.vector());
            }
        }
    }

    if (current_model())
    {
        fit_screen(current_model());
        targetCam = true;
    }
    else
    {
        if (targetCam)
        {
            fit_screen();
            targetCam = false;
        }
    }

    return true;
}

bool ImGuiViewer::calcEpicenters()
{
    MeshAnalyzer::getEpicenter(parts, epicenters, meanDists);
    deciParts.epicenters = deciScene.epicenters = epicenters;
    deciParts.meanDistsFromEpicenters = deciScene.meanDistsFromEpicenters = meanDists;

    createDrawableEpicenters();

    return true;
}

bool ImGuiViewer::toggleExpandParts()
{
    if (models_.empty())
        return false;

    for (auto model : models_)
    {
        if (model->name() == "epicenter")
            model->set_visible(partsExpanded);

        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (surface)
        {
            partid_t userID = -1;
            if (surface->name().substr(0, 4) == "part")
                userID = atoi(surface->name().substr(4).c_str());

            Part::Ptr part;
            for (auto partPtr : parts)
            {
                if (partPtr->userID == userID)
                {
                    part = partPtr;
                    break;
                }
            }
            auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");

            if (part)
            {
                Vec3 c = part->centers.row(visFrames[currentFrame]).transpose();
                easy3d::vec3 center(c(0), c(1), c(2));

                for (auto v : surface->vertices())
                {
                    if (partsExpanded)
                    {
                        pos[v] -= 1.3f * center;
                    }
                    else
                    {
                        pos[v] += 1.3f * center;
                    }
                }
            }
            vector<easy3d::vec3> points;
            if (strainColors)
            {
                for (easy3d::SurfaceMesh::Face f : surface->faces())
                {
                    easy3d::SurfaceMesh::Halfedge he = surface->halfedge(f);
                    for (int j = 0; j < 3; j++)
                    {
                        points.emplace_back(pos[surface->to_vertex(he)]);
                        he = surface->next_halfedge(he);
                    }
                }
            }
            for (auto drawableTriangles : surface->triangles_drawables())
            {
                if (strainColors)
                    drawableTriangles->update_vertex_buffer(points);
                else
                    drawableTriangles->update_vertex_buffer(pos.vector());
            }
            for (auto drawablePoints : surface->points_drawables())
            {
                drawablePoints->update_vertex_buffer(pos.vector());
            }
            auto drawable = surface->lines_drawable("borders");
            if (drawable)
            {
                std::vector<easy3d::vec3> borderPoints;
                for (auto e : surface->edges())
                {
                    if (surface->is_boundary(e))
                    {
                        borderPoints.push_back(pos[surface->vertex(e, 0)]);
                        borderPoints.push_back(pos[surface->vertex(e, 1)]);
                    }
                }
                if (!borderPoints.empty())
                {
                    drawable->update_vertex_buffer(borderPoints);
                }
            }
            for (auto drawableLines : surface->lines_drawables())
            {
                if (drawableLines != surface->lines_drawable("borders"))
                {
                    drawableLines->update_vertex_buffer(pos.vector());
                }
            }
        }
    }
    partsExpanded = !partsExpanded;

    return partsExpanded;
}

bool ImGuiViewer::decimatePartwise()
{
    if (deciParts.quadricPositionOptimization)
    {
        fullReload = true;
    }
    if (!deciParts.decimateParts(parts))
    {
        Logger::lout(Logger::ERROR) << "Decimation failed" << std::endl;
        return false;
    }

    updateMaxStrains();
    createDrawableParts();
    updateGlobalStats();

    return true;
}

bool ImGuiViewer::mergeParts()
{
    scene = MeshBuilder::merge(parts, false);
    if (!scene)
    {
        Logger::lout(Logger::ERROR) << "Merging parts failed" << std::endl;
        return false;
    }

    updateMaxStrains();
    if (!createDrawableScene())
        return false;

    stage = 2;
    updateGlobalStats();
    return true;
}

bool ImGuiViewer::createDrawableScene()
{
    for (auto model : models_)
        delete model;
    models_.clear();
    modelToFrameToVertexbuffer.clear();
    modelToFrameToColorbuffer.clear();

    int preModelIdx = model_idx_;

    std::vector<easy3d::SurfaceMesh*> drawablesMeshes(parts.size());
    std::vector<easy3d::PointsDrawable*> drawablesPoints(parts.size());
    std::vector<easy3d::TrianglesDrawable*> drawablesTriangles(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawablePointsVB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawablePointsCB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawableTrianglesVB(parts.size());
    std::vector<std::vector<easy3d::vec3>> drawableTrianglesCB(parts.size());
    std::vector<std::vector<uint>> drawableTrianglesIB(parts.size());

    std::map<uint, uint> partID2Index;
    for (uint index = 0; index < parts.size(); index++)
    {
        Part::Ptr partPtr = parts[index];
        const CMesh& mesh = partPtr->mesh;

        if (mesh.n_faces() == 0)
            continue;

        partID2Index[partPtr->ID] = index;
        // Create a drawable mesh
        drawablesMeshes[index] = new easy3d::SurfaceMesh;
        drawablesMeshes[index]->set_name("part " + std::to_string(partPtr->userID));
        drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:marking");
        for (uint i = 0; i < visFrames.size(); i++)
        {
            drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
        }
        modelToFrameToVertexbuffer[drawablesMeshes[index]] = std::vector<std::vector<easy3d::vec3>>(numFrames);
        modelToFrameToColorbuffer[drawablesMeshes[index]] = std::vector<std::vector<easy3d::vec3>>(numFrames);
        // Visualize points as billboards
        drawablesPoints[index] = drawablesMeshes[index]->add_points_drawable("vertices");
        drawablesPoints[index]->set_point_size(5);
        drawablesPoints[index]->set_per_vertex_color(true);
        drawablesPoints[index]->set_visible(false);
        // Visualize triangles with face normals
        drawablesTriangles[index] = drawablesMeshes[index]->add_triangles_drawable("faces");
        drawablesTriangles[index]->set_distinct_back_color(false);
        drawablesTriangles[index]->set_lighting_two_sides(true);
        drawablesTriangles[index]->set_smooth_shading(false);
        drawablesTriangles[index]->set_per_vertex_color(true);
        easy3d::Material m;
        m.specular = easy3d::vec3(0.0f, 0.0f, 0.0f);
        drawablesTriangles[index]->set_material(m);
    }

    const CMesh& mesh = scene->mesh;

    auto buildDrawable = [&](size_t index) {
        Part::Ptr partPtr = parts[index];
        int partID = partPtr->ID;

        if (partPtr->mesh.n_faces() == 0)
            return;

        easy3d::SurfaceMesh* drawableMesh = drawablesMeshes[index];
        std::vector<std::vector<easy3d::vec3>>& frameToVertexbuffer = modelToFrameToVertexbuffer[drawableMesh];
        std::vector<std::vector<easy3d::vec3>>& frameToColorbuffer = modelToFrameToColorbuffer[drawableMesh];
        frameToVertexbuffer = std::vector<std::vector<easy3d::vec3>>(numFrames);
        frameToColorbuffer = std::vector<std::vector<easy3d::vec3>>(numFrames);
        // Copy over vertices and their properties to drawable model
        vector<easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex(mesh.n_vertices());
        for (VHandle v : mesh.vertices())
        {
            FHandle f = *mesh.cvf_begin(v);
            if (mesh.data(f).element->partID != partID)
                continue;
            OMVec3 position = mesh.point(v);
            vertexToDrawableVertex[v.idx()]
                = drawableMesh->add_vertex(easy3d::vec3(position[0], position[1], position[2]));
            const MatX3& positions = mesh.data(v).node->positions;
            for (uint i = 0; i < visFrames.size(); i++)
            {
                auto drawablePositions
                    = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
                easy3d::vec3 pos = easy3d::vec3(positions.coeff(visFrames[i], 0),
                                                positions.coeff(visFrames[i], 1),
                                                positions.coeff(visFrames[i], 2));
                drawablePositions[vertexToDrawableVertex[v.idx()]] = pos;
            }
            auto colors = drawableMesh->get_vertex_property<easy3d::vec3>("v:marking");
            if (mesh.status(v).fixed_nonmanifold())
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.8f, 0.0f, 0.0f);
            else if (mesh.data(v).node->referencingParts > 1)
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.9f, 0.6f, 0.0f);
            else if (mesh.is_boundary(v))
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.8f, 0.8f, 0.0f);
            else
                colors[vertexToDrawableVertex[v.idx()]] = easy3d::vec3(0.0f, 0.6f, 0.0f);
        }

        for (FHandle f : mesh.faces())
        {
            if (mesh.data(f).element->partID != partID)
                continue;
            vector<easy3d::SurfaceMesh::Vertex> vs;
            auto& strains = mesh.data(f).element->plasticStrains;
            for (VHandle v : mesh.fv_range(f))
            {
                auto drawableVertex = vertexToDrawableVertex[v.idx()];
                vs.emplace_back(drawableVertex);
                for (uint i = 0; i < visFrames.size(); i++)
                {
                    auto drawablePositions
                        = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[i]));
                    easy3d::vec3 col = getStrainColor(strains(visFrames[i]));
                    frameToColorbuffer[visFrames[i]].emplace_back(col);
                    frameToVertexbuffer[visFrames[i]].emplace_back(drawablePositions[drawableVertex]);
                }
            }
            drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
        }

        auto drawablePositions
            = drawableMesh->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[currentFrame]));
        auto pos = drawableMesh->get_vertex_property<easy3d::vec3>("v:point");
        pos.vector() = drawablePositions.vector();

        // Visualize points as billboards
        drawablePointsVB[index] = drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector();
        drawablePointsCB[index] = drawableMesh->get_vertex_property<easy3d::vec3>("v:marking").vector();

        vector<easy3d::vec3>& points = drawableTrianglesVB[index];
        vector<easy3d::vec3>& faceColors = drawableTrianglesCB[index];
        vector<uint>& indices = drawableTrianglesIB[index];
        if (strainColors)
        {
            points = frameToVertexbuffer[visFrames[currentFrame]];
            faceColors = frameToColorbuffer[visFrames[currentFrame]];
            indices = std::vector<uint>(3 * drawableMesh->faces_size());
            std::iota(indices.begin(), indices.end(), 0);
        }
        else
        {
            points = drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector();
            std::srand(partPtr->userID);
            faceColors = std::vector<easy3d::vec3>(drawableMesh->n_vertices(), easy3d::random_color());

            for (auto f : drawableMesh->faces())
            {
                easy3d::SurfaceMesh::Halfedge he = drawableMesh->halfedge(f);
                for (int j = 0; j < 3; j++)
                {
                    indices.emplace_back(drawableMesh->to_vertex(he).idx());
                    he = drawableMesh->next_halfedge(he);
                }
            }
        }
    };

    std::vector<size_t> partIndices(parts.size());
    std::iota(partIndices.begin(), partIndices.end(), 0);

    parallel_for_each(partIndices, buildDrawable);

    for (uint index = 0; index < parts.size(); index++)
    {
        easy3d::SurfaceMesh* drawableMesh = drawablesMeshes[index];
        if (!drawableMesh)
            continue;

        easy3d::PointsDrawable* drawablePoints = drawablesPoints[index];
        drawablePoints->update_vertex_buffer(drawablePointsVB[index]);
        drawablePoints->update_color_buffer(drawablePointsCB[index]);
        easy3d::TrianglesDrawable* drawableTriangles = drawablesTriangles[index];
        drawableTriangles->update_vertex_buffer(drawableTrianglesVB[index]);
        drawableTriangles->update_color_buffer(drawableTrianglesCB[index]);
        drawableTriangles->update_index_buffer(drawableTrianglesIB[index]);

        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
        add_model(drawableMesh, false);
        std::cout.clear();
        std::cerr.clear();
    }

    updateFaceVisibility();
    updateVertexVisibility();
    updateBoundaryVisibility();
    updateWireframeVisibility();

    model_idx_ = preModelIdx;

    if (partsExpanded)
    {
        partsExpanded = false;
        toggleExpandParts();
    }
    if (epicenters.rows() > 0)
    {
        createDrawableEpicenters();
    }

    return true;
}

bool ImGuiViewer::decimateScene()
{
    if (deciParts.quadricPositionOptimization || deciScene.quadricPositionOptimization)
    {
        fullReload = true;
    }
    if (!deciParts.decimateScene(scene, targetFaces, targetVertices))
    {
        Logger::lout(Logger::ERROR) << "Global scene decimation failed" << std::endl;
        return false;
    }

    updateMaxStrains();

    createDrawableScene();

    updateGlobalStats();

    updateFaceVisibility();
    updateVertexVisibility();
    updateBoundaryVisibility();
    updateWireframeVisibility();

    return true;
}

void ImGuiViewer::HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void ImGuiViewer::drawInfoPanel()
{
    ImGui::SetNextWindowSize(ImVec2(200.0f * widget_scaling(), 300.0f * widget_scaling()), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(0.05f * width(), 0.05 * height()), ImGuiCond_Appearing, ImVec2(0.0f, 0.0f));
    if (ImGui::Begin("Information",
                     nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize
                         | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing
                         | ImGuiWindowFlags_NoNav))
    {
        if (ImGui::CollapsingHeader("Mesh/Info"))
        {
            ImGui::Separator();

            if (stage == 0)
            {
                if (ImGui::Button("Load a file to start"))
                    openDialog();
            }
            else
            {
                ImGui::Text("Current mesh file: %s", easy3d::file_system::base_name(fileName).c_str());
                if (ImGui::Button("Load a different file (close current)"))
                    openDialog();
                ImGui::Separator();
                if (ImGui::Button("Show animation frames"))
                    ImGui::OpenPopup("Choose animation frames");

                if (ImGui::BeginPopupModal("Choose animation frames"))
                {
                    ImGui::Text("How many of the total %i frames should be visualized?", numFrames);
                    ImGui::SliderInt("", &nVisFrames, 1, numFrames);
                    ImGui::Text("Frames are linearly spaced over the whole frame range.");
                    ImGui::Text("Visualizing more than ~10-20 frames NOT recommended for big models!");
                    ImGui::Text("More frames = more memory consumption and longer loading times!");
                    ImGui::Text("This affects only visualization, decimation is independent of this!");
                    if (ImGui::Button("OK"))
                    {
                        if ((int)visFrames.size() != nVisFrames)
                        {
                            float frameSkip = numFrames;
                            if (nVisFrames > 1)
                                frameSkip = std::max(1.0f, 1.0f / (nVisFrames - 1) * (numFrames - 1));

                            visFrames.clear();
                            for (float frameF = 0; frameF < numFrames; frameF += frameSkip)
                                visFrames.emplace_back(std::floor(frameF));

                            currentFrame = 0;

                            if (stage == 1)
                                createDrawableParts();
                            else if (stage == 2)
                                createDrawableScene();
                        }
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }
                ImGui::Separator();
                // if (ImGui::Button("Update global stats"))
                // {
                //     updateGlobalStats();
                // }
                if (ImGui::Button("Configure strain colors"))
                    ImGui::OpenPopup("Configure strain coloring");

                if (ImGui::BeginPopupModal("Configure strain coloring"))
                {
                    ImGui::Text("Set the upper bound for color mapping of strain values");
                    ImGui::SliderFloat("", &maxStrain, 0.001f, maxStrainGlobal);
                    ImGui::Text("%.3f is the global maximum strain value", maxStrainGlobal);
                    ImGui::Text("Color of upper bound (strains = %.3f):.", maxStrain);
                    ImGui::SliderFloat("HueUpper", &maxColorHSV.h, 0.0f, 1.0f);
                    // ImGui::SameLine();
                    ImGui::SliderFloat("SatUpper", &maxColorHSV.s, 0.0f, 1.0f);
                    ImVec4 maxColorRGB;
                    ImGui::ColorConvertHSVtoRGB(
                        maxColorHSV.h, maxColorHSV.s, maxColorHSV.v, maxColorRGB.x, maxColorRGB.y, maxColorRGB.z);
                    // ImGui::SameLine();
                    ImGui::ColorButton("ColUpper", maxColorRGB);
                    ImGui::Text("Color of lower bound (strains = 0):");
                    ImGui::SliderFloat("HueLower", &minColorHSV.h, 0.0f, 1.0f);
                    // ImGui::SameLine();
                    ImGui::SliderFloat("SatLower", &minColorHSV.s, 0.0f, 1.0f);
                    ImVec4 minColorRGB;
                    ImGui::ColorConvertHSVtoRGB(
                        minColorHSV.h, minColorHSV.s, minColorHSV.v, minColorRGB.x, minColorRGB.y, minColorRGB.z);
                    // ImGui::SameLine();
                    ImGui::ColorButton("ColLower", minColorRGB);
                    ImGui::Text("Strains above upper bound are clamped");
                    ImGui::Text("Lower bound: %.3f, %.3f, %.3f", minColorHSV.h, minColorHSV.s, minColorHSV.v);
                    ImGui::Text("Upper bound: %.3f, %.3f, %.3f", maxColorHSV.h, maxColorHSV.s, maxColorHSV.v);
                    ImGui::Text("Colors between bounds are linearly interpolated using HSV, depending on face strains");
                    if (ImGui::Button("OK"))
                    {
                        if (stage == 1)
                            createDrawableParts();
                        else if (stage == 2)
                            createDrawableScene();
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }
                ImGui::Separator();
                if (visFrames.size() > 0)
                {
                    ImGui::Text("Current frame: %u", visFrames[currentFrame]);
                }
                else
                {
                    ImGui::Text("Current frame: none");
                }
                if (visFrames.size() > 1)
                {
                    int preFrame = currentFrame;
                    ImGui::SliderInt("Choose animation frame", &currentFrame, 0, visFrames.size() - 1);
                    if (preFrame != currentFrame && !animating)
                        updateFrame();
                    if (ImGui::Button("Toggle animation"))
                    {
                        animating = !animating;
                    }
                }
                if (ImGui::Button("Toggle strain/part coloring"))
                {
                    toggleStrainColors();
                }
                if (ImGui::Button("Toggle part expansion"))
                {
                    toggleExpandParts();
                }
            }
            // ImGui::Separator();
            // ImGui::Text("Frame rate: %.1f", ImGui::GetIO().Framerate);
            // ImGui::Text("GPU time (ms): %4.1f", gpu_time_);

            ImGui::Separator();

            if (stage > 0 && ImGui::CollapsingHeader("Global scene stats"))
            {
                ImGui::Text("#Frames: %i (#frames visualized: %lu)", numFrames, visFrames.size());
                ImGui::Text("#parts %i", (int)parts.size());
                ImGui::Text("#1Dparts %i", numParts1D);
                ImGui::Text("#2Dparts %i", numParts2D);
                ImGui::Text("#3Dparts %i", numParts3D);
                ImGui::Text("#MeshFaces: %i", numTriangles);
                ImGui::Text("#MeshVertices: %i", numVertices);
                ImGui::Text("#MeshEdges: %i", numEdges);
                ImGui::Text("#MeshComplexEdges: %i", numComplexEdges);
                ImGui::Text("#MeshBoundaryEdges: %i", numBoundaryEdges);
                ImGui::Text("#FENodes: %i", numNodes);
                ImGui::Text("#1DFEs: %i", num1DFE);
                ImGui::Text("#2DFEs (includes extracted 3D surfaces): %i", num2DFE);
                ImGui::Text("Max plastic strain in current frame: %.3f",
                            frame2maxPlasticStrain[visFrames[currentFrame]]);
                ImGui::Text("Max plastic strain over all frames: %.3f", maxStrainGlobal);
            }

            ImGui::Separator();
            if (current_model() && ImGui::CollapsingHeader("Current part stats"))
            {
                const std::string& name = "Current model: " + easy3d::file_system::simple_name(current_model()->name());
                ImGui::Text("%s", name.c_str());
                if (dynamic_cast<easy3d::Graph*>(current_model()))
                {
                    if (current_model()->name() == "epicenter")
                    {
                        ImGui::Text("Type: epicenter visualization");
                        ImGui::TextWrapped(
                            "Decimation error is scaled up for elements inside the sphere, and is scaled "
                            "down for elements outside the sphere.");
                        ImGui::TextWrapped("This means elements inside the sphere are preserved in more detail");
                        ImGui::TextWrapped(
                            "Remove calculated epicenters from decimation by deleting this model (DEL).");
                    }
                }
                else if (dynamic_cast<easy3d::SurfaceMesh*>(current_model()))
                {
                    easy3d::SurfaceMesh* mesh = dynamic_cast<easy3d::SurfaceMesh*>(current_model());
                    partid_t userID = -1;
                    if (current_model()->name().substr(0, 4) == "part")
                        userID = atoi(current_model()->name().substr(4).c_str());
                    if (pIs2DPart[userID])
                        ImGui::Text("2D surface part");
                    else
                        ImGui::Text("Volumetric part surface");
                    ImGui::Text("#MeshFaces: %i", mesh->n_faces());
                    ImGui::Text("#MeshVertices: %i", mesh->n_vertices());
                    ImGui::Text("#MeshEdges: %i", mesh->n_edges());
                    ImGui::Text("#MeshComplexEdges: %i", pNumComplexEdges[userID]);
                    ImGui::Text("#MeshBoundaryEdges: %i", pNumBoundaryEdges[userID]);
                    ImGui::Text("#FENodes: %i", pNumNodes[userID]);
                    ImGui::Text("#2DFEs (includes extracted 3D surfaces): %i", pNum2DFE[userID]);
                    ImGui::Text("Max plastic strain in current frame: %.3f",
                                frame2pMaxPlasticStrain[visFrames[currentFrame]][userID]);
                    float partMaxStrain = 0.0f;
                    for (size_t i = 0; i < frame2maxPlasticStrain.size(); i++)
                    {
                        partMaxStrain = std::max(partMaxStrain, frame2pMaxPlasticStrain[i][userID]);
                    }
                    ImGui::Text("Max plastic strain over all frames: %.3f", partMaxStrain);
                }
            }
        }
        ImGui::End();
    }
}

void ImGuiViewer::drawDecimationPanel()
{

    ImGui::SetNextWindowSize(ImVec2(200 * widget_scaling(), 300 * widget_scaling()), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(width() * 0.5, height() * 0.05), ImGuiCond_Appearing, ImVec2(1.0f, 0.0f));
    if (ImGui::Begin("Tools",
                     nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize
                         | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing
                         | ImGuiWindowFlags_NoNav))
    {
        if (ImGui::CollapsingHeader("Decimation"))
        {
            ImGui::Separator();
            if (stage == 1)
            {
                ImGui::Text("PART-WISE STAGE");
            }
            else if (stage == 2)
            {
                ImGui::Text("GLOBAL SCENE STAGE");
            }
            ImGui::Separator();
            if (stage != 0 && ImGui::CollapsingHeader("Actions"))
            {
                ImGui::Indent();
                if (ImGui::Button("Reload original model"))
                {
                    stage = 0;
                    if (fullReload)
                    {
                        openFile(fileName);
                    }
                    else
                    {
                        buildParts();
                    }
                    ImGui::CloseCurrentPopup();
                }
                ImGui::Separator();
                if (ImGui::Button("Export selected part") && current_model())
                {
                    // if (partsExpanded)
                    // {
                    //     toggleExpandParts();
                    // }
                    exportCurrentPart();
                }
                if (ImGui::Button("Export scene"))
                {
                    // if (partsExpanded)
                    // {
                    //     toggleExpandParts();
                    // }
                    exportScene();
                }
                if (stage == 1)
                {
                    ImGui::Separator();
                    if (ImGui::Button("Calculate epicenters (will then be used for error scaling)"))
                    {
                        calcEpicenters();
                    }
                    ImGui::Separator();
                    if (ImGui::Button("Decimate part-wise"))
                    {
                        decimatePartwise();
                    }
                    ImGui::Separator();
                    if (ImGui::Button("Merge part meshes to scene"))
                    {
                        mergeParts();
                    }
                }
                if (stage == 2)
                {
                    ImGui::Separator();
                    ImGui::InputInt("Target #vertices", &targetVertices, 10000, 100000);
                    ImGui::InputInt("Target #faces", &targetFaces, 10000, 100000);
                    ImGui::Text("0 = as far as possible within error bounds");
                    if (ImGui::Button("Decimate globally"))
                    {
                        decimateScene();
                    }
                }
                ImGui::Unindent();
            }

            ImGui::Separator();
            ImGui::Text("Decimation Settings");
            ImGui::Text("Current guiding function:");
            ImGui::Indent();
            if (deciParts.useQuadric && !deciParts.quadricExcludeOnly && !deciParts.combineQuadricNormal)
            {
                ImGui::Text("QEM (Quadric error metric):");
            }
            else if (deciParts.useNormalDeviation && !deciParts.normalExcludeOnly && !deciParts.combineQuadricNormal)
            {
                ImGui::Text("NDM (Normal deviation metric):");
            }
            else if (deciParts.combineQuadricNormal)
            {
                ImGui::Text("QEM (Quadric error metric) + NDM (Normal deviation metric):");
            }
            ImGui::Unindent();
            ImGui::Text("Exclusion error bounds:");
            ImGui::Indent();
            if (deciParts.useQuadric && deciParts.maxQuadricError < 1e9)
                ImGui::Text("Max QEM error: %f", deciParts.maxQuadricError);
            else
                ImGui::Text("QEM error unbound");
            if (deciParts.useNormalDeviation && deciParts.maxNormalDeviation < 360.0f)
                ImGui::Text("Max NDM error: %f", deciParts.maxNormalDeviation);
            else
                ImGui::Text("NDM error unbound");
            if (deciParts.useBoundaryDeviation && deciParts.maxBoundaryDeviation < 180.0f)
                ImGui::Text("Max boundary angle deviation: %f", deciParts.maxBoundaryDeviation);
            else
                ImGui::Text("Boundary angles unbound");
            if (deciParts.useAspectRatio)
                ImGui::Text("Max triangle aspect ratio: %f", deciParts.maxAspectRatio);
            else
                ImGui::Text("Triangle aspect ratio unbound");
            ImGui::Unindent();
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Quadric error metric settings"))
            {
                ImGui::Indent();
                ImGui::Text("QEM is always used as the base metric");
                if (deciParts.useQuadric)
                {
                    // ImGui::Checkbox("Use QEM for error bound exclusion only", &deciParts.quadricExcludeOnly);
                    int auxFramesQuadric = deciParts.framesQuadric;
                    ImGui::SliderInt("#frames for QEM", &auxFramesQuadric, 1, numFrames);
                    deciParts.framesQuadric = auxFramesQuadric;
                    deciParts.maxQuadricError = std::max(deciParts.maxQuadricError, 0.0f);
                    ImGui::InputFloat("Max allowed QEM error: ", &deciParts.maxQuadricError, 100.0f, 1000.0f, "%.3f");
                    if (ImGui::Button("Make QEM error unbound"))
                    {
                        deciParts.maxQuadricError = FLT_MAX;
                    }
                    ImGui::Checkbox("Use quadrics to preserve boundary shape", &deciParts.boundaryQuadrics);
                    ImGui::Checkbox("Use quadrics to preserve plastic strain distribution", &deciParts.featureQuadrics);
                    ImGui::Checkbox("Weight quadrics by triangle area", &deciParts.quadricAreaWeighting);
                    ImGui::Checkbox("Optimize vertex position using QEM (= optimal edge collapse)",
                                    &deciParts.quadricPositionOptimization);
                }
                ImGui::Unindent();
            }
            if (ImGui::CollapsingHeader("Normal deviation metric settings"))
            {
                ImGui::Indent();
                if (deciParts.useQuadric)
                {
                    ImGui::Checkbox("Use normal deviation metric for decimation", &deciParts.useNormalDeviation);
                }
                else
                {
                    ImGui::Text("Using normal deviation metric");
                    deciParts.useNormalDeviation = true;
                    deciParts.combineQuadricNormal = false;
                }
                if (deciParts.useNormalDeviation)
                {
                    ImGui::Checkbox("Use NDM as guiding function together with QEM", &deciParts.combineQuadricNormal);
                    deciParts.normalExcludeOnly = !deciParts.combineQuadricNormal;
                    if (!deciParts.combineQuadricNormal)
                    {
                        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
                    }
                    ImGui::Text("A QEM error of ");
                    ImGui::SameLine();
                    ImGui::InputFloat("mm^2", &deciParts.combineNormalWeight, 0.1f, 1.0f, "%.3f");
                    ImGui::Text("should match NDM error of");
                    ImGui::SameLine();
                    ImGui::InputFloat("degrees", &deciParts.combineQuadricWeight, 0.1f, 1000.0f, "%.3f");
                    deciParts.combineQuadricWeight = std::clamp(deciParts.combineQuadricWeight, 0.1f, 1000.0f);
                    deciParts.combineNormalWeight = std::clamp(deciParts.combineNormalWeight, 0.1f, 1000.0f);
                    if (!deciParts.combineQuadricNormal)
                    {
                        ImGui::PopItemFlag();
                        ImGui::PopStyleVar();
                    }
                    int auxFramesNormal
                        = deciParts.combineQuadricNormal ? deciParts.framesQuadric : deciParts.framesNormalDeviation;
                    int minNormalFrames = deciParts.combineQuadricNormal ? deciParts.framesQuadric : 1;
                    int maxNormalFrames = deciParts.combineQuadricNormal ? deciParts.framesQuadric : numFrames;
                    ImGui::SliderInt("#frames for NDM: ", &auxFramesNormal, minNormalFrames, maxNormalFrames);
                    deciParts.framesNormalDeviation = auxFramesNormal;
                    ImGui::SliderFloat("Max normal deviation: ", &deciParts.maxNormalDeviation, 1, 360);
                    if (ImGui::Button("Make NDM error unbound"))
                    {
                        deciParts.maxNormalDeviation = 360;
                    }
                }
                ImGui::Unindent();
            }
            ImGui::Checkbox("Max boundary angle change", &deciParts.useBoundaryDeviation);
            ImGui::SameLine();
            if (!deciParts.useBoundaryDeviation)
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            }
            ImGui::SliderFloat(" degrees", &deciParts.maxBoundaryDeviation, 1, 180.0f);
            if (!deciParts.useBoundaryDeviation)
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }
            ImGui::Checkbox("Max triangle aspect ratio", &deciParts.useAspectRatio);
            ImGui::SameLine();
            if (!deciParts.useAspectRatio)
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            }
            ImGui::SliderFloat("", &deciParts.maxAspectRatio, 2, 100);
            if (!deciParts.useAspectRatio)
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }
        }
        ImGui::End();
    }
}

bool ImGuiViewer::updateMaxStrains()
{
    frame2maxPlasticStrain.clear();
    frame2pMaxPlasticStrain.clear();
    maxStrainGlobal = 0.0f;

    if (numFrames == 0)
        return false;

    frame2maxPlasticStrain.resize(numFrames);
    frame2pMaxPlasticStrain.resize(numFrames);
    for (auto partPtr : parts)
    {
        for (auto elem : partPtr->elements2D)
        {
            for (int i = 0; i < numFrames; i++)
            {
                frame2maxPlasticStrain[i] = std::max(frame2maxPlasticStrain[i], elem->plasticStrains(i));
                frame2pMaxPlasticStrain[i][partPtr->userID]
                    = std::max(frame2pMaxPlasticStrain[i][partPtr->userID], elem->plasticStrains(i));
            }
        }
        for (auto elem : partPtr->surfaceElements)
        {
            for (int i = 0; i < numFrames; i++)
            {
                frame2maxPlasticStrain[i] = std::max(frame2maxPlasticStrain[i], elem->plasticStrains(i));
                frame2pMaxPlasticStrain[i][partPtr->userID]
                    = std::max(frame2pMaxPlasticStrain[i][partPtr->userID], elem->plasticStrains(i));
            }
        }
    }
    for (int i = 0; i < numFrames; i++)
    {
        maxStrainGlobal = std::max(maxStrainGlobal, frame2maxPlasticStrain[i]);
    }
    return true;
}

bool ImGuiViewer::updateGlobalStats()
{
    numParts1D = 0;
    numParts2D = 0;
    numParts3D = 0;
    for (auto partPtr : parts)
    {
        if (!partPtr->elements1D.empty())
            numParts1D++;
        else if (!partPtr->elements2D.empty())
            numParts2D++;
        else if (!partPtr->elements3D.empty() || !partPtr->surfaceElements.empty())
            numParts3D++;
    }

    numTriangles = 0;
    numVertices = 0;
    numEdges = 0;
    numComplexEdges = 0;
    numBoundaryEdges = 0;
    num1DFE = 0;
    num2DFE = 0;
    numNodes = 0;
    pNumComplexEdges.clear();
    pNumBoundaryEdges.clear();
    pNum2DFE.clear();
    pNumNodes.clear();
    pIs2DPart.clear();

    std::string name = current_model() ? current_model()->name() : "";
    if (stage <= 0)
    {
        numFrames = 0;
        visFrames.clear();
    }
    else if (stage == 1)
    {
        for (auto partPtr : parts)
        {
            const CMesh& mesh = partPtr->mesh;
            numTriangles += mesh.n_faces();
            numVertices += mesh.n_vertices();
            numEdges += mesh.n_edges();
            float complexEdges = 0.0f;
            int boundaryEdges = 0;
            for (auto e : mesh.edges())
            {
                HEHandle he1 = mesh.halfedge_handle(e, 0);
                auto dupes1 = MeshAnalyzer::dupes(mesh, he1);
                HEHandle he2 = mesh.halfedge_handle(e, 1);
                auto dupes2 = MeshAnalyzer::dupes(mesh, he2);
                if (dupes1.size() > 1)
                    complexEdges += 1.0f / dupes1.size();
                if (dupes2.size() > 1)
                    complexEdges += 1.0f / dupes2.size();
                if (dupes1.size() == 1 && dupes2.size() == 1 && mesh.is_boundary(e))
                    boundaryEdges++;
            }
            numComplexEdges += (int)complexEdges;
            numBoundaryEdges += boundaryEdges;
            num1DFE += partPtr->elements1D.size();
            std::set<Node::Ptr> nodes;
            for (auto v : mesh.vertices())
            {
                nodes.emplace(mesh.data(v).node);
            }
            numNodes += nodes.size();
            std::set<Element2D::Ptr> elem2D;
            for (auto f : mesh.faces())
            {
                elem2D.emplace(mesh.data(f).element);
            }
            num2DFE += elem2D.size();
            pNumComplexEdges[partPtr->userID] += (int)complexEdges;
            pNumBoundaryEdges[partPtr->userID] += boundaryEdges;
            pNum2DFE[partPtr->userID] += elem2D.size();
            pNumNodes[partPtr->userID] += nodes.size();
            pIs2DPart[partPtr->userID] = partPtr->elements3D.empty() && partPtr->surfaceElements.empty();
        }
    }
    else if (stage == 2)
    {
        std::map<int, int> partID2UserID;
        std::map<int, float> userID2pComplexEdges;
        for (auto part : parts)
        {
            pIs2DPart[part->userID] = part->elements3D.empty() && part->surfaceElements.empty();
            num1DFE += part->elements1D.size();
            partID2UserID[part->ID] = part->userID;
        }

        const CMesh& mesh = scene->mesh;
        numTriangles += mesh.n_faces();
        numVertices += mesh.n_vertices();
        numEdges += mesh.n_edges();
        float complexEdges = 0.0f;
        int boundaryEdges = 0;
        for (auto e : mesh.edges())
        {
            HEHandle he1 = mesh.halfedge_handle(e, 0);
            auto dupes1 = MeshAnalyzer::dupes(mesh, he1);
            HEHandle he2 = mesh.halfedge_handle(e, 1);
            auto dupes2 = MeshAnalyzer::dupes(mesh, he2);
            if (dupes1.size() > 1)
                complexEdges += 1.0f / dupes1.size();
            if (dupes2.size() > 1)
                complexEdges += 1.0f / dupes1.size();
            if (dupes1.size() == 1 && dupes2.size() == 1 && mesh.is_boundary(e))
                boundaryEdges++;
            partid_t userID = -1;
            if (mesh.face_handle(he1).is_valid())
                userID = partID2UserID[mesh.data(mesh.face_handle(he1)).element->partID];
            else
                userID = partID2UserID[mesh.data(mesh.face_handle(he2)).element->partID];

            if (mesh.data(mesh.from_vertex_handle(he1)).node->referencingParts > 1
                && mesh.data(mesh.to_vertex_handle(he1)).node->referencingParts > 1)
            {
                pNumBoundaryEdges[userID]++;
            }
            else
            {
                if (dupes1.size() > 1)
                    userID2pComplexEdges[userID] += 1.0f / dupes1.size();
                if (dupes2.size() > 1)
                    userID2pComplexEdges[userID] += 1.0f / dupes2.size();
                if (dupes1.size() == 1 && dupes2.size() == 1 && mesh.is_boundary(e))
                    pNumBoundaryEdges[userID]++;
            }
        }
        numComplexEdges += (int)complexEdges;
        numBoundaryEdges += boundaryEdges;
        std::set<Node::Ptr> nodes;
        std::map<int, std::set<Node::Ptr>> userID2pNodes;
        for (auto v : mesh.vertices())
        {
            userID2pNodes[partID2UserID[mesh.data(*mesh.cvf_begin(v)).element->partID]].emplace(mesh.data(v).node);
            nodes.emplace(mesh.data(v).node);
        }
        numNodes += nodes.size();
        for (auto kv : userID2pNodes)
            pNumNodes[kv.first] += kv.second.size();
        std::set<Element2D::Ptr> elem2D;
        std::map<int, std::set<Element2D::Ptr>> userID2pElem2D;
        for (auto f : mesh.faces())
        {
            userID2pElem2D[partID2UserID[mesh.data(f).element->partID]].emplace(mesh.data(f).element);
            elem2D.emplace(mesh.data(f).element);
        }
        num2DFE += elem2D.size();
        for (auto kv : userID2pElem2D)
            pNum2DFE[kv.first] += kv.second.size();
    }

    return true;
}

bool ImGuiViewer::exportCurrentPart()
{
    const easy3d::Model* m = current_model();
    if (!m)
        return false;

    std::string name = m->name();
    if (name.substr(0, 4) != "part")
        return false;
    entid_t userID = atoi(name.substr(4).c_str());

    const std::string& title = "Please choose a file name";
    const std::vector<std::string>& filters
        = {"One mesh for each frame (*.obj)", "*.obj", "Crash2Mesh file (*.c2m)", "*.c2m"};

    std::string default_file_name = easy3d::file_system::base_name(fileName) + "_part" + std::to_string(userID);
    if (easy3d::file_system::extension(default_file_name).empty()) // no extension?
        default_file_name += ".obj";                               // default to obj

    const bool warn_overwrite = true;
    const std::string& file_name = easy3d::dialog::save(title, default_file_name, filters, warn_overwrite);
    if (file_name.empty())
        return false;

    if (dynamic_cast<const easy3d::SurfaceMesh*>(m))
    {
        // const easy3d::SurfaceMesh* mesh = dynamic_cast<const easy3d::SurfaceMesh*>(m);
        const std::string& ext = easy3d::file_system::extension(file_name, true);
        if (ext == "c2m")
        {
            if (stage == 1)
            {
                for (Part::Ptr part : parts)
                {
                    if (part->userID == userID)
                    {
                        std::vector<Part::Ptr> dummyParts({part});
                        Scene::Ptr dummyScene = MeshBuilder::merge(dummyParts, false);
                        C2MWriter::write(file_name, dummyScene, true);
                        return true;
                    }
                }
                return false;
            }
            else if (stage == 2)
            {
                partid_t partID = -1;
                for (Part::Ptr part : parts)
                {
                    if (part->userID == userID)
                    {
                        partID = part->ID;
                        break;
                    }
                }

                const CMesh& sceneMesh = scene->mesh;
                Part::Ptr part = std::make_shared<Part>(0, userID);
                CMesh& partMesh = part->mesh;
                // Copy over vertices and their properties to new part mesh
                std::map<VHandle, VHandle> sceneVtx2PartVtx;
                for (VHandle v : sceneMesh.vertices())
                {
                    FHandle f = *sceneMesh.cvf_begin(v);
                    if (sceneMesh.data(f).element->partID != partID)
                        continue;
                    sceneVtx2PartVtx[v] = partMesh.add_vertex(sceneMesh.point(v));
                    partMesh.data(sceneVtx2PartVtx[v]).node = sceneMesh.data(v).node;
                }

                for (FHandle f : sceneMesh.faces())
                {
                    if (sceneMesh.data(f).element->partID != partID)
                        continue;
                    vector<VHandle> vs;
                    for (VHandle v : sceneMesh.fv_range(f))
                    {
                        VHandle partVtx = sceneVtx2PartVtx[v];
                        vs.emplace_back(partVtx);
                    }
                    FHandle partFace = partMesh.add_face(vs);
                    partMesh.data(partFace).element = sceneMesh.data(f).element;
                }
                std::vector<Part::Ptr> dummyParts({part});
                Scene::Ptr dummyScene = MeshBuilder::merge(dummyParts, false);
                C2MWriter::write(file_name, dummyScene, true);
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (ext == "obj")
        {
            if (stage == 1)
            {
                // partid_t partID = -1;
                Part::Ptr part;
                for (Part::Ptr partPtr : parts)
                {
                    if (partPtr->userID == userID)
                    {
                        // partID = partPtr->ID;
                        part = partPtr;
                        break;
                    }
                }
                if (!part)
                    return false;

                const CMesh& mesh = part->mesh;
                easy3d::SurfaceMesh drawableMesh;
                drawableMesh.set_name("part " + std::to_string(userID));
                for (int i = 0; i < numFrames; i++)
                {
                    // drawableMesh.add_face_property<float>("f:strain_frame" + std::to_string(i));
                    drawableMesh.add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                }

                vector<easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex(mesh.n_vertices());
                for (VHandle v : mesh.vertices())
                {
                    OMVec3 position = mesh.point(v);
                    easy3d::SurfaceMesh::Vertex vd = vertexToDrawableVertex[v.idx()]
                        = drawableMesh.add_vertex(easy3d::vec3(position[0], position[1], position[2]));
                    const MatX3& positions = mesh.data(v).node->positions;
                    for (int i = 0; i < numFrames; i++)
                    {
                        auto drawablePositions
                            = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                        Vec3 p = positions.row(i).transpose();
                        drawablePositions[vd] = easy3d::vec3(p(0), p(1), p(2));
                    }
                }
                for (FHandle f : mesh.faces())
                {
                    vector<easy3d::SurfaceMesh::Vertex> vs;
                    for (VHandle v : mesh.fv_range(f))
                    {
                        vs.emplace_back(vertexToDrawableVertex[v.idx()]);
                    }
                    /*easy3d::SurfaceMesh::Face fd = */ drawableMesh.add_triangle(vs[0], vs[1], vs[2]);
                    // auto strains = mesh.data(f).element->plasticStrains;
                    // for (int i = 0; i < numFrames; i++)
                    // {
                    //     auto faceStrains = drawableMesh.get_face_property<float>("f:strain_frame" +
                    //     std::to_string(i)); faceStrains[fd] = strains(i);
                    // }
                }
                for (int i = 0; i < numFrames; i++)
                {
                    auto drawablePositions
                        = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                    auto pos = drawableMesh.get_vertex_property<easy3d::vec3>("v:point");
                    pos.vector() = drawablePositions.vector();
                    std::stringstream extStr;
                    extStr << "_" << std::setfill('0') << std::setw(3) << i << ".obj";
                    easy3d::SurfaceMeshIO::save(easy3d::file_system::name_less_extension(file_name) + extStr.str(),
                                                &drawableMesh);
                }
            }
            else if (stage == 2)
            {
                partid_t partID = -1;
                Part::Ptr part;
                for (Part::Ptr partPtr : parts)
                {
                    if (partPtr->userID == userID)
                    {
                        partID = partPtr->ID;
                        part = partPtr;
                        break;
                    }
                }
                if (!part)
                    return false;

                const CMesh& mesh = scene->mesh;
                easy3d::SurfaceMesh drawableMesh;
                drawableMesh.set_name("part " + std::to_string(userID));
                for (int i = 0; i < numFrames; i++)
                {
                    // drawableMesh.add_face_property<float>("f:strain_frame" + std::to_string(i));
                    drawableMesh.add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                }

                std::map<int, easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex;
                for (VHandle v : mesh.vertices())
                {
                    FHandle f = *mesh.cvf_begin(v);
                    if (mesh.data(f).element->partID != partID)
                        continue;
                    OMVec3 position = mesh.point(v);
                    easy3d::SurfaceMesh::Vertex vd = vertexToDrawableVertex[v.idx()]
                        = drawableMesh.add_vertex(easy3d::vec3(position[0], position[1], position[2]));
                    const MatX3& positions = mesh.data(v).node->positions;
                    for (int i = 0; i < numFrames; i++)
                    {
                        auto drawablePositions
                            = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                        Vec3 p = positions.row(i).transpose();
                        drawablePositions[vd] = easy3d::vec3(p(0), p(1), p(2));
                    }
                }
                for (FHandle f : mesh.faces())
                {
                    if (mesh.data(f).element->partID != partID)
                        continue;
                    vector<easy3d::SurfaceMesh::Vertex> vs;
                    for (VHandle v : mesh.fv_range(f))
                    {
                        vs.emplace_back(vertexToDrawableVertex[v.idx()]);
                    }
                    /*easy3d::SurfaceMesh::Face fd = */ drawableMesh.add_triangle(vs[0], vs[1], vs[2]);
                    // auto strains = mesh.data(f).element->plasticStrains;
                    // for (int i = 0; i < numFrames; i++)
                    // {
                    //     auto faceStrains = drawableMesh.get_face_property<float>("f:strain_frame" +
                    //     std::to_string(i)); faceStrains[fd] = strains(i);
                    // }
                }
                for (int i = 0; i < numFrames; i++)
                {
                    auto drawablePositions
                        = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                    auto pos = drawableMesh.get_vertex_property<easy3d::vec3>("v:point");
                    pos.vector() = drawablePositions.vector();
                    std::stringstream extStr;
                    extStr << "_" << std::setfill('0') << std::setw(3) << i << ".obj";
                    easy3d::SurfaceMeshIO::save(easy3d::file_system::name_less_extension(file_name) + extStr.str(),
                                                &drawableMesh);
                }
            }
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

bool ImGuiViewer::exportScene()
{
    const std::string& title = "Please choose a file name";
    const std::vector<std::string>& filters = {"Crash2Mesh file (*.c2m)",
                                               "*.c2m",
                                               //    "Single frame mesh (*.obj)", "*.obj",
                                               "One mesh for each frame (*.obj)",
                                               "*.obj"};

    std::string default_file_name = easy3d::file_system::base_name(fileName);

    const bool warn_overwrite = true;
    const std::string& file_name = easy3d::dialog::save(title, default_file_name, filters, warn_overwrite);
    if (file_name.empty())
        return false;

    Scene::Ptr exportScene;
    if (stage == 2)
        exportScene = scene;
    else
        exportScene = MeshBuilder::merge(parts, false);

    if (!exportScene)
        return false;

    const std::string& ext = easy3d::file_system::extension(file_name, true);
    if (ext == "c2m")
    {
        C2MWriter::write(file_name, exportScene, true);
        return true;
    }
    // else if (ext == "obj")
    // {
    //     const CMesh& mesh = exportScene->mesh;
    //     easy3d::SurfaceMesh drawableMesh;
    //     drawableMesh.set_name("exportScene");
    //     for (int i = 0; i < numFrames; i++)
    //     {
    //         drawableMesh.add_face_property<float>("f:strain_frame" + std::to_string(i));
    //         drawableMesh.add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
    //     }

    //     vector<easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex(mesh.n_vertices());
    //     for (VHandle v : mesh.vertices())
    //     {
    //         OMVec3 position = mesh.point(v);
    //         easy3d::SurfaceMesh::Vertex vd = vertexToDrawableVertex[v.idx()]
    //             = drawableMesh.add_vertex(easy3d::vec3(position[0], position[1], position[2]));
    //         const MatX3& positions = mesh.data(v).node->positions;
    //         for (int i = 0; i < numFrames; i++)
    //         {
    //             auto drawablePositions
    //                 = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
    //             easy3d::vec3 pos = easy3d::vec3(positions.coeff(i, 0), positions.coeff(i, 1), positions.coeff(i, 2));
    //             drawablePositions[vd] = pos;
    //         }
    //     }

    //     for (FHandle f : mesh.faces())
    //     {
    //         vector<easy3d::SurfaceMesh::Vertex> vs;
    //         for (VHandle v : mesh.fv_range(f))
    //         {
    //             vs.emplace_back(vertexToDrawableVertex[v.idx()]);
    //         }
    //         easy3d::SurfaceMesh::Face fd = drawableMesh.add_triangle(vs[0], vs[1], vs[2]);
    //         auto strains = mesh.data(f).element->plasticStrains;
    //         for (int i = 0; i < numFrames; i++)
    //         {
    //             auto faceStrains = drawableMesh.get_face_property<float>("f:strain_frame" + std::to_string(i));
    //             faceStrains[fd] = strains(i);
    //         }
    //     }

    //     auto drawablePositions
    //         = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" +
    //         std::to_string(visFrames[currentFrame]));
    //     auto pos = drawableMesh.get_vertex_property<easy3d::vec3>("v:point");
    //     pos.vector() = drawablePositions.vector();

    //     return easy3d::SurfaceMeshIO::save(file_name, &drawableMesh);
    // }
    else if (ext == "obj")
    {
        std::map<partid_t, Part::Ptr> partID2Part;
        for (auto part : parts)
        {
            partID2Part[part->ID] = part;
        }
        const CMesh& mesh = exportScene->mesh;
        easy3d::SurfaceMesh drawableMesh;
        drawableMesh.set_name("scene");
        for (int i = 0; i < numFrames; i++)
        {
            // drawableMesh.add_face_property<float>("f:strain_frame" + std::to_string(i));
            drawableMesh.add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
        }

        vector<easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex(mesh.n_vertices());
        for (VHandle v : mesh.vertices())
        {
            FHandle f = *mesh.cvf_begin(v);
            Part::Ptr part = partID2Part[mesh.data(f).element->partID];

            OMVec3 position = mesh.point(v);
            easy3d::SurfaceMesh::Vertex vd = vertexToDrawableVertex[v.idx()]
                = drawableMesh.add_vertex(easy3d::vec3(position[0], position[1], position[2]));
            const MatX3& positions = mesh.data(v).node->positions;
            for (int i = 0; i < numFrames; i++)
            {
                auto drawablePositions
                    = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                Vec3 p = positions.row(i).transpose();
                if (partsExpanded)
                    p += 1.3f * part->centers.row(i).transpose();
                drawablePositions[vd] = easy3d::vec3(p(0), p(1), p(2));
            }
        }
        for (FHandle f : mesh.faces())
        {
            vector<easy3d::SurfaceMesh::Vertex> vs;
            for (VHandle v : mesh.fv_range(f))
            {
                vs.emplace_back(vertexToDrawableVertex[v.idx()]);
            }
            /*easy3d::SurfaceMesh::Face fd = */ drawableMesh.add_triangle(vs[0], vs[1], vs[2]);
            // auto strains = mesh.data(f).element->plasticStrains;
            // for (int i = 0; i < numFrames; i++)
            // {
            //     auto faceStrains = drawableMesh.get_face_property<float>("f:strain_frame" + std::to_string(i));
            //     faceStrains[fd] = strains(i);
            // }
        }
        for (int i = 0; i < numFrames; i++)
        {
            auto drawablePositions = drawableMesh.get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
            auto pos = drawableMesh.get_vertex_property<easy3d::vec3>("v:point");
            pos.vector() = drawablePositions.vector();
            std::stringstream extStr;
            extStr << "_" << std::setfill('0') << std::setw(3) << i << ".obj";
            easy3d::SurfaceMeshIO::save(easy3d::file_system::name_less_extension(file_name) + extStr.str(),
                                        &drawableMesh);
        }
        return true;
    }
    else
        return false;
}

bool ImGuiViewer::createDrawableEpicenters()
{
    for (auto it = models_.begin(); it != models_.end(); it++)
    {
        auto model = *it;
        if (model->name() == "epicenter")
        {
            models_.erase(it);
            delete model;
            break;
        }
    }
    if (epicenters.rows() < numFrames)
    {
        std::cout << "TOO FEW ROWS" << std::endl;
        return false;
    }
    int preModelIdx = model_idx_;

    easy3d::Graph* epicenterSphere = new easy3d::Graph;
    epicenterSphere->set_name("epicenter");
    for (int i = 0; i < numFrames; i++)
    {
        // if (meandists(visFrames[i]) > 0.0f)
        epicenterSphere->add_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
    }

    int slices = 16;
    int stacks = 16;
    std::vector<std::vector<easy3d::Graph::Vertex>> stackToSliceToVertex(stacks + 1,
                                                                         std::vector<easy3d::Graph::Vertex>(slices));
    for (int t = 0; t < stacks + 1; t++)
    {
        float theta = ((float)(t) / stacks) * M_PI;
        for (int p = 0; p < slices; p++)
        {
            float phi = ((float)(p) / slices) * 2 * M_PI;
            stackToSliceToVertex[t][p] = epicenterSphere->add_vertex(easy3d::vec3(0.0f, 0.0f, 0.0f));
            easy3d::vec3 normal = easy3d::vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            for (int i = 0; i < numFrames; i++)
            {
                auto pos
                    = epicenterSphere->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(i));
                float r = meanDists(i);
                easy3d::vec3 center(epicenters.coeff(i, 0), epicenters.coeff(i, 1), epicenters.coeff(i, 2));
                pos[stackToSliceToVertex[t][p]] = center + r * normal;
            }
        }
    }

    std::vector<uint> indices;
    for (int t = 0; t < stacks; t++) // stacks are ELEVATION so they count theta
    {
        for (int p = 0; p < slices; p++) // slices are ORANGE SLICES so the count azimuth
        {
            auto vertex1 = stackToSliceToVertex[t][p];
            auto vertex2 = stackToSliceToVertex[t][(p + 1) % slices];
            auto vertex3 = stackToSliceToVertex[t + 1][p];

            epicenterSphere->add_edge(vertex1, vertex2);
            epicenterSphere->add_edge(vertex1, vertex3);
        }
    }
    epicenterSphere->add_edge(stackToSliceToVertex[0][0], stackToSliceToVertex[16][0]);
    epicenterSphere->add_edge(stackToSliceToVertex[8][0], stackToSliceToVertex[8][8]);
    epicenterSphere->add_edge(stackToSliceToVertex[8][4], stackToSliceToVertex[8][12]);
    auto pos = epicenterSphere->get_vertex_property<easy3d::vec3>("v:point");
    auto currentPos
        = epicenterSphere->get_vertex_property<easy3d::vec3>("v:pos_frame" + std::to_string(visFrames[currentFrame]));
    pos.vector() = currentPos.vector();

    // Visualize as vertices
    easy3d::PointsDrawable* pointsDrawable = epicenterSphere->add_points_drawable("vertices");
    pointsDrawable->update_vertex_buffer(pos.vector());
    pointsDrawable->set_point_size(5);
    pointsDrawable->set_per_vertex_color(false);
    pointsDrawable->set_default_color(easy3d::vec3(1.0f, 0.0f, 0.0f));
    pointsDrawable->set_visible(false);

    // + wireframe
    easy3d::LinesDrawable* wireframe = epicenterSphere->add_lines_drawable("edges");
    std::vector<unsigned int> lineIndices;
    for (auto e : epicenterSphere->edges())
    {
        easy3d::Graph::Vertex s = epicenterSphere->from_vertex(e);
        easy3d::Graph::Vertex t = epicenterSphere->to_vertex(e);
        lineIndices.push_back(s.idx());
        lineIndices.push_back(t.idx());
    }
    wireframe->update_vertex_buffer(pos.vector());
    wireframe->update_index_buffer(lineIndices);
    wireframe->set_default_color(easy3d::vec3(1.0f, 0.0f, 0.0f));
    wireframe->set_per_vertex_color(false);
    wireframe->set_visible(true);
    wireframe->set_line_width(4.0f);

    add_model(epicenterSphere, false);

    model_idx_ = preModelIdx;

    return true;
}
} // namespace c2m
