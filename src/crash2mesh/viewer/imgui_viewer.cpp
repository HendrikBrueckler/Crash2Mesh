#include <crash2mesh/viewer/imgui_viewer.hpp>

#include <cmath>
#include <iostream>

#include <easy3d/core/point_cloud.h>
#include <easy3d/core/random.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/util/file_system.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/drawable_lines.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/manipulated_camera_frame.h>
#include <easy3d/viewer/opengl_timer.h>
#include <easy3d/viewer/setting.h>

#include <easy3d/gui/picker_model.h>
#include <easy3d/util/dialogs.h>

#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/impl/imgui_impl_glfw.h>
#include <3rd_party/imgui/impl/imgui_impl_opengl3.h>
#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>

#include <3rd_party/glfw/include/GLFW/glfw3.h>

#include <algorithm>
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
#include <execution>
#endif

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

        // load font
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
        currentFrame = currentFrame <= 0 ? visFrames.size() - 1 : currentFrame - 1;
        updateFrame();
    }
    else if (key == GLFW_KEY_RIGHT && modifiers == 0)
    {
        currentFrame = currentFrame >= visFrames.size() - 1 ? 0 : currentFrame + 1;
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
            auto pos = std::find(models_.begin(), models_.end(), model);
            if (pos != models_.end())
            {
                model_idx_ = pos - models_.begin();
            }
        }
        else
        {
            model_idx_ = -1;
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
        currentFrame = currentFrame >= visFrames.size() - 1 ? 0 : currentFrame + 1;
        updateFrame();

        if (current_model())
        {
            fit_screen(current_model());
        }
    }
    AnimationViewer::pre_draw();
}

void ImGuiViewer::draw_overlay(bool* visible)
{
    drawInfoPanel();
    drawDecimationPanel();
}

void ImGuiViewer::post_draw()
{
    static bool show_overlay = true;
    if (show_overlay)
        draw_overlay(&show_overlay);

    static bool show_about = false;
    if (show_about)
    {
        ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::Begin("About Easy3D ImGui Viewer", &show_about, ImGuiWindowFlags_NoResize);
        ImGui::Text("This viewer shows how to use ImGui for GUI creation and event handling");
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

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
    if (ImGui::BeginMainMenuBar())
    {
        draw_menu_file();

        draw_menu_view();

        if (ImGui::BeginMenu("Help"))
        {
            ImGui::MenuItem("Manual", nullptr, &show_manual);
            ImGui::Separator();
            ImGui::MenuItem("About", nullptr, &show_about);
            ImGui::EndMenu();
        }
        menu_height_ = ImGui::GetWindowHeight();
        ImGui::EndMainMenuBar();
    }
    ImGui::PopStyleVar();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    AnimationViewer::post_draw();
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
        uint userID = atoi(name.substr(4).c_str());
        if (stage == 1)
        {
            for (auto partptr : parts)
            {
                if (partptr->userID == userID)
                {
                    partptr->mesh.clear();
                }
            }
        }
    }
    delete_model(current_model());

    model_idx_ = -1;

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
                    drawable->set_line_width(easy3d::setting::surface_mesh_borders_line_width);
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
    {
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        for (auto vertices : surface->points_drawables())
        {
            vertices->set_visible(drawVertices);
        }
    }
    return true;
}

bool ImGuiViewer::updateFaceVisibility()
{
    for (auto model : models_)
        for (easy3d::TrianglesDrawable* drawable : model->triangles_drawables())
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

    stage = -1;

    fileName = fileName_;
    erfh5::Reader reader(fileName, 100);

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

    buildParts();

    fit_screen();
    return true;
}

bool ImGuiViewer::buildParts()
{
    if (!MeshBuilder::build(parts, false))
    {
        Logger::lout(Logger::ERROR) << "Mesh building failed!" << std::endl;
        fileName = "";
        return false;
    }

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
            drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
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
        int n = 0;
        for (VHandle v : mesh.vertices())
        {
            OMVec3 position = mesh.point(v);
            vertexToDrawableVertex[v.idx()]
                = drawableMesh->add_vertex(easy3d::vec3(position[0], position[1], position[2]));
            const MatX3& positions = mesh.data(v).node->positions;
            for (uint i = 0; i < visFrames.size(); i++)
            {
                auto drawablePositions
                    = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
                easy3d::vec3 pos = easy3d::vec3(positions.coeff(visFrames[i], 0), positions.coeff(visFrames[i], 1), positions.coeff(visFrames[i], 2));
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

        auto drawablePositions
            = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[currentFrame]));
        auto pos = drawableMesh->get_vertex_property<easy3d::vec3>("v:point");
        pos.vector() = drawablePositions.vector();

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
                        = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
                    float delta = std::min(1.0f, 20.0f * strains(visFrames[i]));
                    easy3d::vec3 col = easy3d::vec3(1.0f, 1.0f - delta, 1.0f - delta);
                    frameToColorbuffer[visFrames[i]].emplace_back(col);
                    frameToVertexbuffer[visFrames[i]].emplace_back(drawablePositions[drawableVertex]);
                }
            }
            easy3d::SurfaceMesh::Face fd = drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
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

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
    std::for_each(std::execution::par_unseq, partIndices.begin(), partIndices.end(), buildDrawable);
#else
    for (size_t index : partIndices)
        buildDrawable(index);
#endif

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

        easy3d::SurfaceMesh::VertexProperty drawablePositions
            = surface->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[currentFrame]));
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
                    drawableTriangles->update_vertex_buffer(modelToFrameToVertexbuffer[surface][visFrames[currentFrame]]);
                std::vector<uint> indices = std::vector<uint>(3 * surface->faces_size());
                std::iota(indices.begin(), indices.end(), 0);
                drawableTriangles->update_index_buffer(indices);
            }
            else
            {
                std::string name = surface->name();
                uint userID = atoi(name.substr(4).c_str());
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
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (!surface)
            continue;

        easy3d::SurfaceMesh::VertexProperty drawablePositions
            = surface->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[currentFrame]));

        auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");
        pos.vector() = drawablePositions.vector();

        if (partsExpanded)
        {
            easy3d::vec3 center(0.0f, 0.0f, 0.0f);
            for (auto v : surface->vertices())
            {
                center += pos[v];
            }
            center /= surface->n_vertices();
            for (auto v : surface->vertices())
            {
                pos[v] += (1.3f) * center;
            }
        }

        for (auto drawableTriangles : surface->triangles_drawables())
        {
            if (strainColors)
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
                    drawableTriangles->update_vertex_buffer(modelToFrameToVertexbuffer[surface][visFrames[currentFrame]]);
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

    return true;
}

bool ImGuiViewer::calcEpicenters()
{
    MatX3 epicenters;
    VecX meanDists;
    MeshAnalyzer::getEpicenter(parts, epicenters, deciParts.meanDistsFromEpicenters);
    deciParts.epicenters = deciScene.epicenters = epicenters;
    deciParts.meanDistsFromEpicenters = deciScene.meanDistsFromEpicenters = meanDists;

    return true;
}

bool ImGuiViewer::toggleExpandParts()
{
    if (models_.empty())
        return false;

    for (auto model : models_)
    {
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        if (surface)
        {
            easy3d::vec3 center(0.0f, 0.0f, 0.0f);
            auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");
            for (auto v : surface->vertices())
            {
                center += pos[v];
            }
            center /= surface->n_vertices();
            for (auto v : surface->vertices())
            {
                if (partsExpanded)
                {
                    pos[v] -= (1.3f / 2.3f) * center;
                }
                else
                {
                    pos[v] += 1.3f * center;
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
    if (!deciParts.decimateParts(parts))
    {
        Logger::lout(Logger::ERROR) << "Decimation failed" << std::endl;
        return false;
    }
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
            drawablesMeshes[index]->add_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
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
    }

    const CMesh& mesh = scene->mesh;

    auto buildDrawable = [&](size_t index) {
        Part::Ptr partPtr = parts[index];
        uint partID = partPtr->ID;

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
            OMVec3 position = mesh.point(v);
            FHandle f = *mesh.cvf_begin(v);
            if (mesh.data(f).element->partID != partID)
                continue;
            vertexToDrawableVertex[v.idx()] = drawableMesh->add_vertex(easy3d::vec3(position[0], position[1], position[2]));
            const MatX3& positions = mesh.data(v).node->positions;
            for (uint i = 0; i < visFrames.size(); i++)
            {
                auto drawablePositions
                    = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
                easy3d::vec3 pos = easy3d::vec3(
                    positions.coeff(visFrames[i], 0), positions.coeff(visFrames[i], 1), positions.coeff(visFrames[i], 2));
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
            auto strains = mesh.data(f).element->plasticStrains;
            vector<easy3d::SurfaceMesh::Vertex> vs;
            if (mesh.data(f).element->partID != partID)
                continue;
            for (VHandle v : mesh.fv_range(f))
            {
                auto drawableVertex = vertexToDrawableVertex[v.idx()];
                vs.emplace_back(drawableVertex);
                for (uint i = 0; i < visFrames.size(); i++)
                {
                    auto drawablePositions
                        = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[i]));
                    float delta = std::min(1.0f, 20.0f * strains(visFrames[i]));
                    easy3d::vec3 col = easy3d::vec3(1.0f, 1.0f - delta, 1.0f - delta);
                    frameToColorbuffer[visFrames[i]].emplace_back(col);
                    frameToVertexbuffer[visFrames[i]].emplace_back(drawablePositions[drawableVertex]);
                }
            }
            easy3d::SurfaceMesh::Face fd = drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
        }

        auto drawablePositions
            = drawableMesh->get_vertex_property<easy3d::vec3>("v:posframe" + std::to_string(visFrames[currentFrame]));
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

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
    std::for_each(std::execution::par_unseq, partIndices.begin(), partIndices.end(), buildDrawable);
#else
    for (size_t index : partIndices)
        buildDrawable(index);
#endif

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
        toggleExpandScene();
    }

    return true;
}

bool ImGuiViewer::toggleExpandScene()
{
    if (models_.empty())
        return false;

    for (auto model : models_)
    {
        easy3d::SurfaceMesh* surface = dynamic_cast<easy3d::SurfaceMesh*>(model);
        auto pos = surface->get_vertex_property<easy3d::vec3>("v:point");
        if (surface)
        {
            easy3d::vec3 center(0.0f, 0.0f, 0.0f);
            for (auto v : surface->vertices())
            {
                center += pos[v];
            }
            center /= surface->n_vertices();
            for (auto v : surface->vertices())
            {
                if (partsExpanded)
                {
                    pos[v] -= (1.3f / 2.3f) * center;
                }
                else
                {
                    pos[v] += 1.3f * center;
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
                std::vector<easy3d::vec3> points;
                for (auto e : surface->edges())
                {
                    if (surface->is_boundary(e))
                    {
                        points.push_back(pos[surface->vertex(e, 0)]);
                        points.push_back(pos[surface->vertex(e, 1)]);
                    }
                }
                if (!points.empty())
                {
                    drawable->update_vertex_buffer(points);
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

    return true;
}

bool ImGuiViewer::decimateScene()
{
    for (FHandle f : scene->mesh.faces())
    {
        for (NormalCone& nc : scene->mesh.data(f).normalCones)
        {
            nc.max_angle() *= 20;
        }
    }
    if (!deciParts.decimateScene(scene, targetFaces, targetVertices))
    {
        Logger::lout(Logger::ERROR) << "Global scene decimation failed" << std::endl;
        return false;
    }

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
        ImGui::Text("Info");
        ImGui::Separator();

        if (stage > 0)
        {
            if (ImGui::Button("Show animation frames"))
                ImGui::OpenPopup("Choose animation frames");

            if (ImGui::BeginPopupModal("Choose animation frames"))
            {
                ImGui::Text("How many of the total %i frames should be visualized?", numFrames);
                ImGui::SliderInt("", &nVisFrames, 1, numFrames);
                ImGui::Text("Visualizing more than ~10-20 frames NOT recommended for big models!");
                ImGui::Text("More frames = more memory consumption and longer loading times!");
                ImGui::Text("This affects only visualization, decimation is independent of this!");
                if (ImGui::Button("OK")) 
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
                    ImGui::CloseCurrentPopup(); 
                }
                ImGui::EndPopup();
            }
            ImGui::Separator();
        }
        if (visFrames.size() > 0)
        {
            ImGui::Text("Current frame: %i", visFrames[currentFrame]);
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
        }
        if (stage > 0)
        {
            if (visFrames.size() > 1)
            {
                if (ImGui::Button("Toggle animation"))
                {
                    animating = !animating;
                }
            }
            if (ImGui::Button("Toggle strain coloring"))
            {
                toggleStrainColors();
            }
            if (ImGui::Button("Toggle part expansion"))
            {
                toggleExpandParts();
            }
        }
        ImGui::Separator();
        ImGui::Text("Frame rate: %.1f", ImGui::GetIO().Framerate);
        ImGui::Text("GPU time (ms): %4.1f", gpu_time_);

        ImGui::Separator();

        ImGui::Text("Global stats: ");
        ImGui::Text("#Frames: %i (#frames visualized: %i)", numFrames, visFrames.size());
        ImGui::Text("#Faces: %i", numTriangles);
        ImGui::Text("#Vertices: %i", numVertices);

        ImGui::Separator();
        if (current_model())
        {
            const std::string& name = "Current model: " + easy3d::file_system::simple_name(current_model()->name());
            ImGui::Text("%s", name.c_str());
            if (dynamic_cast<easy3d::PointCloud*>(current_model()))
            {
                easy3d::PointCloud* cloud = dynamic_cast<easy3d::PointCloud*>(current_model());
                ImGui::Text("Type: point cloud");
                ImGui::Text("#Vertices: %i", cloud->n_vertices());
            }
            else if (dynamic_cast<easy3d::SurfaceMesh*>(current_model()))
            {
                easy3d::SurfaceMesh* mesh = dynamic_cast<easy3d::SurfaceMesh*>(current_model());
                ImGui::Text("Type: surface mesh");
                ImGui::Text("#Faces: %i", mesh->n_faces());
                ImGui::Text("#Vertices: %i", mesh->n_vertices());
                ImGui::Text("#Edges: %i", mesh->n_edges());
            }
        }
        ImGui::End();
    }
}

void ImGuiViewer::drawDecimationPanel()
{
    if (stage < 0)
        return;

    ImGui::SetNextWindowSize(ImVec2(200 * widget_scaling(), 300 * widget_scaling()), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(width() * 0.5, height() * 0.05), ImGuiCond_Appearing, ImVec2(1.0f, 0.0f));
    if (ImGui::Begin("Tools",
                     nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize
                         | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing
                         | ImGuiWindowFlags_NoNav))
    {
        ImGui::Separator();
        if (stage == 1)
        {
            ImGui::Text("PART-WISE STAGE");
            if (ImGui::Button("Reload original model"))
            {
                stage = 0;
                buildParts();
            }
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
        else if (stage == 2)
        {
            ImGui::Text("GLOBAL SCENE STAGE");
            if (ImGui::Button("Reload original model"))
            {
                stage = 0;
                buildParts();
            }
            ImGui::Separator();
            ImGui::InputInt("Target #vertices (0 = no limit): ", &targetVertices, 10000, 100000);
            ImGui::InputInt("Target #faces (0 = no limit): ", &targetFaces, 10000, 100000);
            if (ImGui::Button("Decimate globally"))
            {
                decimateScene();
            }
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
        if (deciParts.useNormalDeviation && deciParts.maxNormalDeviation < 360)
            ImGui::Text("Max NDM error: %f", deciParts.maxNormalDeviation);
        else
            ImGui::Text("NDM error unbound", deciParts.maxNormalDeviation);
        if (deciParts.useBoundaryDeviation)
            ImGui::Text("Max boundary angle deviation: %f", deciParts.maxBoundaryDeviation);
        else
            ImGui::Text("boundary angle deviation unbound");
        if (deciParts.useAspectRatio)
            ImGui::Text("Max triangle aspect ratio: %f", deciParts.maxAspectRatio);
        else
            ImGui::Text("Triangle aspect ratio unbound");
        ImGui::Unindent();
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Quadric error metric settings"))
        {
            ImGui::Indent();
            ImGui::Checkbox("Use QEM for decimation", &deciParts.useQuadric);
            if (deciParts.useQuadric)
            {
                ImGui::Checkbox("Use QEM for error bound exclusion only", &deciParts.quadricExcludeOnly);
                int auxFramesQuadric = deciParts.framesQuadric;
                ImGui::SliderInt("How many frames should QEM consider: ", &auxFramesQuadric, 1, numFrames);
                deciParts.framesQuadric = auxFramesQuadric;
                deciParts.maxQuadricError = std::max(deciParts.maxQuadricError, 0.0f);
                ImGui::InputFloat("Exclude collapses with QEM error greater than: ",
                                  &deciParts.maxQuadricError,
                                  100.0f,
                                  1000.0f,
                                  "%.3f");
                if (ImGui::Button("Make QEM error unbound"))
                {
                    deciParts.maxQuadricError = FLT_MAX;
                }
                ImGui::Checkbox("Weight quadrics by triangle area", &deciParts.quadricAreaWeighting);
                ImGui::Checkbox("Optimize vertex position using QEM during decimation",
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
                if (deciParts.useQuadric && !deciParts.quadricExcludeOnly)
                {
                    ImGui::Checkbox("Use NDM for error bound exclusion only", &deciParts.normalExcludeOnly);
                    deciParts.combineQuadricNormal = !deciParts.normalExcludeOnly;
                }
                else
                {
                    ImGui::Text("Using NDM as primary guiding function");
                    deciParts.normalExcludeOnly = false;
                    deciParts.combineQuadricNormal = false;
                }
                int auxFramesNormal
                    = deciParts.combineQuadricNormal ? deciParts.framesQuadric : deciParts.framesNormalDeviation;
                int minNormalFrames = deciParts.combineQuadricNormal ? deciParts.framesQuadric : 1;
                int maxNormalFrames = deciParts.combineQuadricNormal ? deciParts.framesQuadric : numFrames;
                ImGui::SliderInt(
                    "How many frames should NDM consider: ", &auxFramesNormal, minNormalFrames, maxNormalFrames);
                deciParts.framesNormalDeviation = auxFramesNormal;
                ImGui::SliderFloat(
                    "Exclude collapses with NDM angle error greater than: ", &deciParts.maxNormalDeviation, 1, 360);
                if (ImGui::Button("Make NDM error unbound"))
                {
                    deciParts.maxNormalDeviation = 360;
                }
            }
            ImGui::Unindent();
        }
        ImGui::Checkbox("Exclude collapses altering boundary path angles by more than ",
                        &deciParts.useBoundaryDeviation);
        ImGui::SameLine();
        ImGui::SliderFloat(" degrees", &deciParts.maxBoundaryDeviation, 1, 360);
        ImGui::Checkbox("Limit triangle aspect ratios to less than than...", &deciParts.useAspectRatio);
        ImGui::SameLine();
        ImGui::SliderFloat("", &deciParts.maxAspectRatio, 2, 100);
        ImGui::End();
    }
}

bool ImGuiViewer::updateGlobalStats()
{
    numTriangles = 0;
    numVertices = 0;
    if (stage <= 0)
    {
        numFrames = 0;
        visFrames.clear();
        return false;
    }
    else if (stage == 1)
    {
        for (auto partPtr : parts)
        {
            numTriangles += partPtr->mesh.n_faces();
            numVertices += partPtr->mesh.n_vertices();
        }
        return true;
    }
    else if (stage == 2)
    {
        numTriangles += scene->mesh.n_faces();
        numVertices += scene->mesh.n_vertices();
        return true;
    }
    return false;
}

} // namespace c2m
