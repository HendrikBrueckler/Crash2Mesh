#include <3rd_party/glfw/include/GLFW/glfw3.h> // Include glfw3.h after our OpenGL definitions
#include <crash2mesh/viewer/animation_viewer.hpp>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>

namespace c2m
{

AnimationViewer::AnimationViewer(const std::string& title) : Viewer(title)
{
}

bool AnimationViewer::key_press_event(int key, int modifiers)
{
    using namespace easy3d;
    if (key == GLFW_KEY_F && modifiers == 0)
    {
        fit_screen(current_model());
    }
    else if (key == GLFW_KEY_W && modifiers == 0)
    {
        for (auto model : models_)
        {
            SurfaceMesh* m = dynamic_cast<SurfaceMesh*>(model);
            if (m)
            {
                LinesDrawable* wireframe = m->lines_drawable("wireframe");
                if (!wireframe)
                {
                    wireframe = m->add_lines_drawable("wireframe");
                    std::vector<unsigned int> indices;
                    for (auto e : m->edges())
                    {
                        SurfaceMesh::Vertex s = m->vertex(e, 0);
                        SurfaceMesh::Vertex t = m->vertex(e, 1);
                        indices.push_back(s.idx());
                        indices.push_back(t.idx());
                    }
                    auto points = m->get_vertex_property<vec3>("v:point");
                    wireframe->update_vertex_buffer(points.vector());
                    wireframe->update_index_buffer(indices);
                    wireframe->set_default_color(vec3(0.0f, 0.0f, 0.0f));
                    wireframe->set_per_vertex_color(false);
                    wireframe->set_visible(true);
                }
                else
                    wireframe->set_visible(!wireframe->is_visible());
            }
        }
    }
    else if (key == GLFW_KEY_F9 && modifiers == 0)
    {
        if (models_.empty())
            model_idx_ = -1;
        else
            model_idx_ = int((model_idx_ - 1 + models_.size()) % models_.size());
        for (uint i = 0; i < models_.size(); i++)
        {
            models_[i]->set_visible(i == static_cast<uint>(model_idx_));
        }
    }
    else if (key == GLFW_KEY_F10 && modifiers == 0)
    {
        if (models_.empty())
            model_idx_ = -1;
        else
            model_idx_ = int((model_idx_ + 1) % models_.size());
        for (uint i = 0; i < models_.size(); i++)
        {
            models_[i]->set_visible(i == static_cast<uint>(model_idx_));
        }
    }
    else if (key == GLFW_KEY_F9 && modifiers == GLFW_MOD_CONTROL)
    {
        if (models_.empty())
            model_idx_ = -1;
        else
            model_idx_ = int((model_idx_ - 1 + models_.size()) % models_.size());
        for (uint i = 0; i < models_.size(); i++)
        {
            models_[i]->set_visible(i == static_cast<uint>(model_idx_));
        }
        if (model_idx_ >= 0)
        {
            fit_screen(current_model());
        }
    }
    else if (key == GLFW_KEY_F10 && modifiers == GLFW_MOD_CONTROL)
    {
        if (models_.empty())
            model_idx_ = -1;
        else
            model_idx_ = int((model_idx_ + 1) % models_.size());
        for (uint i = 0; i < models_.size(); i++)
        {
            models_[i]->set_visible(i == static_cast<uint>(model_idx_));
        }
        if (model_idx_ >= 0)
        {
            fit_screen(current_model());
        }
    }
    else
    {
        return Viewer::key_press_event(key, modifiers);
    }

    return false;
}
}