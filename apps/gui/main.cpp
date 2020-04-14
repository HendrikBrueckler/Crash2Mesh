#include <crash2mesh/viewer/imgui_viewer.hpp>

int main(int argc, char** argv)
{
    c2m::ImGuiViewer viewer("Mesh View");
    return viewer.run();
}
