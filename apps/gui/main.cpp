#include <crash2mesh/viewer/imgui_viewer.hpp>

int main(int argc, char** argv)
{
    easy3d::logging::initialize();
    try {
        c2m::ImGuiViewer viewer("Mesh View");
        viewer.resize(1920, 1080);
        viewer.run();
    } catch (const std::runtime_error &e) {
        std::cout << "caught a fatal error: " + std::string(e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
