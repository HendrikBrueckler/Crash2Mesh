#ifndef C2M_ANIMATION_VIEWER_HPP
#define C2M_ANIMATION_VIEWER_HPP

#include <easy3d/viewer/viewer.h>

namespace c2m
{

class AnimationViewer : public easy3d::Viewer
{
  public:
    AnimationViewer::AnimationViewer(const std::string& title /* = "Easy3D ImGui Viewer" */,
                                     int samples /* = 4 */,
                                     int gl_major /* = 3 */,
                                     int gl_minor /* = 2 */,
                                     bool full_screen /* = false */,
                                     bool resizable /* = true */,
                                     int depth_bits /* = 24 */,
                                     int stencil_bits /* = 8 */
                                     )
        : Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits)
    {
    }

  protected:
    virtual bool key_press_event(int key, int modifiers) override;
};
} // namespace c2m
#endif
