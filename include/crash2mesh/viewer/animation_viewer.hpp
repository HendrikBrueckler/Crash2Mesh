#ifndef C2M_ANIMATION_VIEWER_HPP
#define C2M_ANIMATION_VIEWER_HPP

#include <easy3d/viewer/viewer.h>

namespace c2m
{

class AnimationViewer : public easy3d::Viewer
{
  public:
    AnimationViewer(const std::string& title = "AnimationViewer");

  protected:
    virtual bool key_press_event(int key, int modifiers) override;
};
}
#endif
