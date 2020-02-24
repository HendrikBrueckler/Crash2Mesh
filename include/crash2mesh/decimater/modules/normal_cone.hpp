#ifndef C2M_NORMAL_CONE_HPP
#define C2M_NORMAL_CONE_HPP

#include <crash2mesh/core/types.hpp>

namespace c2m
{

class NormalCone
{
  public:
    //! default constructor (not initialized)
    NormalCone()
    {
    }

    //! Initialize cone with center (unit vector) and angle (radius in radians)
    NormalCone(const OMVec3& _center_normal, float _max_angle, float _angle = 0.0);

    //! merge cones this instance will then enclose both former cones
    void merge(const NormalCone& _cone);

    //! returns center normal
    const OMVec3& center_normal() const
    {
        return center_normal_;
    }

    //! returns size of cone (radius in radians)
    inline float angle() const
    {
        return angle_;
    }

    //! returns max allowed size of cone (radius in radians)
    inline float max_angle() const
    {
        return max_angle_;
    }

  private:
    OMVec3 center_normal_;
    float max_angle_; //< As a way to specify for each normal cone a maximum tolerance at creation time
    float angle_;
};

} // namespace c2m
#endif
