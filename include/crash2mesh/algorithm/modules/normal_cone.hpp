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

    //! merge _cone; this instance will then enclose both former cones
    void merge(const NormalCone&);

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
    float max_angle_;
    float angle_;
};

//== IMPLEMENTATION ==========================================================

NormalCone::NormalCone(const OMVec3& _center_normal,  float _max_angle, float _angle) : center_normal_(_center_normal), max_angle_(_max_angle), angle_(_angle)
{
}

//----------------------------------------------------------------------------

void NormalCone::merge(const NormalCone& _cone)
{
    float dotp = (center_normal_ | _cone.center_normal_);
    max_angle_ = std::min(max_angle_, _cone.max_angle_);

    if (fabs(dotp) < 0.99999f)
    {
        // new angle
        float centerAngle = acos(dotp);
        float minAngle = std::min(-angle(), centerAngle - _cone.angle());
        float maxAngle = std::max(angle(), centerAngle + _cone.angle());
        angle_ = (maxAngle - minAngle) * float(0.5f);

        // axis by SLERP
        float axisAngle = float(0.5f) * (minAngle + maxAngle);
        center_normal_ = ((center_normal_ * sin(centerAngle - axisAngle) + _cone.center_normal_ * sin(axisAngle))
                          / sin(centerAngle));
    }
    else
    {
        // axes point in same direction
        if (dotp > 0.0f)
            angle_ = std::max(angle_, _cone.angle_);

        // axes point in opposite directions
        else
            angle_ = float(2.0f * M_PI);
    }
}

} // namespace c2m
#endif
