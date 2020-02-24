#ifndef C2M_MOD_BOUNDARY_HPP
#define C2M_MOD_BOUNDARY_HPP

#include <crash2mesh/decimater/modules/mod_base.hpp>

namespace c2m
{

class ModBoundary : public ModBase
{
  public:
    using Self = ModBoundary;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MBoundaryModule);

    /// Constructor
    ModBoundary(CMesh& _mesh, float _max_angle = 45.0);

    /// Destructor
    ~ModBoundary()
    {
    }

    /**
     * @brief Get the maximum allowed angle between two subsequent boundary edges
     *        which may still be collapsed into one of their non-mutual vertices.
     */
    float boundary_angle() const
    {
        return max_boundary_angle_ / M_PI * 180.0;
    }

    /**
     * @brief Set the maximum allowed angle between two subsequent boundary edges
     *        which may still be collapsed into one of their non-mutual vertices.
     */
    void set_max_boundary_angle(float _s)
    {
        max_boundary_angle_ = _s / 180.0 * M_PI;
    }

    void initialize() override
    {
    }

    /** \brief Control normals when Decimating
     *
     * Binary mode.
     *
     * The module tracks the boundaries.
     * The angle between two subsequent boundary edges
     * which are to be collapsed into one of their
     * non-mutual vertices are compared to a maximum angle
     * weighted by the distance to the deformation epicenter.
     * If the angle is bigger than the maximum allowed angle
     * Base::ILLEGAL_COLLAPSE is returned.
     *
     * @param _ci Collapse info data
     * @return Base::ILLEGAL_COLLAPSE or Base::LEGAL_COLLAPSE
     */
    float collapse_priority(const CollapseInfo& _ci) override;

    /**
     * @brief Set error tolerance factor relative to current factor (0..1)
     */
    void set_error_tolerance_factor(double _factor) override;

  protected:
    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) override;

    float max_boundary_angle_;
};

} // namespace c2m

#endif // C2M_MOD_QUADRIC_HPP defined
