#ifndef C2M_MOD_NORMAL_HPP
#define C2M_MOD_NORMAL_HPP

#include <crash2mesh/decimater/modules/mod_base.hpp>
#include <crash2mesh/decimater/modules/normal_cone.hpp>

#include <OpenMesh/Core/Utils/Property.hh>

namespace c2m
{

class ModNormal : public ModBase
{
  public:
    using Self = ModNormal;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MNormalModule);

    /// Constructor
    ModNormal(CMesh& _mesh, float _max_dev = 180.0);

    /// Destructor
    ~ModNormal();

    /**
     * @brief Get maximum allowed normal deviation
     */
    float normal_deviation() const
    {
        return max_normal_deviation_ / M_PI * 180.0;
    }

    /**
     * @brief Set maximum allowed normal deviation
     */
    void set_max_normal_deviation(float _s)
    {
        max_normal_deviation_ = _s / 180.0 * M_PI;
    }
    /**
     * @brief Allocate and init normal cones
     */
    void initialize() override;

    /** \brief Control normals when Decimating
     *
     * Binary and Cont. mode.
     *
     * The module tracks the normals while decimating
     * a normal cone consisting of all normals of the
     * faces collapsed together is computed and if
     * a collapse would increase the size of
     * the cone to a value greater than the given value
     * the collapse will be illegal. Several frames are
     * considered.
     *
     * @param _ci Collapse info data
     * @return Half of the normal cones size (radius in radians)
     */
    float collapse_priority(const CollapseInfo& _ci) override;

    /**
     * @brief Set error tolerance factor relative to current factor (0..1)
     */
    void set_error_tolerance_factor(double _factor) override;

    /**
     * @brief Merge normal cones of merged faces
     *
     * @param _ci Collapse info data
     */
    void postprocess_collapse(const CollapseInfo& _ci) override;

  protected:
    /**
     * @brief Get the normal spanned by the 3 face points assuming CCW front-face
     */
    static OMVec3 face_normal(Vec3 p0, Vec3 p1, Vec3 p2);

    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) override;

  private:
    float max_normal_deviation_;
    OpenMesh::FPropHandleT<std::vector<NormalCone>> normal_cones_;
};

} // namespace c2m

#endif
