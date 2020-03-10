#ifndef C2M_MOD_QUADRIC_NORMAL_HPP
#define C2M_MOD_QUADRIC_NORMAL_HPP

#include <crash2mesh/decimater/modules/mod_base.hpp>
#include <crash2mesh/decimater/modules/mod_quadric.hpp>
#include <crash2mesh/decimater/modules/mod_normal.hpp>

namespace c2m
{

class ModQuadricNormal : public ModQuadric, public ModNormal
{
  public:
    using Base = OpenMesh::Decimater::ModBaseT<CMesh>;
    using Mesh = typename Base::Mesh;
    using CollapseInfo = typename Base::CollapseInfo;
    using Self = ModQuadricNormal;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MQuadricNormalModule);

    /// Constructor
    ModQuadricNormal(CMesh& _mesh, float _max_err = FLT_MAX, float _max_dev = FLT_MAX);

    /// Destructor
    ~ModQuadricNormal();

    /**
     * @brief TODO
     */
    void initialize() override;

    /**
     * TODO
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
     * @brief TODO
     *
     * @param _ci Collapse info data
     */
    void postprocess_collapse(const CollapseInfo& _ci) override;

  protected:
    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) const override;

    Vec3 bbox_min;
    Vec3 bbox_max;
    float bbox_diag;
    float bbox_vol;
};

} // namespace c2m

#endif
