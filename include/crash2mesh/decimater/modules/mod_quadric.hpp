#ifndef C2M_MOD_QUADRIC_HPP
#define C2M_MOD_QUADRIC_HPP

#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/decimater/modules/mod_base.hpp>
#include <crash2mesh/core/types.hpp>

#include <OpenMesh/Core/Geometry/QuadricT.hh>
#include <OpenMesh/Core/Utils/Property.hh>

#include <float.h>

namespace c2m
{

/** \brief Mesh decimation module computing collapse priority based on error quadrics.
 *
 *  This module can be used as a binary and non-binary module.
 */
class ModQuadric : public ModBase
{
  public:
    using Self = ModQuadric;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MQuadricModule);

    explicit ModQuadric(CMesh& _mesh);

    /// Destructor
    virtual ~ModQuadric();

    /**
     * Calculate all quadrics.
     * Weight by distance to epicenter.
     * Include feature edge quadrics for high strain deviation.
     * Include boundary edge quadrics.
     * TODO currently face area weighting disabled permanently
     */
    virtual void initialize(void) override;

    /** \brief Control normals when Decimating
     *
     * Binary and Cont. mode.
     *
     * The module accumulates quadrics while decimating.
     * If the quadric error is greater than the set max error,
     * Base::ILLEGAL_COLLAPSE is returned.
     * \see initialize() for quadric computation.
     *
     * @param _ci Collapse info data
     * @return quadric error or BASE::ILLEGAL_COLLAPSE
     */
    virtual float collapse_priority(const CollapseInfo& _ci) override;

    /**
     * @brief Post-process halfedge collapse (accumulate quadrics)
     */
    virtual void postprocess_collapse(const CollapseInfo& _ci) override;

    /**
     * @brief Set error tolerance factor relative to current factor (0..1)
     */
    void set_error_tolerance_factor(double _factor) override;

    /** Set maximum quadric error constraint and enable binary mode.
     *  \param _err    Maximum error allowed
     *  \param _binary Let the module work in non-binary mode in spite of the
     *                 enabled constraint.
     *  \see unset_max_err()
     */
    void set_max_err(double _err, bool _binary = true)
    {
        max_err_ = _err;
        Base::set_binary(_binary);
    }

    /**
     * @brief Unset maximum quadric error constraint and restore non-binary mode.
     * \see set_max_err()
     */
    void unset_max_err(void)
    {
        max_err_ = DBL_MAX;
        Base::set_binary(false);
    }

    /**
     * @brief Return value of max. allowed error.
     */
    double max_err() const
    {
        return max_err_;
    }

  protected:
    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) override;

  private:
    double max_err_;

    // this vertex property stores a quadric for each frame for each vertex
    OpenMesh::VPropHandleT<std::vector<OpenMesh::Geometry::Quadricf>> quadrics_;
}; // namespace c2m

} // namespace c2m

#endif
