#ifndef C2M_MOD_QUADRIC_HPP
#define C2M_MOD_QUADRIC_HPP

#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/decimater/modules/mod_base.hpp>

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

    /**
     * @brief Enable or disable the weighting of quadrics with face area
     */
    void set_area_weighting(bool _area_weighting)
    {
        area_weighting_ = _area_weighting;
    }

    /**
     * @brief Enable or disable the weighting of quadrics with face area
     */
    void set_optimize_position(bool _optimize_position)
    {
        optimize_position_ = _optimize_position;
    }

    /**
     * @brief Calculate the optimal position from solving A * x = b (A, b from quadric)
     *
     * @param q Quadric
     * @param optimalPos optimal position will be returned in this parameter.
     *      initialize with reference point for least-distance solution wrt that point.
     * @return true if system could be solved
     * @return false else
     */
    bool optimal_position(Quadric& q, Vec3& optimalPos) const;

  protected:
    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) const override;

    /**
     * @brief Calculate the quadric of a triangle (p, q, r) at a certain frame
     *
     * @param frame
     * @param p first corner
     * @param q second corner
     * @param r third corner
     * @return Quadric quadric of (p, q, r)
     */
    Quadric calc_face_quadric(uint frame, const Vec3& p, const Vec3& q, const Vec3& r) const;

    /**
     * @brief Calculate the quadric of a halfedge (p, q) at a certain frame.
     *        r is the opposing vertex of the triangle.
     *
     * @param he halfedge
     * @param frame
     * @param p first edge vertex
     * @param q second edge vertex
     * @param r opposing triangle vertex
     * @return Quadric plane quadric through (p, q), perpendicular to (p, q, r)
     */
    Quadric calc_edge_quadric(HEHandle he, uint frame, const Vec3& p, const Vec3& q, const Vec3& r) const;

  private:
    double max_err_;
    bool area_weighting_;
    bool optimize_position_;

#ifdef C2M_PROB_QUADRICS
    /**
     * @brief Utility method to get the crossproduct matrix of vector v
     */
    Mat3 crossProductMatrix(const Vec3& v) const;

    /**
     * @brief Utility method to ge the crossinterference matrix of two covariance matrices
     */
    Mat3 crossInterferenceMatrix(const Mat3& A, const Mat3& B) const;

    /**
     * @brief Calculates the gaussian probabilistic quadric of a triangle (p, q, r)
     * Values for Covariances are hardcoded.
     */
    Quadric probabilisticTriQuadric(const Vec3& p, const Vec3& q, const Vec3& r) const;

    /**
     * @brief Calculates the gaussian probabilistic quadric of a triangle (p, q, r)
     * Values for Covariances are hardcoded.
     */
    Quadric probabilisticTriQuadric(const OMVec3& p, const OMVec3& q, const OMVec3& r) const;

    /**
     * @brief Calculates the gaussian probabilistic quadric of a plane through (p, q, r)
     * Values for Covariances are hardcoded.
     */
    Quadric probabilisticPlaneQuadric(const Vec3& p, const Vec3& q, const Vec3& r) const;

    /**
     * @brief Calculates the gaussian probabilistic quadric of a plane through (p, q, r)
     * Values for Covariances are hardcoded.
     */
    Quadric probabilisticPlaneQuadric(const OMVec3& p, const OMVec3& q, const OMVec3& r) const;
#endif
}; // namespace c2m

} // namespace c2m

#endif
