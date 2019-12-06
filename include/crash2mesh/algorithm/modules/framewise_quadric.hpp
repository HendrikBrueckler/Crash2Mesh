#ifndef C2M_FRAMEWISE_QUADRIC_HPP
#define C2M_FRAMEWISE_QUADRIC_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Geometry/QuadricT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>

#include <float.h>
#include <vector>

namespace c2m
{

/** \brief Mesh decimation module computing collapse priority based on error quadrics.
 *
 *  This module can be used as a binary and non-binary module.
 */
template <class MeshT> class ModFramewiseQuadricT : public OpenMesh::Decimater::ModBaseT<MeshT>
{
  public:
    // Defines the types Self, Handle, Base, Mesh, and CollapseInfo
    // and the memberfunction name()
    DECIMATING_MODULE(ModFramewiseQuadricT, MeshT, FramewiseQuadric);

  public:
    /** Constructor
     *  \internal
     */
    explicit ModFramewiseQuadricT(MeshT& _mesh) : Base(_mesh, false)
    {
        unset_max_err();
        Base::mesh().add_property(quadrics_);
    }

    /// Destructor
    virtual ~ModFramewiseQuadricT()
    {
        Base::mesh().remove_property(quadrics_);
    }

  public: // inherited
    /// Initalize the module and prepare the mesh for decimation.
    virtual void initialize(void) override
    {
        using Quadric = OpenMesh::Geometry::Quadricf;

        // alloc quadrics
        if (!quadrics_.is_valid())
            Base::mesh().add_property(quadrics_);

        // clear quadrics
        typename Mesh::VertexIter v_it = Base::mesh().vertices_begin(), v_end = Base::mesh().vertices_end();

        for (; v_it != v_end; ++v_it)
            Base::mesh().property(quadrics_, *v_it).clear();

        // calc (normal weighted) quadric
        typename Mesh::FaceIter f_it = Base::mesh().faces_begin(), f_end = Base::mesh().faces_end();

        typename Mesh::FaceVertexIter fv_it;
        typename Mesh::VertexHandle vh[3];

        for (; f_it != f_end; ++f_it)
        {
            fv_it = Base::mesh().fv_iter(*f_it);
            vh[0] = *fv_it;
            ++fv_it;
            vh[1] = *fv_it;
            ++fv_it;
            vh[2] = *fv_it;

            Node::Ptr node[3]
                = {Base::mesh().data(vh[0]).node, Base::mesh().data(vh[1]).node, Base::mesh().data(vh[2]).node};
            OMVec3 points[3] = {Base::mesh().point(vh[0]), Base::mesh().point(vh[1]), Base::mesh().point(vh[2])};
            Vec3 displacements[3];
            OMVec3 displacedPoints[3];
            for (uint i = 0; i < node[0]->displacements.rows(); i++)
            {
                for (uint j = 0; j < 3; j++)
                {
                    displacements[j] = node[j]->displacements.row(i);
                    displacedPoints[j]
                        = points[j] + OMVec3(displacements[j](0), displacements[j](1), displacements[j](2));
                }

                OMVec3 n = (displacedPoints[1] - displacedPoints[0]) % (displacedPoints[2] - displacedPoints[0]);
                double area = n.norm();
                if (area > FLT_MIN)
                {
                    n /= area;
                    area *= 0.5;
                }

                const double a = n[0];
                const double b = n[1];
                const double c = n[2];
                const double d = -displacedPoints[0] | n;

                Quadric q(a, b, c, d);
                q *= area;

                for (uint j = 0; j < 3; j++)
                {
                    std::vector<Quadric>& quadricPerFrame = Base::mesh().property(quadrics_, vh[j]);
                    if (i == quadricPerFrame.size())
                    {
                        quadricPerFrame.emplace_back(q);
                    }
                    else
                    {
                        quadricPerFrame[i] += q;
                    }
                }
            }
        }
    }

    /** Compute collapse priority based on error quadrics.
     *
     *  \see ModBaseT::collapse_priority() for return values
     *  \see set_max_err()
     */
    virtual float collapse_priority(const CollapseInfo& _ci) override
    {
        using Quadric = OpenMesh::Geometry::Quadricf;
        std::vector<Quadric> quadricsRemoved = Base::mesh().property(quadrics_, _ci.v0);
        std::vector<Quadric>& quadricsRemaining = Base::mesh().property(quadrics_, _ci.v1);

        VecX errors(quadricsRemaining.size());
        MatX3 displacements = Base::mesh().data(_ci.v1).node->displacements;
        OMVec3 pointRemaining;
        for (uint i = 0; i < quadricsRemaining.size(); i++)
        {
            quadricsRemaining[i] += quadricsRemoved[i];
            pointRemaining
                = _ci.p1 + OMVec3(displacements.coeff(i, 0), displacements.coeff(i, 1), displacements.coeff(i, 2));
            errors(i) = quadricsRemaining[i](pointRemaining);
        }

        float err = errors.maxCoeff();
        // float err = errors.mean();

        return float((err < max_err_) ? err : float(Base::ILLEGAL_COLLAPSE));
    }

    /// Post-process halfedge collapse (accumulate quadrics)
    virtual void postprocess_collapse(const CollapseInfo& _ci) override
    {
        using Quadric = OpenMesh::Geometry::Quadricf;
        std::vector<Quadric>& quadricsRemoved = Base::mesh().property(quadrics_, _ci.v0);
        std::vector<Quadric>& quadricsRemaining = Base::mesh().property(quadrics_, _ci.v1);
        for (uint i = 0; i < quadricsRemaining.size(); i++)
        {
            quadricsRemaining[i] += quadricsRemoved[i];
        }
    }

    /// set the percentage of maximum quadric error
    void set_error_tolerance_factor(double _factor) override
    {
        if (this->is_binary())
        {
            if (_factor >= 0.0 && _factor <= 1.0)
            {
                // the smaller the factor, the smaller max_err_ gets
                // thus creating a stricter constraint
                // division by error_tolerance_factor_ is for normalization
                double max_err = max_err_ * _factor / this->error_tolerance_factor_;
                set_max_err(max_err);
                this->error_tolerance_factor_ = _factor;

                initialize();
            }
        }
    }

  public: // specific methods
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

    /// Unset maximum quadric error constraint and restore non-binary mode.
    /// \see set_max_err()
    void unset_max_err(void)
    {
        max_err_ = DBL_MAX;
        Base::set_binary(false);
    }

    /// Return value of max. allowed error.
    double max_err() const
    {
        return max_err_;
    }

  private:
    // maximum quadric error
    double max_err_;

    // this vertex property stores a quadric for each frame for each vertex
    OpenMesh::VPropHandleT<std::vector<OpenMesh::Geometry::Quadricf>> quadrics_;
}; // namespace c2m

} // namespace c2m

#endif // C2M_MOD_QUADRIC_HPP defined
