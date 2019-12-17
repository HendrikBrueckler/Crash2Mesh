#ifndef C2M_FRAMEWISE_BOUNDARY_HPP
#define C2M_FRAMEWISE_BOUNDARY_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Geometry/NormalConeT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>

#include <float.h>
#include <vector>

namespace c2m
{

template <class MeshT> class ModFramewiseBoundaryT : public OpenMesh::Decimater::ModBaseT<MeshT>
{
  public:
    DECIMATING_MODULE(ModFramewiseBoundaryT, MeshT, FramewiseBoundaryDeviation);

    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Normal Normal;
    typedef typename Mesh::VertexHandle VertexHandle;
    typedef typename Mesh::FaceHandle FaceHandle;
    typedef typename Mesh::EdgeHandle EdgeHandle;

  public:
    /// Constructor
    ModFramewiseBoundaryT(MeshT& _mesh, float _max_angle = 45.0)
        : Base(_mesh, true), mesh_(Base::mesh()), frame_skip_(0)
    {
        set_boundary_angle(_max_angle);
    }

    /// Destructor
    ~ModFramewiseBoundaryT()
    {
    }

    /// Get normal deviation ( 0 .. 360 )
    Scalar boundary_angle() const
    {
        return boundary_angle_ / M_PI * 180.0;
    }

    /// Set normal deviation ( 0 .. 360 )
    void set_boundary_angle(Scalar _s)
    {
        boundary_angle_ = _s / static_cast<Scalar>(180.0) * static_cast<Scalar>(M_PI);
    }

    void initialize() override
    {
        if (Base::mesh().n_vertices() != 0)
            num_frames_ = Base::mesh().data(*Base::mesh().vertices_begin()).node->positions.rows();
        else
            num_frames_ = 0;
    }

    /** \brief Control normals when Decimating
     *
     * Binary and Cont. mode.
     *
     * The module tracks the normals while decimating
     * a normal cone consisting of all normals of the
     * faces collapsed together is computed and if
     * a collapse would increase the size of
     * the cone to a value greater than the given value
     * the collapse will be illegal.
     *
     * @param _ci Collapse info data
     * @return Half of the normal cones size (radius in radians)
     */
    float collapse_priority(const CollapseInfo& _ci) override
    {
        if (mesh_.is_boundary(_ci.v0v1) || mesh_.is_boundary(_ci.v1v0))
        {
            typename Mesh::VertexHandle vh2;
            if (mesh_.is_boundary(_ci.v0v1))
            {
                for (typename Mesh::HalfedgeHandle he : mesh_.vih_range(_ci.v0))
                {
                    if (mesh_.is_boundary(he))
                    {
                        vh2 = mesh_.from_vertex_handle(he);
                        break;
                    }
                }
            }
            else
            {
                for (typename Mesh::HalfedgeHandle he : mesh_.voh_range(_ci.v0))
                {
                    if (mesh_.is_boundary(he))
                    {
                        vh2 = mesh_.to_vertex_handle(he);
                        break;
                    }
                }
            }
            if (!vh2.is_valid())
                throw std::logic_error("ALARM");

            Node::Ptr nodes[3] = {mesh_.data(_ci.v0).node, mesh_.data(_ci.v1).node, mesh_.data(vh2).node};
            OMVec3 points[3];
            for (uint frame = 0; frame < num_frames_; frame += (1 + frame_skip_))
            {
                for (uint j = 0; j < 3; j++)
                {
                    points[j] = OMVec3(nodes[j]->positions.coeff(frame, 0),
                                       nodes[j]->positions.coeff(frame, 1),
                                       nodes[j]->positions.coeff(frame, 2));
                }
                if (acos((points[1] - points[0]).normalize() | (points[0] - points[2]).normalize()) > boundary_angle_)
                    return float(Base::ILLEGAL_COLLAPSE);
            }
        }

        return float(Base::LEGAL_COLLAPSE);
    }

    /// set the percentage of normal deviation
    void set_error_tolerance_factor(double _factor) override
    {
        if (_factor >= 0.0 && _factor <= 1.0)
        {
            // the smaller the factor, the smaller boundary_angle_ gets
            // thus creating a stricter constraint
            // division by error_tolerance_factor_ is for normalization
            Scalar boundary_angle_value
                = boundary_angle_ * static_cast<Scalar>(180.0 / M_PI * _factor / this->error_tolerance_factor_);

            set_boundary_angle(boundary_angle_value);
            this->error_tolerance_factor_ = _factor;
        }
    }

    void set_frame_skip(uint fs)
    {
        frame_skip_ = fs;
    }

    uint frame_skip() const
    {
        return frame_skip_;
    }

  private:
    Mesh& mesh_;
    uint frame_skip_;
    uint num_frames_;
    Scalar boundary_angle_;
};

} // namespace c2m

#endif // C2M_MOD_QUADRIC_HPP defined
