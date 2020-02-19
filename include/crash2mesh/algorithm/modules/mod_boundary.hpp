#ifndef C2M_MOD_BOUNDARY_HPP
#define C2M_MOD_BOUNDARY_HPP

#include <crash2mesh/algorithm/modules/mod_base.hpp>

namespace c2m
{

class ModBoundary : public ModBase
{
  public:
    using Self = ModBoundary;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MBoundaryModule);

    /// Constructor
    ModBoundary(CMesh& _mesh, float _max_angle = 45.0) : ModBase(_mesh, true)
    {
        set_boundary_angle(_max_angle);
    }

    /// Destructor
    ~ModBoundary()
    {
    }

    /// Get normal deviation ( 0 .. 360 )
    float boundary_angle() const
    {
        return boundary_angle_ / M_PI * 180.0;
    }

    /// Set normal deviation ( 0 .. 360 )
    void set_boundary_angle(float _s)
    {
        boundary_angle_ = _s / 180.0 * M_PI;
    }

    void initialize() override
    {
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
        if ((mesh_.is_boundary(_ci.v0v1) || mesh_.is_boundary(_ci.v1v0))
            && MeshAnalyzer::dupes(Base::mesh(), _ci.v0v1).size() == 1
            && MeshAnalyzer::dupes(Base::mesh(), _ci.v1v0).size() == 1)
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
                throw std::logic_error("Error in calculation of followup boundary edge");

            Node::Ptr nodes[3] = {mesh_.data(_ci.v0).node, mesh_.data(_ci.v1).node, mesh_.data(vh2).node};
            OMVec3 points[3];
            for (uint frame : frame_seq())
            {
                for (uint j = 0; j < 3; j++)
                {
                    points[j] = OMVec3(nodes[j]->positions.coeff(frame, 0),
                                       nodes[j]->positions.coeff(frame, 1),
                                       nodes[j]->positions.coeff(frame, 2));
                }
                if (acos((points[1] - points[0]).normalize() | (points[0] - points[2]).normalize())
                    > boundary_angle_ * dist2epicenter_f((points[1] + points[0]) / 2.0, frame))
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
            float boundary_angle_value
                = boundary_angle_ * 180.0 / M_PI * _factor / this->error_tolerance_factor_;

            set_boundary_angle(boundary_angle_value);
            this->error_tolerance_factor_ = _factor;
        }
    }

  protected:
    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) override
    {
        // TODO implement a proper function here
        float factor = 0.3f + 0.7f * ((pt - epicenter).norm() / mean_dist);
        return factor * factor;
    }

    float boundary_angle_;
};

} // namespace c2m

#endif // C2M_MOD_QUADRIC_HPP defined
