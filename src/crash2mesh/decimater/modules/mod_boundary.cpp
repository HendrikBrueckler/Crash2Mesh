#include <crash2mesh/decimater/modules/mod_boundary.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>

namespace c2m
{

ModBoundary::ModBoundary(CMesh& _mesh, float _max_angle) : ModBase(_mesh, true)
{
    set_max_boundary_angle(_max_angle);
}

float ModBoundary::collapse_priority(const CollapseInfo& _ci)
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
                > max_boundary_angle_ * dist2epicenter_f((points[1] + points[0]) / 2.0, frame))
                return float(Base::ILLEGAL_COLLAPSE);
        }
    }

    return float(Base::LEGAL_COLLAPSE);
}

void ModBoundary::set_error_tolerance_factor(double _factor)
{
    if (_factor >= 0.0 && _factor <= 1.0)
    {
        // the smaller the factor, the smaller max_boundary_angle_ gets
        // thus creating a stricter constraint
        // division by error_tolerance_factor_ is for normalization
        float max_boundary_angle_value = max_boundary_angle_ * 180.0 / M_PI * _factor / this->error_tolerance_factor_;

        set_max_boundary_angle(max_boundary_angle_value);
        this->error_tolerance_factor_ = _factor;
    }
}

float ModBoundary::factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist)
{
    // TODO implement a proper function here
    float factor = 0.3f + 0.7f * ((pt - epicenter).norm() / mean_dist);
    return factor * factor;
}

} // namespace c2m
