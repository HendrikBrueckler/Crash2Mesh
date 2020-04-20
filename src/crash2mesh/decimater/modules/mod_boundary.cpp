#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/decimater/modules/mod_boundary.hpp>

namespace c2m
{

ModBoundary::ModBoundary(CMesh& _mesh, float _max_angle) : ModBase(_mesh, true)
{
    set_max_boundary_angle(_max_angle);
}

float ModBoundary::collapse_priority(const CollapseInfo& _ci)
{
    if ((mesh_.is_boundary(_ci.v0v1) || mesh_.is_boundary(_ci.v1v0))
        /*&& MeshAnalyzer::dupes(Base::mesh(), _ci.v0v1).size() == 1
        && MeshAnalyzer::dupes(Base::mesh(), _ci.v1v0).size() == 1*/)
    {
        typename Mesh::VertexHandle vhPre, vhPost;
        if (mesh_.is_boundary(_ci.v0v1))
        {
            for (typename Mesh::HalfedgeHandle he : mesh_.vih_range(_ci.v0))
            {
                if (mesh_.is_boundary(he))
                {
                    vhPre = mesh_.from_vertex_handle(he);
                    break;
                }
            }
            for (typename Mesh::HalfedgeHandle he : mesh_.voh_range(_ci.v1))
            {
                if (mesh_.is_boundary(he))
                {
                    vhPost = mesh_.to_vertex_handle(he);
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
                    vhPre = mesh_.to_vertex_handle(he);
                    break;
                }
            }
            for (typename Mesh::HalfedgeHandle he : mesh_.vih_range(_ci.v1))
            {
                if (mesh_.is_boundary(he))
                {
                    vhPost = mesh_.from_vertex_handle(he);
                    break;
                }
            }
        }
        if (!vhPre.is_valid() || !vhPost.is_valid() || vhPre == vhPost)
            throw std::logic_error("Error in calculation of followup boundary edge");

        Node::Ptr nodes[4]
            = {mesh_.data(vhPre).node, mesh_.data(_ci.v0).node, mesh_.data(_ci.v1).node, mesh_.data(vhPost).node};
        OpenMesh::HPropHandleT<MatX3> collapseTargets;
        bool optimalPos = mesh_.get_property_handle(collapseTargets, "collapseTargets");
        const MatX3& remainingPos
            = optimalPos ? mesh_.property(collapseTargets, _ci.v0v1) : mesh_.data(_ci.v1).node->positions;
        if (remainingPos.rows() < num_frames())
            throw std::logic_error("Error in calculation of followup boundary edge");
        for (uint frame : frame_seq())
        {
            if (!optimalPos)
            {
                float leftAngle = acos((nodes[2]->positions.row(frame) - nodes[0]->positions.row(frame))
                                       * (nodes[1]->positions.row(frame) - nodes[0]->positions.row(frame)).transpose());
                float rightAngle
                    = acos((nodes[2]->positions.row(frame) - nodes[0]->positions.row(frame))
                           * (nodes[2]->positions.row(frame) - nodes[1]->positions.row(frame)).transpose());
                if (std::max(leftAngle, rightAngle) > max_boundary_angle_)
                    return float(Base::ILLEGAL_COLLAPSE);
            }
            else
            {
                float leftAngle1
                    = acos((remainingPos.row(frame) - nodes[0]->positions.row(frame))
                           * (nodes[1]->positions.row(frame) - nodes[0]->positions.row(frame)).transpose());
                float leftAngle2
                    = acos((remainingPos.row(frame) - nodes[3]->positions.row(frame))
                           * (nodes[2]->positions.row(frame) - nodes[3]->positions.row(frame)).transpose());
                float rightAngle1 = acos((remainingPos.row(frame) - nodes[0]->positions.row(frame))
                                         * (remainingPos.row(frame) - nodes[1]->positions.row(frame)).transpose());
                float rightAngle2 = acos((remainingPos.row(frame) - nodes[3]->positions.row(frame))
                                         * (remainingPos.row(frame) - nodes[2]->positions.row(frame)).transpose());
                if (std::max(std::max(leftAngle1, rightAngle1), std::max(leftAngle2, rightAngle2))
                    > max_boundary_angle_)
                    return float(Base::ILLEGAL_COLLAPSE);
            }
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

} // namespace c2m
