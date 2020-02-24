#include <crash2mesh/decimater/modules/mod_quadric.hpp>

#include <crash2mesh/util/logger.hpp>
#include <crash2mesh/core/structure_elements.hpp>

namespace c2m
{

ModQuadric::ModQuadric(CMesh& _mesh) : ModBase(_mesh, false)
{
    mesh_.add_property(quadrics_);
}

/// Destructor
ModQuadric::~ModQuadric()
{
    mesh_.remove_property(quadrics_);
}

void ModQuadric::initialize(void)
{
    using Quadric = OpenMesh::Geometry::Quadricf;

    // alloc quadrics
    if (!quadrics_.is_valid())
        mesh_.add_property(quadrics_);

    // clear quadrics
    Mesh::VertexIter v_it = mesh_.vertices_begin(), v_end = mesh_.vertices_end();

    for (; v_it != v_end; ++v_it)
        mesh_.property(quadrics_, *v_it).clear();

    // calc (normal weighted) quadric
    Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();

    Mesh::FaceHalfedgeCCWIter fh_it;
    Mesh::HalfedgeHandle heh[3];
    Mesh::VertexHandle vh[3];

    for (; f_it != f_end; ++f_it)
    {
        fh_it = mesh_.fh_ccwiter(*f_it);
        for (int i = 0; i < 3; i++)
        {
            heh[i] = *fh_it;
            vh[i] = mesh_.from_vertex_handle(*fh_it);
            ++fh_it;
        }

        OMVec3 points[3];
        std::vector<uint> frames(frame_seq());
        for (size_t i = 0; i < frames.size(); i++)
        {
            uint frame = frames[i];
            for (uint j = 0; j < 3; j++)
            {
                points[j] = OMVec3(mesh_.data(vh[j]).node->positions.coeff(frame, 0),
                                   mesh_.data(vh[j]).node->positions.coeff(frame, 1),
                                   mesh_.data(vh[j]).node->positions.coeff(frame, 2));
            }

            OMVec3 faceNormal = (points[1] - points[0]) % (points[2] - points[0]);
            double area = faceNormal.norm();
            if (area > FLT_MIN)
            {
                faceNormal /= area;
                area *= 0.5;
            }

            float a = faceNormal[0];
            float b = faceNormal[1];
            float c = faceNormal[2];
            float d = -(points[0] | faceNormal);

            Quadric q(a, b, c, d);
            // TODO area weighting yes or no?
            q *= dist2epicenter_f((points[0] + points[1] + points[2]) / 3.0, frame);

            for (uint j = 0; j < 3; j++)
            {
                std::vector<Quadric>& quadricPerFrame = mesh_.property(quadrics_, vh[j]);
                if (i == quadricPerFrame.size())
                {
                    quadricPerFrame.emplace_back(q);
                }
                else
                {
                    quadricPerFrame[i] += q;
                }
            }

            float strain = mesh_.data(*f_it).element->plasticStrains(frame);
            // Preserve special edges by assigning to them a plane perpendicular to the face
            for (uint j = 0; j < 3; j++)
            {
                float weightFactor = 0;
                if (mesh_.is_boundary(mesh_.opposite_halfedge_handle(heh[j])))
                {
                    if (MeshAnalyzer::dupes(mesh_, heh[j]).size() != 1)
                        continue;
                    // preserve boundary contours
                    weightFactor = 1.0;
                }
                else
                {
                    // preserve feature edges, across which plastic strains differ
                    Mesh::FaceHandle neighbor_fh = mesh_.face_handle(mesh_.opposite_halfedge_handle(heh[j]));
                    float neighbor_strain = mesh_.data(neighbor_fh).element->plasticStrains(frame);
                    if (abs(neighbor_strain - strain) < 0.005)
                        continue;
                    weightFactor = abs(neighbor_strain - strain) * 10000;
                }

                OMVec3 edgeNormal = (points[(j + 1) % 3] - points[j]) % faceNormal;
                double edgePlaneArea = edgeNormal.sqrnorm(); // n is length 1 so square the influence of boundary
                if (edgePlaneArea > FLT_MIN)
                {
                    edgeNormal /= sqrt(edgePlaneArea);
                    edgePlaneArea *= 0.5;
                }

                a = edgeNormal[0];
                b = edgeNormal[1];
                c = edgeNormal[2];
                d = -points[j] | edgeNormal;

                // TODO area weighting yes or no?
                Quadric qEdge(Quadric(a, b, c, d));
                qEdge *= weightFactor * dist2epicenter_f((points[(j + 1) % 3] + points[j]) / 2.0, frame);

                mesh_.property(quadrics_, vh[j])[i] += qEdge;
                mesh_.property(quadrics_, vh[(j + 1) % 3])[i] += qEdge;
            }
        }
    }
}

float ModQuadric::collapse_priority(const CollapseInfo& _ci)
{
    using Quadric = OpenMesh::Geometry::Quadricf;
    const std::vector<Quadric>& quadricsRemoved = mesh_.property(quadrics_, _ci.v0);
    const std::vector<Quadric>& quadricsRemaining = mesh_.property(quadrics_, _ci.v1);

    float error = 0;
    const MatX3& positions = mesh_.data(_ci.v1).node->positions;
    std::vector<uint> frames(frame_seq());
    for (size_t i = 0; i < frames.size(); i++)
    {
        uint frame = frames[i];
        Quadric q = quadricsRemaining[i] + quadricsRemoved[i];
        OMVec3 pointRemaining = OMVec3(positions.coeff(frame, 0), positions.coeff(frame, 1), positions.coeff(frame, 2));
        error = std::max(error, q(pointRemaining));
        if (error > max_err_)
            return Base::ILLEGAL_COLLAPSE;
    }

    return error;
}

void ModQuadric::postprocess_collapse(const CollapseInfo& _ci)
{
    using Quadric = OpenMesh::Geometry::Quadricf;
    const std::vector<Quadric>& quadricsRemoved = mesh_.property(quadrics_, _ci.v0);
    std::vector<Quadric>& quadricsRemaining = mesh_.property(quadrics_, _ci.v1);
    for (uint i = 0; i < quadricsRemaining.size(); i++)
    {
        quadricsRemaining[i] += quadricsRemoved[i];
    }
}

void ModQuadric::set_error_tolerance_factor(double _factor)
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

float ModQuadric::factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist)
{
    // TODO implement a proper function here
    float factor = 0.3f + 0.7f * ((pt - epicenter).norm() / mean_dist);
    return factor * factor;
}

} // namespace c2m
