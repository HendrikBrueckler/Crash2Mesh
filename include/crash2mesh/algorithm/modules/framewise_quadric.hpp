#ifndef C2M_FRAMEWISE_QUADRIC_HPP
#define C2M_FRAMEWISE_QUADRIC_HPP

#include <crash2mesh/algorithm/mesh_analyzer.hpp>
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
    explicit ModFramewiseQuadricT(MeshT& _mesh) : Base(_mesh, false), mesh_(_mesh), frame_skip_(0)
    {
        mesh_.add_property(quadrics_);
    }

    /// Destructor
    virtual ~ModFramewiseQuadricT()
    {
        mesh_.remove_property(quadrics_);
    }

  public: // inherited
    /// Initalize the module and prepare the mesh for decimation.
    virtual void initialize(void) override
    {
        using Quadric = OpenMesh::Geometry::Quadricf;

        if (mesh_.n_vertices() != 0)
            num_frames_ = static_cast<uint>(mesh_.data(*mesh_.vertices_begin()).node->positions.rows());
        else
            num_frames_ = 0;

        // alloc quadrics
        if (!quadrics_.is_valid())
            mesh_.add_property(quadrics_);

        // clear quadrics
        typename Mesh::VertexIter v_it = mesh_.vertices_begin(), v_end = mesh_.vertices_end();

        for (; v_it != v_end; ++v_it)
            mesh_.property(quadrics_, *v_it).clear();

        // calc (normal weighted) quadric
        typename Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();

        typename Mesh::FaceHalfedgeCCWIter fh_it;
        typename Mesh::HalfedgeHandle heh[3];
        typename Mesh::VertexHandle vh[3];

        for (; f_it != f_end; ++f_it)
        {
            fh_it = mesh_.fh_ccwiter(*f_it);
            heh[0] = *fh_it;
            vh[0] = mesh_.from_vertex_handle(*fh_it);
            ++fh_it;
            heh[1] = *fh_it;
            vh[1] = mesh_.from_vertex_handle(*fh_it);
            ++fh_it;
            heh[2] = *fh_it;
            vh[2] = mesh_.from_vertex_handle(*fh_it);

            OMVec3 points[3];
            for (uint frame = 0, i = 0; frame < num_frames_; frame += (1 + frame_skip_), i++)
            {
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
                q *= weight_dist_to_epicenter((points[0] + points[1] + points[2]) / 3.0, frame);

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
                        // preserve boundary edges
                        weightFactor = 1.0;
                    }
                    else
                    {
                        // preserve feature edges, across which plastic strains differ
                        typename Mesh::FaceHandle neighbor_fh
                            = mesh_.face_handle(mesh_.opposite_halfedge_handle(heh[j]));
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

                    Quadric qBoundary(Quadric(a, b, c, d));
                    qBoundary *= weightFactor * weight_dist_to_epicenter((points[(j + 1) % 3] + points[j]) / 2.0, frame);;
                    // area weighting yes or no?

                    mesh_.property(quadrics_, vh[j])[i] += qBoundary;
                    mesh_.property(quadrics_, vh[(j + 1) % 3])[i] += qBoundary;
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
        const std::vector<Quadric>& quadricsRemoved = mesh_.property(quadrics_, _ci.v0);
        const std::vector<Quadric>& quadricsRemaining = mesh_.property(quadrics_, _ci.v1);

        float error = 0;
        const MatX3& positions = mesh_.data(_ci.v1).node->positions;
        for (uint frame = 0, i = 0; frame < num_frames_; frame += 1 + frame_skip_, i++)
        {
            Quadric q = quadricsRemaining[i] + quadricsRemoved[i];
            OMVec3 pointRemaining
                = OMVec3(positions.coeff(frame, 0), positions.coeff(frame, 1), positions.coeff(frame, 2));
            error = std::max(error, q(pointRemaining));
            if (error > max_err_)
                return Base::ILLEGAL_COLLAPSE;
        }

        return error;
    }

    /// Post-process halfedge collapse (accumulate quadrics)
    virtual void postprocess_collapse(const CollapseInfo& _ci) override
    {
        using Quadric = OpenMesh::Geometry::Quadricf;
        const std::vector<Quadric>& quadricsRemoved = mesh_.property(quadrics_, _ci.v0);
        std::vector<Quadric>& quadricsRemaining = mesh_.property(quadrics_, _ci.v1);
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

    void set_frame_skip(uint fs)
    {
        frame_skip_ = fs;
    }

    uint frame_skip() const
    {
        return frame_skip_;
    }

    void set_epicenter_vars(const MatX3& _epicenters, VecX _mean_dists)
    {
        epicenters_ = _epicenters;
        mean_dists_ = _mean_dists;
    }

    VecX mean_dists()
    {
        return mean_dists_;
    }

    MatX3 epicenters()
    {
        return epicenters_;
    }

    float weight_dist_to_epicenter(OMVec3 pt, uint frame)
    {
        if (epicenters_.size() == 0 || mean_dists_.size() == 0 || epicenters_.row(frame).squaredNorm() < 1e-10)
            return 1.0;
        Vec3 p(pt[0], pt[1], pt[2]);
        // TODO implement a proper function here
        float factor = (0.3f + 0.7f * ((p - epicenters_.row(frame).transpose()).norm() / mean_dists_[frame]));
        return 1.0f / (factor * factor);
    }

  private:
    Mesh& mesh_;
    // maximum quadric error
    double max_err_;

    MatX3 epicenters_;
    VecX mean_dists_;

    uint num_frames_;
    uint frame_skip_;

    // this vertex property stores a quadric for each frame for each vertex
    OpenMesh::VPropHandleT<std::vector<OpenMesh::Geometry::Quadricf>> quadrics_;
}; // namespace c2m

} // namespace c2m

#endif // C2M_FRAMEWISE_QUADRIC_HPP defined
