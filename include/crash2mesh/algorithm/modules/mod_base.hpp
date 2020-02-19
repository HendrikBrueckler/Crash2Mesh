#ifndef C2M_BASE_MODULE_HPP
#define C2M_BASE_MODULE_HPP

#include <OpenMesh/Tools/Decimater/ModBaseT.hh>
#include <crash2mesh/core/mesh.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/util/logger.hpp>

namespace c2m
{

class ModBase : public OpenMesh::Decimater::ModBaseT<CMesh>
{
  public:
    using Base = OpenMesh::Decimater::ModBaseT<CMesh>;
    using Mesh = typename Base::Mesh;
    using CollapseInfo = typename Base::CollapseInfo;

    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Normal Normal;
    typedef typename Mesh::VertexHandle VertexHandle;
    typedef typename Mesh::FaceHandle FaceHandle;
    typedef typename Mesh::EdgeHandle EdgeHandle;

  public:
    /// Constructor
    ModBase(CMesh& _mesh, bool binary) : Base(_mesh, binary), mesh_(_mesh), frame_skip_(1.0)
    {
        if (mesh_.n_vertices() != 0 || !mesh_.data(*mesh_.vertices_begin()).node)
            num_frames_ = static_cast<uint>(mesh_.data(*mesh_.vertices_begin()).node->positions.rows());
        else
            num_frames_ = 0;
    }

    /// Destructor
    ~ModBase()
    {
    }

    uint num_frames() const
    {
        return num_frames_;
    }

    std::vector<uint> frame_seq() const
    {
        std::vector<uint> fs;
        for (float frameF = 0; frameF < num_frames(); frameF += frame_skip())
            fs.emplace_back(std::floor(frameF));
        return fs;
    }

    // Specify frameskip interval relative to total frames (0..1)
    void set_frame_skip(float _frame_skip_rel)
    {
        if (_frame_skip_rel >= 0.0 && _frame_skip_rel <= 1.0)
            frame_skip_ = std::max(1.0f, _frame_skip_rel * (num_frames_ - 1));
        else
            Logger::lout(Logger::ERROR) << "Specify frameskip relative to total frames (0..1)" << std::endl;
    }

    // Specify total number of frames
    void set_frames(uint frames)
    {
        if (frames <= 1)
            frame_skip_ = num_frames_;
        else
            set_frame_skip(1.0f / (frames - 1));
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

    float dist2epicenter_f(Point pt, uint frame)
    {
        if (epicenters_.size() == 0 || mean_dists_.size() == 0 || epicenters_.row(frame).squaredNorm() == 0.0)
            return 1.0;

        float factor = factor_dist_to_epicenter(
            Vec3(pt[0], pt[1], pt[2]), epicenters_.row(frame).transpose(), mean_dists_[frame]);
        assert(factor >= 0.0);
        return factor;
    }

  protected:
    virtual float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float meanDist) = 0;

    Mesh& mesh_;

  private:
    uint frame_skip_;
    uint num_frames_;

    MatX3 epicenters_;
    VecX mean_dists_;
};

} // namespace c2m

#endif
