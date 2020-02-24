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

        update_frame_sequence();
    }

    /// Destructor
    ~ModBase()
    {
    }

    /**
     * @brief How many frames will be evaluated by the module
     */
    uint num_frames() const
    {
        return num_frames_;
    }

    /**
     * @brief Which frames will be evaluated by the module
     *
     * @return const std::vector<uint>& evaluated frame indices
     */
    const std::vector<uint>& frame_seq() const
    {
        return frame_sequence_;
    }

    /**
     * @brief Set frameskip interval relative to total frames (0..1)
     */
    void set_frame_skip(float _frame_skip_rel)
    {
        if (_frame_skip_rel >= 0.0 && _frame_skip_rel <= 1.0)
            frame_skip_ = std::max(1.0f, _frame_skip_rel * (num_frames_ - 1));
        else
            Logger::lout(Logger::ERROR) << "Specify frameskip relative to total frames (0..1)" << std::endl;
        update_frame_sequence();
    }

    /**
     * @brief Set number of frames to be evaluated, linearly spaced.
     */
    void set_num_frames(uint frames)
    {
        if (frames <= 1)
        {
            frame_skip_ = num_frames_;
            update_frame_sequence();
        }
        else
        {
            set_frame_skip(1.0f / (frames - 1));
        }
    }

    /**
     * @brief Frames skipped between two subsequent evaluated frames (1.0 means none, 2.0 is one etc.)
     */
    float frame_skip() const
    {
        return frame_skip_;
    }

    /**
     * @brief Set deformation epicenter variables
     *
     * @param _epicenters epicenter position for each frame
     * @param _mean_dists mean distance from epicenter for each frame
     */
    void set_epicenter_vars(const MatX3& _epicenters, VecX _mean_dists)
    {
        epicenters_ = _epicenters;
        mean_dists_ = _mean_dists;
    }

    /**
     * @brief Get mean distance from deformation epicenter for each frame
     */
    VecX mean_dists()
    {
        return mean_dists_;
    }

    /**
     * @brief Get epicenter position for each frame
     */
    MatX3 epicenters()
    {
        return epicenters_;
    }

    /**
     * @brief Base function to get weight of a points distance
     *        to the deformation epicenter of a specific frame.
     *
     * @param pt point for which a weight is returned
     * @param frame frame to consider
     * @return float weight of specified point (>= 0.0)
     */
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
    /**
     * @brief A weighting factor for submodules to override. Should be between about 0 and 5-10.
     *
     * @param pt point for which a weight is returned
     * @param epicenter epicenter
     * @param meanDist mean distance to epicenter
     * @return float weighting factor of \p p
     */
    virtual float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float meanDist) = 0;

    Mesh& mesh_;

  private:
    /**
     * @brief Update the frame sequence according to frame_skip_
     */
    void update_frame_sequence()
    {
        frame_sequence_.clear();
        for (float frameF = 0; frameF < num_frames(); frameF += frame_skip())
            frame_sequence_.emplace_back(std::floor(frameF));
    }

    float frame_skip_;
    uint num_frames_;
    std::vector<uint> frame_sequence_;

    MatX3 epicenters_;
    VecX mean_dists_;
};

} // namespace c2m

#endif
