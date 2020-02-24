#include <crash2mesh/decimater/modules/mod_base.hpp>

#include <crash2mesh/util/logger.hpp>

namespace c2m
{

ModBase::ModBase(CMesh& _mesh, bool binary) : Base(_mesh, binary), mesh_(_mesh), frame_skip_(1.0)
{
    if (mesh_.n_vertices() != 0 || !mesh_.data(*mesh_.vertices_begin()).node)
        num_frames_ = static_cast<uint>(mesh_.data(*mesh_.vertices_begin()).node->positions.rows());
    else
        num_frames_ = 0;

    update_frame_sequence();
}

void ModBase::set_frame_skip(float _frame_skip_rel)
{
    if (_frame_skip_rel >= 0.0 && _frame_skip_rel <= 1.0)
        frame_skip_ = std::max(1.0f, _frame_skip_rel * (num_frames_ - 1));
    else
        Logger::lout(Logger::ERROR) << "Specify frameskip relative to total frames (0..1)" << std::endl;
    update_frame_sequence();
}

void ModBase::set_num_frames(uint frames)
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

float ModBase::dist2epicenter_f(Point pt, uint frame)
{
    if (epicenters_.size() == 0 || mean_dists_.size() == 0 || epicenters_.row(frame).squaredNorm() == 0.0)
        return 1.0;

    float factor
        = factor_dist_to_epicenter(Vec3(pt[0], pt[1], pt[2]), epicenters_.row(frame).transpose(), mean_dists_[frame]);
    assert(factor >= 0.0);
    return factor;
}

void ModBase::update_frame_sequence()
{
    frame_sequence_.clear();
    for (float frameF = 0; frameF < num_frames(); frameF += frame_skip())
        frame_sequence_.emplace_back(std::floor(frameF));
}

} // namespace c2m
