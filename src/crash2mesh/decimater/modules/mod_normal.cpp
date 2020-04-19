#include <crash2mesh/decimater/modules/mod_normal.hpp>

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/util/logger.hpp>

namespace c2m
{

/// Constructor
ModNormal::ModNormal(CMesh& _mesh, float _max_dev) : ModBase(_mesh, true)
{
    set_max_normal_deviation(_max_dev);
}

/// Destructor
ModNormal::~ModNormal()
{
}

void ModNormal::initialize()
{
    // alloc quadrics if not already there
    bool preDefined = false;
    bool preEmpty = false;
    for (FHandle f : mesh_.faces())
    {
        if (mesh_.data(f).normalCones.empty())
        {
            preEmpty = true;
            if (preDefined)
            {
                throw std::logic_error("Some normalCones allocated, others aren't, can't handle this.");
            }
        }
        else
        {
            if (mesh_.data(f).normalCones.size() != frame_seq().size())
            {
                throw std::logic_error("Unexpected number of normalCones per vertex (!= number of frames).");
            }
            preDefined = true;
            if (preEmpty)
            {
                throw std::logic_error("Some normalCones allocated, others aren't, can't handle this.");
            }
        }
    }
    if (preDefined)
        return;

    Mesh::FaceVertexCCWIter fv_it;

    for (FHandle f : mesh_.all_faces())
    {
        fv_it = mesh_.fv_ccwbegin(f);
        const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
        const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
        const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
        std::vector<NormalCone> normalCones;
        std::vector<uint> frames(frame_seq());
        for (size_t i = 0; i < frames.size(); i++)
        {
            uint frame = frames[i];
            Vec3 p0(positions0.row(frame).transpose());
            Vec3 p1(positions1.row(frame).transpose());
            Vec3 p2(positions2.row(frame).transpose());
            OMVec3 pt0(p0[0], p0[1], p0[2]);
            OMVec3 pt1(p1[0], p1[1], p1[2]);
            OMVec3 pt2(p2[0], p2[1], p2[2]);
            OMVec3 n = face_normal(p0, p1, p2);
            normalCones.emplace_back(n);
            // TODO do we need min(1.0f, ...) here?
        }
        mesh_.data(f).normalCones = normalCones;
    }
}

float ModNormal::collapse_priority(const CollapseInfo& _ci)
{
    float max_angle(0.0f);
    Mesh::FaceHandle fh, fhl, fhr;
    Mesh::ConstVertexFaceIter vf_it;
    Mesh::FaceVertexCCWIter fv_it;


    std::vector<uint> frames(frame_seq());
    std::vector<NormalCone> nc;
    const MatX3 *positions0, *positions1, *positions2;
    OpenMesh::HPropHandleT<MatX3> collapseTargets;
    bool optimalPos = mesh_.get_property_handle(collapseTargets, "collapseTargets");
    const MatX3* remainingPos
        = optimalPos ? &mesh_.property(collapseTargets, _ci.v0v1) : &mesh_.data(_ci.v1).node->positions;

    for (VHandle vCenter : (optimalPos ? std::vector<VHandle>({_ci.v0, _ci.v1}) : std::vector<VHandle>({_ci.v0})))
    {
        if (vCenter == _ci.v0)
        {
            if (_ci.v0vl.is_valid())
                fhl = mesh_.face_handle(_ci.v0vl);
            if (_ci.vrv0.is_valid())
                fhr = mesh_.face_handle(_ci.vrv0);
        }
        else
        {
            if (_ci.vlv1.is_valid())
                fhl = mesh_.face_handle(_ci.vlv1);
            if (_ci.v1vr.is_valid())
                fhr = mesh_.face_handle(_ci.v1vr);
        }
        for (vf_it = mesh_.vf_begin(vCenter); vf_it.is_valid(); ++vf_it)
        {
            fh = *vf_it;
            if (fh == _ci.fl || fh == _ci.fr)
                continue;

            nc = mesh_.data(fh).normalCones;
            fv_it = mesh_.fv_ccwbegin(fh);
            // simulate position change
            positions0
                = (*fv_it == vCenter) ? remainingPos : &mesh_.data(*fv_it).node->positions;
            fv_it++;
            positions1
                = (*fv_it == vCenter) ? remainingPos : &mesh_.data(*fv_it).node->positions;
            fv_it++;
            positions2
                = (*fv_it == vCenter) ? remainingPos : &mesh_.data(*fv_it).node->positions;
            fv_it++;
            for (size_t i = 0; i < frames.size(); i++)
            {
                uint frame = frames[i];
                OMVec3 n = face_normal(positions0->row(frame).transpose(),
                                    positions1->row(frame).transpose(),
                                    positions2->row(frame).transpose());
                nc[i].merge(NormalCone(n));
                if (fh == fhl)
                    nc[i].merge(mesh_.data(_ci.fl).normalCones[i]);
                if (fh == fhr)
                    nc[i].merge(mesh_.data(_ci.fr).normalCones[i]);

                Vec3 pos = (positions0->row(frame).transpose() + positions1->row(frame).transpose() + positions2->row(frame).transpose()) / 3.0f;
                float dist2epifactor = dist2epicenter_f(pos, frame);

                // Priority
                if (nc[i].angle() * dist2epifactor > max_angle)
                {
                    max_angle = nc[i].angle() * dist2epifactor;
                    // Legality
                    if (max_angle > 0.5 * max_normal_deviation_)
                        return float(Base::ILLEGAL_COLLAPSE);
                }
            }
        }
    }

    return max_angle;
}

void ModNormal::set_error_tolerance_factor(double _factor)
{
    if (_factor >= 0.0 && _factor <= 1.0)
    {
        // the smaller the factor, the smaller max_normal_deviation_ gets
        // thus creating a stricter constraint
        // division by error_tolerance_factor_ is for normalization
        float max_normal_deviation_value
            = max_normal_deviation_ * 180.0f / M_PI * _factor / this->error_tolerance_factor_;

        set_max_normal_deviation(max_normal_deviation_value);
        this->error_tolerance_factor_ = _factor;
    }
}

void ModNormal::postprocess_collapse(const CollapseInfo& _ci)
{
    Mesh::ConstVertexFaceIter vf_it(mesh_, _ci.v0);
    Mesh::FaceHandle fh, fhl, fhr;

    if (_ci.v0vl.is_valid())
        fhl = mesh_.face_handle(_ci.v0vl);
    if (_ci.vrv0.is_valid())
        fhr = mesh_.face_handle(_ci.vrv0);

    for (; vf_it.is_valid(); ++vf_it)
    {
        fh = *vf_it;
        if (fh == _ci.fl || fh == _ci.fr)
            continue;

        std::vector<NormalCone>& ncs = mesh_.data(fh).normalCones;
        Mesh::FaceVertexCCWIter fv_it = mesh_.fv_ccwbegin(fh);
        const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
        const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
        const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
        for (uint frame = 0; frame < num_frames(); frame++)
        {
            OMVec3 n = face_normal(positions0.row(frame).transpose(),
                                   positions1.row(frame).transpose(),
                                   positions2.row(frame).transpose());
            ncs[frame].merge(NormalCone(n));
            if (fh == fhl)
                ncs[frame].merge(mesh_.data(_ci.fl).normalCones[frame]);
            if (fh == fhr)
                ncs[frame].merge(mesh_.data(_ci.fl).normalCones[frame]);
        }
    }
}

OMVec3 ModNormal::face_normal(Vec3 p0, Vec3 p1, Vec3 p2)
{
    OMVec3 pt0(p0[0], p0[1], p0[2]);
    OMVec3 pt1(p1[0], p1[1], p1[2]);
    OMVec3 pt2(p2[0], p2[1], p2[2]);
    OMVec3 n = cross(pt2 - pt1, pt0 - pt1);
    float length = norm(n);
    return (length > 0.0) ? n * (1.0f / length) : Normal(0, 0, 0);
}

} // namespace c2m
