#ifndef C2M_MOD_NORMAL_HPP
#define C2M_MOD_NORMAL_HPP

#include <crash2mesh/algorithm/modules/mod_base.hpp>
#include <crash2mesh/algorithm/modules/normal_cone.hpp>
#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Utils/Property.hh>

namespace c2m
{

class ModNormal : public ModBase
{
  public:
    using Self = ModNormal;
    using Handle = OpenMesh::Decimater::ModHandleT<Self>;
    DECIMATER_MODNAME(C2MNormalModule);

    /// Constructor
    ModNormal(CMesh& _mesh, float _max_dev = 180.0) : ModBase(_mesh, true)
    {
        set_normal_deviation(_max_dev);
        mesh_.add_property(normal_cones_);
    }

    /// Destructor
    ~ModNormal()
    {
        mesh_.remove_property(normal_cones_);
    }

    /// Get normal deviation ( 0 .. 360 )
    float normal_deviation() const
    {
        return normal_deviation_ / M_PI * 180.0;
    }

    /// Set normal deviation ( 0 .. 360 )
    void set_normal_deviation(float _s)
    {
        normal_deviation_ = _s / 180.0 * M_PI;
    }

    /// Allocate and init normal cones
    void initialize() override
    {
        if (!normal_cones_.is_valid())
            mesh_.add_property(normal_cones_);

        Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
        Mesh::FaceVertexCCWIter fv_it;

        for (; f_it != f_end; ++f_it)
        {
            fv_it = mesh_.fv_ccwbegin(*f_it);
            const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
            std::vector<NormalCone> normalCones;
            for (uint frame : frame_seq())
            {
                Vec3 p0(positions0.row(frame).transpose());
                Vec3 p1(positions1.row(frame).transpose());
                Vec3 p2(positions2.row(frame).transpose());
                OMVec3 pt0(p0[0], p0[1], p0[2]);
                OMVec3 pt1(p1[0], p1[1], p1[2]);
                OMVec3 pt2(p2[0], p2[1], p2[2]);
                OMVec3 n = face_normal(p0, p1, p2);
                normalCones.emplace_back(
                    NormalCone(n, normal_deviation_ * dist2epicenter_f((pt0 + pt1 + pt2) / 3.0, frame)));
            }
            mesh_.property(normal_cones_, *f_it) = normalCones;
        }
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
        float max_angle(0.0f);
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

            std::vector<NormalCone> nc = mesh_.property(normal_cones_, fh);
            Mesh::FaceVertexCCWIter fv_it = mesh_.fv_ccwbegin(fh);
            // simulate position change
            const MatX3& positions0
                = (*fv_it == _ci.v0) ? mesh_.data(_ci.v1).node->positions : mesh_.data(*fv_it).node->positions;
            fv_it++;
            const MatX3& positions1
                = (*fv_it == _ci.v0) ? mesh_.data(_ci.v1).node->positions : mesh_.data(*fv_it).node->positions;
            fv_it++;
            const MatX3& positions2
                = (*fv_it == _ci.v0) ? mesh_.data(_ci.v1).node->positions : mesh_.data(*fv_it).node->positions;
            fv_it++;
            std::vector<uint> frames(frame_seq());
            for (size_t i = 0; i < frames.size(); i++)
            {
                uint frame = frames[i];
                OMVec3 n = face_normal(positions0.row(frame).transpose(),
                                       positions1.row(frame).transpose(),
                                       positions2.row(frame).transpose());
                nc[i].merge(NormalCone(n, nc[i].max_angle()));
                if (fh == fhl)
                    nc[i].merge(mesh_.property(normal_cones_, _ci.fl)[i]);
                if (fh == fhr)
                    nc[i].merge(mesh_.property(normal_cones_, _ci.fr)[i]);

                if (nc[i].angle() > max_angle)
                {
                    max_angle = nc[i].angle();
                    if (max_angle > 0.5 * nc[i].max_angle())
                        return float(Base::ILLEGAL_COLLAPSE);
                }
            }
        }

        return max_angle;
    }

    /// set the percentage of normal deviation
    void set_error_tolerance_factor(double _factor) override
    {
        if (_factor >= 0.0 && _factor <= 1.0)
        {
            // the smaller the factor, the smaller normal_deviation_ gets
            // thus creating a stricter constraint
            // division by error_tolerance_factor_ is for normalization
            float normal_deviation_value = normal_deviation_ * 180.0f / M_PI * _factor / this->error_tolerance_factor_;

            set_normal_deviation(normal_deviation_value);
            this->error_tolerance_factor_ = _factor;
        }
    }

    void postprocess_collapse(const CollapseInfo& _ci) override
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

            std::vector<NormalCone>& ncs = mesh_.property(normal_cones_, fh);
            Mesh::FaceVertexCCWIter fv_it = mesh_.fv_ccwbegin(fh);
            const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
            int i = 0;
            for (uint frame : frame_seq())
            {
                OMVec3 n = face_normal(positions0.row(frame).transpose(),
                                       positions1.row(frame).transpose(),
                                       positions2.row(frame).transpose());
                ncs[i].merge(NormalCone(n, ncs[i].max_angle()));
                if (fh == fhl)
                    ncs[i].merge(mesh_.property(normal_cones_, _ci.fl)[i]);
                if (fh == fhr)
                    ncs[i].merge(mesh_.property(normal_cones_, _ci.fr)[i]);
            }
        }
    }

  protected:
    static OMVec3 face_normal(Vec3 p0, Vec3 p1, Vec3 p2)
    {
        OMVec3 pt0(p0[0], p0[1], p0[2]);
        OMVec3 pt1(p1[0], p1[1], p1[2]);
        OMVec3 pt2(p2[0], p2[1], p2[2]);
        OMVec3 n = cross(pt2 - pt1, pt0 - pt1);
        float length = norm(n);
        return (length > 0.0) ? n * (1.0f / length) : Normal(0, 0, 0);
    }

    float factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) override
    {
        // TODO implement a proper function here
        float factor = 0.3f + 0.7f * ((pt - epicenter).norm() / mean_dist);
        return factor * factor;
    }

  private:
    float normal_deviation_;
    OpenMesh::FPropHandleT<std::vector<NormalCone>> normal_cones_;
};

} // namespace c2m

#endif
