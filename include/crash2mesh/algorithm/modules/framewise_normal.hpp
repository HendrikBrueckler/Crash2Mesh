#ifndef C2M_FRAMEWISE_NORMAL_HPP
#define C2M_FRAMEWISE_NORMAL_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Geometry/NormalConeT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>

#include <float.h>
#include <vector>

namespace c2m
{

template <class MeshT> class ModFramewiseNormalT : public OpenMesh::Decimater::ModBaseT<MeshT>
{
  public:
    DECIMATING_MODULE(ModFramewiseNormalT, MeshT, FramewiseNormalDeviation);

    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Normal Normal;
    typedef typename Mesh::VertexHandle VertexHandle;
    typedef typename Mesh::FaceHandle FaceHandle;
    typedef typename Mesh::EdgeHandle EdgeHandle;
    typedef OpenMesh::NormalConeT<Scalar> NormalCone;

  public:
    /// Constructor
    ModFramewiseNormalT(MeshT& _mesh, float _max_dev = 180.0) : Base(_mesh, true), mesh_(Base::mesh()), frame_skip_(0)
    {
        set_normal_deviation(_max_dev);
        mesh_.add_property(normal_cones_);
    }

    /// Destructor
    ~ModFramewiseNormalT()
    {
        mesh_.remove_property(normal_cones_);
    }

    /// Get normal deviation ( 0 .. 360 )
    Scalar normal_deviation() const
    {
        return normal_deviation_ / M_PI * 180.0;
    }

    /// Set normal deviation ( 0 .. 360 )
    void set_normal_deviation(Scalar _s)
    {
        normal_deviation_ = _s / static_cast<Scalar>(180.0) * static_cast<Scalar>(M_PI);
    }

    /// Allocate and init normal cones
    void initialize() override
    {
        if (Base::mesh().n_vertices() != 0)
            num_frames_ = Base::mesh().data(*Base::mesh().vertices_begin()).node->positions.rows();
        else
            num_frames_ = 0;

        if (!normal_cones_.is_valid())
            mesh_.add_property(normal_cones_);

        typename Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();
        typename Mesh::FaceVertexCCWIter fv_it;

        for (; f_it != f_end; ++f_it)
        {
            fv_it = mesh_.fv_ccwbegin(*f_it);
            const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
            std::vector<NormalCone> normalCones;
            for (uint frame = 0; frame < num_frames_; frame += 1 + frame_skip_)
            {
                Point pt0(positions0.coeff(frame, 0), positions0.coeff(frame, 1), positions0.coeff(frame, 2));
                Point pt1(positions1.coeff(frame, 0), positions1.coeff(frame, 1), positions1.coeff(frame, 2));
                Point pt2(positions2.coeff(frame, 0), positions2.coeff(frame, 1), positions2.coeff(frame, 2));
                typename Mesh::Normal n;
                Point p1p0 = pt0;
                p1p0 -= pt1;
                Point p1p2 = pt2;
                p1p2 -= pt1;
                n = OpenMesh::vector_cast<typename Mesh::Normal>(cross(p1p2, p1p0));
                typename OpenMesh::vector_traits<typename Mesh::Normal>::value_type length = norm(n);
                n = (length != 0.0) ? n * (1.0 / length) : Normal(0, 0, 0);
                normalCones.emplace_back(NormalCone(n));
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
        typename Mesh::Scalar max_angle(0.0);
        typename Mesh::ConstVertexFaceIter vf_it(mesh_, _ci.v0);
        typename Mesh::FaceHandle fh, fhl, fhr;

        if (_ci.v0vl.is_valid())
            fhl = mesh_.face_handle(_ci.v0vl);
        if (_ci.vrv0.is_valid())
            fhr = mesh_.face_handle(_ci.vrv0);

        bool legal = true;
        for (; vf_it.is_valid() && legal; ++vf_it)
        {
            fh = *vf_it;
            if (fh == _ci.fl || fh == _ci.fr)
                continue;

            std::vector<NormalCone> nc = mesh_.property(normal_cones_, fh);
            typename Mesh::FaceVertexCCWIter fv_it = mesh_.fv_ccwbegin(fh);
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
            for (uint frame = 0, i = 0; frame < num_frames_ && legal; frame += 1 + frame_skip_, i++)
            {
                Point pt0(positions0.coeff(frame, 0), positions0.coeff(frame, 1), positions0.coeff(frame, 2));
                Point pt1(positions1.coeff(frame, 0), positions1.coeff(frame, 1), positions1.coeff(frame, 2));
                Point pt2(positions2.coeff(frame, 0), positions2.coeff(frame, 1), positions2.coeff(frame, 2));
                typename Mesh::Normal n;
                Point p1p0 = pt0;
                p1p0 -= pt1;
                Point p1p2 = pt2;
                p1p2 -= pt1;
                n = OpenMesh::vector_cast<typename Mesh::Normal>(cross(p1p2, p1p0));
                typename OpenMesh::vector_traits<typename Mesh::Normal>::value_type length = norm(n);
                n = (length != 0.0) ? n * (1.0 / length) : Normal(0, 0, 0);
                nc[i].merge(NormalCone(n));
                if (fh == fhl)
                    nc[i].merge(mesh_.property(normal_cones_, _ci.fl)[i]);
                if (fh == fhr)
                    nc[i].merge(mesh_.property(normal_cones_, _ci.fr)[i]);

                if (nc[i].angle() > max_angle)
                {
                    max_angle = nc[i].angle();
                    if (max_angle > 0.5 * normal_deviation_)
                        legal = false;
                }
            }
        }

        return (legal ? max_angle : float(Base::ILLEGAL_COLLAPSE));
    }

    /// set the percentage of normal deviation
    void set_error_tolerance_factor(double _factor) override
    {
        if (_factor >= 0.0 && _factor <= 1.0)
        {
            // the smaller the factor, the smaller normal_deviation_ gets
            // thus creating a stricter constraint
            // division by error_tolerance_factor_ is for normalization
            Scalar normal_deviation_value
                = normal_deviation_ * static_cast<Scalar>(180.0 / M_PI * _factor / this->error_tolerance_factor_);

            set_normal_deviation(normal_deviation_value);
            this->error_tolerance_factor_ = _factor;
        }
    }

    void postprocess_collapse(const CollapseInfo& _ci) override
    {
        typename Mesh::ConstVertexFaceIter vf_it(mesh_, _ci.v0);
        typename Mesh::FaceHandle fh, fhl, fhr;

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
            typename Mesh::FaceVertexCCWIter fv_it = mesh_.fv_ccwbegin(fh);
            const MatX3& positions0 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions1 = mesh_.data(*(fv_it++)).node->positions;
            const MatX3& positions2 = mesh_.data(*fv_it).node->positions;
            for (uint frame = 0, i = 0; frame < num_frames_; frame += 1 + frame_skip_, i++)
            {
                Point pt0(positions0.coeff(frame, 0), positions0.coeff(frame, 1), positions0.coeff(frame, 2));
                Point pt1(positions1.coeff(frame, 0), positions1.coeff(frame, 1), positions1.coeff(frame, 2));
                Point pt2(positions2.coeff(frame, 0), positions2.coeff(frame, 1), positions2.coeff(frame, 2));
                typename Mesh::Normal n;
                Point p1p0 = pt0;
                p1p0 -= pt1;
                Point p1p2 = pt2;
                p1p2 -= pt1;
                n = OpenMesh::vector_cast<typename Mesh::Normal>(cross(p1p2, p1p0));
                typename OpenMesh::vector_traits<typename Mesh::Normal>::value_type length = norm(n);
                n = (length != 0.0) ? n * (1.0 / length) : Normal(0, 0, 0);
                ncs[i].merge(NormalCone(n));
                if (fh == fhl)
                    ncs[i].merge(mesh_.property(normal_cones_, _ci.fl)[i]);
                if (fh == fhr)
                    ncs[i].merge(mesh_.property(normal_cones_, _ci.fr)[i]);
            }
        }
    }

    void set_frame_skip(uint fs)
    {
        frame_skip_ = fs;
    }

    uint frame_skip() const
    {
        return frame_skip_;
    }

  private:
    Mesh& mesh_;
    uint frame_skip_;
    uint num_frames_;
    Scalar normal_deviation_;
    OpenMesh::FPropHandleT<std::vector<NormalCone>> normal_cones_;
};

} // namespace c2m

#endif // C2M_FRAMEWISE_NORMAL_HPP defined
