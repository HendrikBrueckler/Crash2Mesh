#ifndef C2M_FRAMEWISE_STRAIN_HPP
#define C2M_FRAMEWISE_STRAIN_HPP

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

template <class MeshT> class ModFramewiseStrainT : public OpenMesh::Decimater::ModBaseT<MeshT>
{
  public:
    DECIMATING_MODULE(ModFramewiseStrainT, MeshT, FramewiseStrainDeviation);

    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Normal Normal;
    typedef typename Mesh::VertexHandle VertexHandle;
    typedef typename Mesh::FaceHandle FaceHandle;
    typedef typename Mesh::EdgeHandle EdgeHandle;

  public:
    /// Constructor
    ModFramewiseStrainT(MeshT& _mesh, float _max_dev = 45.0)
        : Base(_mesh, true), mesh_(Base::mesh()), frame_skip_(0)
    {
        set_max_dev(_max_dev);
    }

    /// Destructor
    ~ModFramewiseStrainT()
    {
    }

    /// Get normal deviation ( 0 .. 360 )
    Scalar max_dev() const
    {
        return max_dev_ / M_PI * 180.0;
    }

    /// Set normal deviation ( 0 .. 360 )
    void set_max_dev(Scalar _s)
    {
        max_dev_ = _s / static_cast<Scalar>(180.0) * static_cast<Scalar>(M_PI);
    }

    void initialize() override
    {
        if (Base::mesh().n_vertices() != 0)
            num_frames_ = Base::mesh().data(*Base::mesh().vertices_begin()).node->positions.rows();
        else
            num_frames_ = 0;
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
        // TODO

        return float(Base::LEGAL_COLLAPSE);
    }

    /// set the percentage of normal deviation
    void set_error_tolerance_factor(double _factor) override
    {
        if (_factor >= 0.0 && _factor <= 1.0)
        {
            // the smaller the factor, the smaller max_dev_ gets
            // thus creating a stricter constraint
            // division by error_tolerance_factor_ is for normalization
            Scalar max_dev_value
                = max_dev_ * static_cast<Scalar>(180.0 / M_PI * _factor / this->error_tolerance_factor_);

            set_max_dev(max_dev_value);
            this->error_tolerance_factor_ = _factor;
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
    Scalar max_dev_;
};

} // namespace c2m

#endif // C2M_FRAMEWISE_STRAIN_HPP
