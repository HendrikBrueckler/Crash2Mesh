#ifndef C2M_TYPES_HPP
#define C2M_TYPES_HPP

#include <OpenMesh/Core/Geometry/VectorT.hh>

#include <Eigen/Dense>

#include <highfive/H5Exception.hpp>
#include <highfive/H5File.hpp>

namespace c2m
{
using uint = unsigned int;
/**
 * @brief Types to be used for the identifiers of HDF5 entities
 *
 */
using entid_t = int; // needs to be >= nodeid_t, elemid_t, partid_t
using nodeid_t = int;
using elemid_t = int;
using partid_t = int;
#define ID_NULL 0

/**
 * @brief Exception to handle when reading from HDF5
 *
 */
using H5Exception = HighFive::Exception;
/**
 * @brief HDF5 file handle
 *
 */
using H5File = HighFive::File;

using OMVec3 = OpenMesh::Vec3f;

/**
 * @brief 3D float vector for use with OpenVolumeMesh
 *
 */
using Vec3 = Eigen::Vector3f;
using VecX = Eigen::VectorXf;

template <typename Scalar, int Rows, int Cols>
using Mat = Eigen::Matrix<Scalar, Rows, Cols>;
using Eigen::Dynamic;

using Mat3X = Eigen::Matrix3Xf;
using MatX3 = Eigen::MatrixX3f;
using MatX = Eigen::MatrixXf;

} // namespace c2m

#endif
