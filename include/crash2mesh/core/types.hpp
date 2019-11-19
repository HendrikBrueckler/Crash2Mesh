#ifndef C2M_TYPES_HPP
#define C2M_TYPES_HPP

#include <OpenVolumeMesh/Geometry/VectorT.hh>

#include <highfive/H5Exception.hpp>
#include <highfive/H5File.hpp>

namespace c2m
{
/**
 * @brief Types to be used for the identifiers of HDF5 entities
 *
 */
using entid_t = long; // needs to be >= nodeid_t, elemid_t, partid_t
using nodeid_t = long;
using elemid_t = long;
using partid_t = long;
#define ID_NULL 0

/**
 * @brief 3D double vector for use with OpenVolumeMesh
 *
 */
using VVec3d = OpenVolumeMesh::Geometry::Vec3d;
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
} // namespace c2m

#endif
