#ifndef C2M_TYPES_HPP
#define C2M_TYPES_HPP

#include <OpenVolumeMesh/Geometry/VectorT.hh>

#include <highfive/H5Exception.hpp>
#include <highfive/H5File.hpp>

namespace c2m
{
/**
 * @brief Type to be used for the identifiers of HDF5 entities
 *
 */
using entid_t = long;
/**
 * @brief 3D double vector for use with OpenVolumeMesh
 *
 */
using VolVec3 = OpenVolumeMesh::Geometry::VectorT<double, 3>;
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
}

#endif
