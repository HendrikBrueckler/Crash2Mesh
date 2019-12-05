#ifndef C2M_MESH_DECIMATER_HPP
#define C2M_MESH_DECIMATER_HPP

#include <crash2mesh/core/collectors.hpp>
#include <crash2mesh/core/mesh.hpp>
#include <crash2mesh/core/structure_elements.hpp>

namespace c2m
{
/**
 * @brief Collector for functions that decimate part-meshes in different ways.
 *
 */
class MeshDecimater
{
  public:
    /**
     * @brief Just for demo purpose, ignores the timeseries and only considers the first time sample.
     *      Uses Quadric Error metric and avoids big normal deviation.
     *
     * @param parts the parts for which meshes should be decimated
     * @return true if successful for all parts
     * @return false else
     */
    static bool decimateSimple(std::vector<Part::Ptr> parts);
};
} // namespace c2m

#endif
