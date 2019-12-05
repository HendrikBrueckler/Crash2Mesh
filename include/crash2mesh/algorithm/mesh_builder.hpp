#ifndef C2M_MESH_BUILDER_HPP
#define C2M_MESH_BUILDER_HPP

#include <crash2mesh/core/collectors.hpp>
#include <vector>

namespace c2m
{

/**
 * @brief Collector for functions to build a OpenMesh type mesh from parts containing finite element primitives
 *
 */
class MeshBuilder
{
  public:
    /**
     * @brief Build a mesh for each part, given the 2D and surface elements contained in the respective part
     *
     * @param parts parts to build meshes for
     * @param deleteMeshedElements whether to remove the element references that are represented/referenced by the mesh
     *      from the parts' lists
     * @return true if successful
     * @return false else
     */
    static bool build(std::vector<Part::Ptr>& parts, bool deleteMeshedElements = true);
};

} // namespace c2m

#endif
