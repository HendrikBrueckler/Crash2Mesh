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
     * @brief Build a mesh for each part, given the 2D and surface elements contained in the respective part.
     *        fixes non-manifoldness by duplicating vertices.
     *
     *        The parts' Elements1D will be preserved and wont enter the resulting mesh. The Elements2D and SurfaceElements
     *        will be meshed and may be deleted.
     *
     * @param parts parts to build meshes for
     * @param deleteMeshedElements whether to remove the element references that were meshed from the parts' lists
     * @return true if successful
     * @return false else
     */
    static bool build(std::vector<Part::Ptr>& parts, bool deleteMeshedElements = true);

    /**
     * @brief Builds a single combined mesh from all finite elements of the parts,
     *        fixes non-manifoldness by duplicating vertices.
     *
     *        The parts' Elements1D will be preserved and wont enter the resulting mesh. The Elements2D and SurfaceElements
     *        will be meshed and may be deleted.
     *
     * @param parts parts to transform into meshes
     * @param deleteMeshedElements whether to remove the element references that were meshed from the parts' lists
     * @return CMesh the generated mesh
     */
    static CMesh buildSingle(std::vector<Part::Ptr>& parts, bool deleteMeshedElements = true);

    /**
     * @brief Builds a single combined mesh from all part meshes, fixes non-manifoldness by duplicating vertices.
     *        fixes non-manifoldness by duplicating vertices.
     *
     * @param parts parts whose meshes should be merged into a single mesh
     * @param deleteMeshedElements whether to remove the separate part meshes after merging into a single mesh
     * @return Scene::Ptr a scene with its own combined mesh and references to its contained parts
     */
    static Scene::Ptr merge(std::vector<Part::Ptr>& parts, bool deleteMeshedElements = true);

};

} // namespace c2m

#endif
