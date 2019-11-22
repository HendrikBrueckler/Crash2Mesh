#ifndef C2M_MESH_BUILDER_HPP
#define C2M_MESH_BUILDER_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/core/mesh.hpp>
#include <vector>

namespace c2m
{

class MeshBuilder
{
  public:
    static bool build(std::vector<Part::Ptr>& parts, std::vector<Mesh>& meshes);
};

} // namespace c2m

#endif
