#ifndef C2M_ROBUST_DECIMATER_HPP
#define C2M_ROBUST_DECIMATER_HPP

#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <crash2mesh/core/mesh.hpp>

namespace c2m
{
class RobustDecimater : virtual public OpenMesh::Decimater::BaseDecimaterT<CMesh>
{
  public:
    using Base = BaseDecimaterT<CMesh>;
    using CollapseInfo = OpenMesh::Decimater::CollapseInfoT<CMesh>;
    using Module = OpenMesh::Decimater::ModBaseT<CMesh>;
    using ModuleList = std::vector<Module*>;
    using ModuleListIterator = ModuleList::iterator;
    using HeapInterface = OpenMesh::Decimater::DecimaterT<CMesh>::HeapInterface;
    using DeciHeap = OpenMesh::Utils::HeapT<VHandle, HeapInterface>;

    /// Constructor
    explicit RobustDecimater(CMesh& _mesh);

    /// Destructor
    ~RobustDecimater();

    /**
     * @brief Attempts to decimate the mesh until a desired vertex or face
     *        complexity is achieved.
     * @param _n_vertices Target vertex complexity.
     * @param _n_faces Target face complexity.
     * @return Number of collapses that were actually performed.
     * @note Decimation stops as soon as either one of the two complexity bounds
     *       is satisfied.
     * @note This operation only marks the removed mesh elements for deletion. In
     *       order to actually remove the decimated elements from the mesh, a
     *       subsequent call to ArrayKernel::garbage_collection() is required.
     */
    size_t decimate_to_faces(size_t _n_vertices = 0, size_t _n_faces = 0);

  protected:
    /**
     * @brief Gather duplicates of single vertices and halfedge
     *        neighborhoods.
     *
     * @param _ci
     * @param v0Dupes
     * @param v1Dupes
     * @param ciDupes
     */
    void getDupes(const CollapseInfo& _ci,
                  std::vector<VHandle>& v0Dupes,
                  std::vector<VHandle>& v1Dupes,
                  std::vector<CollapseInfo>& ciDupes) const;

    /// Insert vertex in heap
    void heap_vertex(VHandle _vh);

    // reference to mesh
    Mesh& mesh_;

    // heap
    std::unique_ptr<DeciHeap> heap_;

    // vertex properties
    OpenMesh::VPropHandleT<HEHandle> collapse_target_;
    OpenMesh::VPropHandleT<float> priority_;
    OpenMesh::VPropHandleT<int> heap_position_;
};
} // namespace c2m

#endif
