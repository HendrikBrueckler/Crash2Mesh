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
     *        The parts' Elements1D will be preserved and wont enter the resulting mesh. The Elements2D and
     * SurfaceElements will be meshed and may be deleted.
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
     *        The parts' Elements1D will be preserved and wont enter the resulting mesh. The Elements2D and
     * SurfaceElements will be meshed and may be deleted.
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

  private:
    /**
     * @brief Simple edge representation for internal detection of connected faces,
     *        non-manifold connections, detection of flipped triangles etc.
     */
    struct C2MEdge
    {
        C2MEdge(Node::Ptr _from, Node::Ptr _to) : from(_from), to(_to)
        {
        }

        Node::Ptr from;
        Node::Ptr to;

        // Direct comparison
        bool operator==(const C2MEdge& other)
        {
            return (from->ID == other.from->ID && to->ID == other.to->ID)
                   || (from->ID == other.to->ID && to->ID == other.from->ID);
        }

        // Compare for use in std::set / std::map, lexicographical comparison of sorted vertex indices
        struct Less
        {
            bool operator()(const C2MEdge& lhs, const C2MEdge& rhs) const
            {
                nodeid_t lhsfromID = lhs.from->ID;
                nodeid_t lhstoID = lhs.to->ID;
                nodeid_t rhsfromID = rhs.from->ID;
                nodeid_t rhstoID = rhs.to->ID;
                if (lhstoID < lhsfromID)
                {
                    std::swap(lhsfromID, lhstoID);
                }
                if (rhstoID < rhsfromID)
                {
                    std::swap(rhstoID, rhsfromID);
                }
                return (lhsfromID < rhsfromID) || ((lhsfromID == rhsfromID) && (lhstoID < rhstoID));
            }
        };
    };

    /**
     * @brief Simple triangle representation for internal detection of connected faces,
     *        non-manifold connections, detection of flipped triangles etc.
     */
    struct Triangle
    {
        Triangle() : mark(false), edges(), elem()
        {
        }
        bool mark;
        std::vector<C2MEdge> edges;
        Element2D::Ptr elem;
    };

    /**
     * @brief Breadth-first floodfill-type algorithm, that flips adjacent triangles, if their normals
     *        are inversed in comparison to the source triangle.
     *        No recursion is used.
     *        No expansion will be done over non-manifold edges (> 2 connected triangles).
     *
     * @param fi index of triangle to start floodfill
     * @param allTriangles vector of triangles which \p fi indexes
     * @param edgeTriangleIndices mapping from edge to all triangles connected to that edge
     * @param sortedTriangles triangles will be placed in here in the order of expansion
     */
    static void floodFlip(size_t fi,
                          std::vector<Triangle>& allTriangles,
                          std::map<C2MEdge, std::vector<size_t>, C2MEdge::Less>& edgeTriangleIndices,
                          std::vector<size_t>& sortedTriangles);

    /**
     * @brief Triangulates the face stored in \p elem using a triangle fan (ccw);
     *
     * @param elem element to triangulate
     * @param allTriangles new triangles will be appended to this
     * @param edgeTriangleIndices edge to triangle references will be added to this
     * @return size_t number of triangles added
     */
    static size_t triangulate(const Element2D::Ptr& elem,
                              std::vector<Triangle>& allTriangles,
                              std::map<C2MEdge, std::vector<size_t>, C2MEdge::Less>& edgeTriangleIndices);

    /**
     * @brief Triangulates all faces contained in Elements2D/SurfaceElements of \p partptr
     *        using a triangle fan.
     *
     * @param partptr part to triangulate
     * @param allTriangles new triangles will be appended to this
     * @param edgeTriangleIndices edge to triangle references will be added to this
     * @param deleteMeshedElements whether to remove the pointers of triangulated elements from the part
     * @return size_t
     */
    static size_t triangulateAll(Part::Ptr& partptr,
                                 std::vector<Triangle>& allTriangles,
                                 std::map<C2MEdge, std::vector<size_t>, C2MEdge::Less>& edgeTriangleIndices,
                                 bool deleteMeshedElements);

    /**
     * @brief Retrieve a duplicate vertex of \p v which has the same 3D position.
     *        Necessary for fixing of complex non-manifold geometry.
     *
     * @param v vertex to duplicate
     * @param orig2dupe map from original vertex to its duplicate (a dupe may be mapped a 2nd order dupe)
     * @param mesh mesh in which the duplicate vertex should be inserted
     * @return VHandle orig2dupe[v] if exists, new duplicate vertex else
     */
    static VHandle getDuplicate(VHandle& v, std::map<VHandle, VHandle>& orig2dupe, CMesh& mesh);

    /**
     * @brief Writes a mesh to \p mesh incorporating all primitive triangles of \p allTriangles .
     *        Triangles will be added in index order of \p triangleOrdering .
     *        This is useful because non-manifold elements are detected on the fly and duplicates
     *        are generated for the first added triangle that would create a non-manifold connectivity.
     *
     * @param mesh The generated mesh will be stored here
     * @param allTriangles primitive triangles to mesh
     * @param triangleOrdering ordering of primitive triangles (by index)
     * @param validTris number of meshed triangles which were added without a non-manifoldness-alert
     * @param invalidTris number of meshed triangles which had to be fixed because of non-manifoldness
     */
    static void assembleMeshFromTriangles(CMesh& mesh,
                                          const std::vector<Triangle>& allTriangles,
                                          const std::vector<size_t>& triangleOrdering,
                                          int& validTris,
                                          int& invalidTris);

#if 0
  /**
   * @brief Retrieve all connected non-manifold edge paths. Primitive expansion. Tree-like connected
   *        non-manifold edge paths will be split into arbitrary branches (depending on search startedge).
   *
   * @param edgeTriangleIndices mapping from edge to connected triangles
   * @return std::vector<std::list<C2MEdge>> connected non-manifold edge paths
   */
  std::vector<std::list<C2MEdge>> nonManifoldPaths(std::map<C2MEdge, std::vector<size_t>, C2MEdge::Less>& edgeTriangleIndices);

  /**
   * @brief Attempt to sort triangles adjacent to non-manifold paths in a way that keeps plane surfaces connected,
   *        but splits faces perpendicular to such planes.
   *        Compares face normals to determin base plane and perpendicular faces.
   *        // TODO does not work well at all, as only triangles with complex edges are considered, but no triangles
   *        // with only complex vertices
   *
   * @param nMfPaths connected non-manifold edge paths
   * @param allTriangles all triangles. will contain sorted triangles after function call
   * @param edgeTriangleIndices mapping from edge to connected triangles
   */
  void sortFromNonManifoldPaths(std::vector<std::list<C2MEdge>>& nMfPaths,
                                             std::vector<Triangle>& allTriangles,
                                             std::map<C2MEdge, std::vector<size_t>, C2MEdge::Less>& edgeTriangleIndices);
#endif
};

} // namespace c2m

#endif
