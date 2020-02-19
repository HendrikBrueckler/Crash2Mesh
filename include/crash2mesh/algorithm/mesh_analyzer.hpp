#ifndef C2M_MESH_ANALYZER_HPP
#define C2M_MESH_ANALYZER_HPP

#include <crash2mesh/core/mesh.hpp>

namespace c2m
{

/**
 * @brief Holds info about a Mesh
 *
 */
struct MeshInfo
{
    /**
     * @brief Initialize MeshInfo with Zero values
     *
     */
    MeshInfo()
        : numNodes(0), numConnectedElements(0), numVertices(0), numFaces(0), numBoundaryVertices(0),
          numLockedVertices(0), numComplexVertices(0), numMultiPartVertices(0), bboxMin(), bboxMax(),
          meanPlasticStrain(), minPlasticStrain(), maxPlasticStrain()
    {
    }

    int numNodes;             //!< Number of finite element nodes referenced by vertices of the mesh
    int numConnectedElements; //!< Number of connected finite elements referenced by faces of the mesh

    int numVertices; //!< Number of mesh vertices
    int numFaces;    //!< Number of mesh faces

    int numDuplicateFaces; //!< Number of mesh faces which have an exact duplicate

    int numBoundaryVertices;  //!< Number of boundary vertices
    int numLockedVertices;     //!< Number of vertices locked for decimation
    int numComplexVertices; //!< Number of complex vertices
    int numMultiPartVertices; //!< Number of multipart vertices

    Vec3 bboxSize; //!< width, height, depth of initial bbox

    MatX3 bboxMin; //!< One vec3 per time sample (minX/minY corner)
    MatX3 bboxMax; //!< One vec3 per time sample (maxX/maxY corner)

    VecX meanPlasticStrain; //!< One float per time sample (plastic strain mean)
    VecX minPlasticStrain;  //!< One float per time sample (plastic strain min)
    VecX maxPlasticStrain;  //!< One float per time sample (plastic strain max)

    /**
     * @brief Prettyprint the contained info in string format
     *
     * @return std::string the contained info
     */
    std::string print() const;
};

/**
 * @brief Collector for several mesh meta functions
 *
 */
class MeshAnalyzer
{
  public:
    /**
     * @brief Get a MeshInfo instance describing the mesh
     *
     * @param mesh mesh to query info for
     * @return MeshInfo info about the mesh
     */
    static MeshInfo getInfo(const CMesh& mesh);

    /**
     * @brief Opens a window rendering the passed mesh's triangles
     *
     * @param mesh mesh to render
     */
    static void render(const CMesh& mesh, const MatX3* epicenters = nullptr, const VecX* meanDists = nullptr);

    /**
     * @brief Get the epicenter positions for the given mesh for each frame
     *        as well as the mean distances of all vertices from the epicenter.
     *
     * @param mesh deforming mesh to extract epicenters from
     * @param epicenters the epicenter coordinates for each frame
     * @param meanDists the mean distances from the epicenter for each frame
     */
    static void getEpicenter(CMesh& mesh, MatX3& epicenters, VecX& meanDists);

    /**
     * @brief Get all duplicate vertices of \p vh (duplicated to fix non-manifoldness)
     *
     * @param mesh mesh which contains \p vh
     * @param vh the handle of the vertex to query
     * @return std::vector<VHandle> all duplicates of \p vh, this will include \p vh !
     */
    static std::vector<VHandle> dupes(const CMesh& mesh, const VHandle& vh);

    /**
     * @brief Get all duplicate vertices of \p heh (duplicated to fix non-manifoldness)
     *
     * @param mesh mesh which contains \p heh
     * @param heh the handle of the halfedge to query
     * @return std::vector<VHandle> all duplicates of \p heh, this will include \p heh !
     */
    static std::vector<HEHandle> dupes(const CMesh& mesh, const HEHandle& heh);
};

} // namespace c2m

#endif
