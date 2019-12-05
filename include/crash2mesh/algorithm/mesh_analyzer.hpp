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
          numFixedVertices(0), numMultiPartVertices(0), bboxMin(), bboxMax(), meanDisplacement(), minDisplacement(),
          maxDisplacement(), meanPlasticStrain(), minPlasticStrain(), maxPlasticStrain(), meanEPlasticStrain(),
          minEPlasticStrain(), maxEPlasticStrain()
    {
    }

    int numNodes;             //!< Number of finite element nodes referenced by vertices of the mesh
    int numConnectedElements; //!< Number of connected finite elements referenced by faces of the mesh

    int numVertices; //!< Number of mesh vertices
    int numFaces;    //!< Number of mesh faces

    int numBoundaryVertices;  //!< Number of boundary vertices
    int numFixedVertices;     //!< Number of fixed vertices (either multipart vertices or manifold/complex vertices)
    int numMultiPartVertices; //!< Number of multipart vertices

    MatX3 bboxMin; //!< One vec3 per time sample (minX/minY corner)
    MatX3 bboxMax; //!< One vec3 per time sample (maxX/maxY corner)

    MatX3 meanDisplacement; //!< One vec3 per time sample (displacement mean)
    MatX3 minDisplacement;  //!< One vec3 per time sample (displacement min)
    MatX3 maxDisplacement;  //!< One vec3 per time sample (displacement max)

    VecX meanPlasticStrain; //!< One float per time sample (plastic strain mean)
    VecX minPlasticStrain;  //!< One float per time sample (plastic strain min)
    VecX maxPlasticStrain;  //!< One float per time sample (plastic strain max)

    VecX meanEPlasticStrain; //!< One float per time sample (equivalent plastic strain mean
    VecX minEPlasticStrain;  //!< One float per time sample (equivalent plastic strain min)
    VecX maxEPlasticStrain;  //!< One float per time sample (equivalent plastic strain max)

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
    static MeshInfo getInfo(const Mesh& mesh);

    /**
     * @brief Opens a window rendering the passed mesh's triangles
     *
     * @param mesh mesh to render
     */
    static void render(const Mesh& mesh);
};

} // namespace c2m

#endif
