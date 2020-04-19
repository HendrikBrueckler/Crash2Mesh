#ifndef C2M_MESH_DECIMATER_HPP
#define C2M_MESH_DECIMATER_HPP

#include <crash2mesh/core/collectors.hpp>
#include <crash2mesh/core/mesh.hpp>
#include <crash2mesh/core/structure_elements.hpp>

#if __cplusplus > 201703L
#include <version>
#else
#include <ciso646>
#endif

namespace c2m
{
/**
 * @brief Collector for functions that decimate part-meshes in different ways.
 *
 */
class MeshDecimater
{
  public:
    MeshDecimater()
        : useQuadric(true), useNormalDeviation(true), useBoundaryDeviation(true), useAspectRatio(true),
          quadricExcludeOnly(false), normalExcludeOnly(true),
          combineQuadricNormal(false), quadricAreaWeighting(false), quadricPositionOptimization(false),
          quadricPostProcessOptimize(false),
          boundaryQuadrics(true), featureQuadrics(true), maxQuadricError(2000), maxNormalDeviation(10), maxBoundaryDeviation(10),
          maxAspectRatio(10), framesQuadric(10), framesNormalDeviation(10), framesBoundaryDeviation(3), minVLog(1000),
          minVRender(1000), maxVLog(10000000), maxVRender(500000)
    {
    }

    /**
     * @brief Query user for minimum vertices for logging/rendering a part/scene
     *
     */
    void queryLogParts();

    /**
     * @brief Decimates using preset parameters.
     *        If you know what you're doing use other methods.
     *
     * @param parts the parts for which meshes should be decimated
     * @return true if successful for all parts
     * @return false else
     */
    static bool decimateSimple(std::vector<Part::Ptr>& parts);

    /**
     * @brief Decimates each part separately using this instances configuration.
     *        Stops when no valid incremental decimation operations remain that
     *        wouldn't violate the given error bounds.
     *
     * @param parts parts to decimate
     * @return true if successful
     * @return false else
     */
    bool decimateParts(std::vector<Part::Ptr>& parts) const;

    /**
     * @brief Decimates the whole scene using this instances configuration
     *        Stops when both thresholds are reached or no valid incremental
     *        decimation operations remain that wouldn't violate the configured
     *        error bounds.
     *
     * @param scene scene to decimate
     * @param nFaces number of faces to decimate to
     * @param nVertices number of vertices to decimate to
     * @return true if successful
     * @return false else
     */
    bool decimateScene(Scene::Ptr scene, uint nFaces, uint nVertices = 0) const;

  public:
    MatX3 epicenters;             //< deformation/impact epicenter positions. see MeshAnalyzer::getEpicenter
    VecX meanDistsFromEpicenters; //< mean distances from epicenters. see MeshAnalyzer::getEpicenter

    bool useQuadric;           //< Whether to use quadric module
    bool useNormalDeviation;   //< Whether to use normal deviation module
    bool useBoundaryDeviation; //< Whether to use boundary deviation module
    bool useAspectRatio;       //< Whether to use aspect ratio module

    bool quadricExcludeOnly; //< Whether to use quadrics as binary exclusion modules only
    bool normalExcludeOnly;  //< Whether to use normals as binary exclusion modules only

    bool combineQuadricNormal; //< Whether to combine quadrics and normals to one joint continuous module

    bool quadricAreaWeighting;        //< Whether to weight quadrics with the face area
    bool quadricPositionOptimization; //< Whether to optimize positions wrt quadrics during decimation
    bool quadricPostProcessOptimize;  //< Whether to optimize positions wrt quadrics AFTER decimation
    bool boundaryQuadrics;
    bool featureQuadrics;

    float maxQuadricError;      //< Maximum tolerable error of quadric module
    float maxNormalDeviation;   //< Maximum tolerable normal change of normal module
    float maxBoundaryDeviation; //< Maximum tolerable angular difference of boundary module
    float maxAspectRatio;       //< Maximum tolerable aspect ratio of aspect ratio module

    uint framesQuadric;           //< How many linearly spaced frames to consider for quadric module
    uint framesNormalDeviation;   //< How many linearly spaced frames to consider for normal module
    uint framesBoundaryDeviation; //< How many linearly spaced frames to consider for boundary module

    uint minVLog;    //< Min number of mesh vertices to log meshinfo on console
    uint minVRender; //< Min number of mesh vertices to render mesh in own window
    uint maxVLog;    //< Max number of mesh vertices to log meshinfo on console
    uint maxVRender; //< Max number of mesh vertices to render mesh in own window

  private:
    static std::mutex mutLog;
    static std::mutex mutRender;
    /**
     * @brief Decimates a given mesh down to a certain number of faces/vertices or
     *        or until no valid incremental decimation operations remain, that
     *        wouldn't violate the given error bounds.
     *
     * @param mesh the mesh to decimate
     * @param nFaces the number of faces to achieve
     * @param nVertices the number of vertices to achieve
     * @param puid part user id of part to be decimated
     * @param forceRemove whether force removal of faces connected to vertices that need to be forcibly removed
     */
    void decimate(CMesh& mesh, uint nFaces, uint nVertices = 0, entid_t puid = 0, bool forceRemove = false) const;

    /**
     * @brief Logs the mesh info if number of vertices satisfies the internal bounds.
     *
     * @param mesh mesh to log
     * @param preDecimation whether to log "before decimation" or "after decimation" on console
     * @param force whether to force logging regardless of internal bounds
     * @param pid part id of part to be decimated
     * @param uid user-specified part id of part to be decimated
     * @return true if logged
     * @return false else
     */
    bool log(const CMesh& mesh, bool preDecimation, bool force, partid_t pid = 0, entid_t uid = 0) const;

    /**
     * @brief Renders the mesh if number of vertices satisfies the internal bounds.
     *
     * @param mesh mesh to render
     * @param epicenterVis whether to visualize epicenter vars
     * @param force whether to force rendering regardless of internal bounds
     * @return true if rendered
     * @return false else
     */
    bool render(const CMesh& mesh, bool epicenterVis, bool force) const;
};
} // namespace c2m

#endif
