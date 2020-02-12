#include <crash2mesh/algorithm/mesh_decimater.hpp>

#include <crash2mesh/algorithm/decimater/robust_decimater.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/modules/framewise_boundary.hpp>
#include <crash2mesh/algorithm/modules/framewise_normal.hpp>
#include <crash2mesh/algorithm/modules/framewise_quadric.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>
#include <OpenMesh/Tools/Decimater/ModHausdorffT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalFlippingT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Tools/Decimater/ModRoundnessT.hh>

#include <set>

namespace c2m
{
using Decimater = OpenMesh::Decimater::DecimaterT<CMesh>;
using ModQuadricH = OpenMesh::Decimater::ModQuadricT<CMesh>::Handle;
using ModAspectRatioH = OpenMesh::Decimater::ModAspectRatioT<CMesh>::Handle;
using ModHausdorffH = OpenMesh::Decimater::ModHausdorffT<CMesh>::Handle;
using ModNormalFlippingH = OpenMesh::Decimater::ModNormalFlippingT<CMesh>::Handle;
using ModNormalDeviationH = OpenMesh::Decimater::ModNormalDeviationT<CMesh>::Handle;
using ModRoundnessH = OpenMesh::Decimater::ModRoundnessT<CMesh>::Handle;
using ModFramewiseQuadricH = ModFramewiseQuadricT<CMesh>::Handle;
using ModFramewiseNormalH = ModFramewiseNormalT<CMesh>::Handle;
using ModFramewiseBoundaryH = ModFramewiseBoundaryT<CMesh>::Handle;

using std::set;

static void queryLogParts(uint& minVLog, uint& minVRender)
{
    set<uint> partNums;
    std::cout << "Wanna visualize parts ? (y/N)" << std::endl;
    char query = 'n';
    std::cin >> query;
    if (query == 'y' || query == 'Y')
    {
        std::cout << "Enter minimum amount of vertices for logging " << std::endl;
        std::cin >> minVLog;
        std::cout << "Enter minimum amount of vertices for rendering " << std::endl;
        std::cin >> minVRender;
    }
    else
    {
        minVLog = minVRender = std::numeric_limits<uint>::max();
    }
}

static bool
renderAndLog(const CMesh& mesh, bool preDecimation, uint minVLog, uint maxVLog, uint minVRender, uint maxVRender)
{
    if (mesh.n_vertices() < minVLog || mesh.n_vertices() > maxVLog)
        return false;

    Logger& lout = Logger::lout(Logger::DEBUG);
    if (preDecimation)
        lout << "Meshinfo before decimation:" << std::endl;
    else
        lout << "Meshinfo after decimation:" << std::endl;

    lout << MeshAnalyzer::getInfo(mesh).print();

    if (mesh.n_vertices() > minVRender && mesh.n_vertices() < maxVRender)
    {
        MeshAnalyzer::render(mesh);
    }
    return true;
}

static void decimate(CMesh& mesh,
                     const MatX3& epicenters,
                     const VecX& meanDistsFromEpicenters,
                     float maxQuadricError,
                     float maxNormalAngle,
                     float maxBoundaryAngle,
                     uint frameSkipQuadric,
                     uint frameSkipNormal,
                     uint frameSkipBoundary,
                     uint nFaces = 0)
{
    mesh.request_face_normals();
    mesh.update_face_normals();

    RobustDecimater decimater(mesh);

    ModAspectRatioH hModAspectRatio;
    decimater.add(hModAspectRatio);
    decimater.module(hModAspectRatio).set_binary(true);
    decimater.module(hModAspectRatio).set_aspect_ratio(20);

    ModFramewiseQuadricH hModFramewiseQuadric;
    decimater.add(hModFramewiseQuadric);
    decimater.module(hModFramewiseQuadric).set_max_err(maxQuadricError, false);
    decimater.module(hModFramewiseQuadric).set_frame_skip(frameSkipQuadric);
    decimater.module(hModFramewiseQuadric).set_epicenter_vars(epicenters, meanDistsFromEpicenters);

    ModFramewiseNormalH hModFramewiseNormal;
    decimater.add(hModFramewiseNormal);
    decimater.module(hModFramewiseNormal).set_normal_deviation(maxNormalAngle);
    decimater.module(hModFramewiseNormal).set_frame_skip(frameSkipNormal);
    decimater.module(hModFramewiseNormal).set_epicenter_vars(epicenters, meanDistsFromEpicenters);

    ModFramewiseBoundaryH hModFramewiseBoundary;
    decimater.add(hModFramewiseBoundary);
    decimater.module(hModFramewiseBoundary).set_boundary_angle(maxBoundaryAngle);
    decimater.module(hModFramewiseBoundary).set_frame_skip(frameSkipBoundary);
    decimater.module(hModFramewiseBoundary).set_epicenter_vars(epicenters, meanDistsFromEpicenters);

    decimater.initialize();
    decimater.decimate_to_faces(0, nFaces);

    // Prepare garbage collection
    std::vector<VHandle> vs;
    std::vector<VHandle*> vPtrs;
    std::map<int, VHandle*> oldIdx2ptr;
    std::vector<HEHandle*> trash1;
    std::vector<FHandle*> trash2;

    for (const VHandle& vh : mesh.vertices())
        if (mesh.data(vh).duplicate.is_valid())
            vs.emplace_back(vh);

    for (VHandle& vh : vs)
    {
        vPtrs.emplace_back(&vh);
        oldIdx2ptr[vh.idx()] = &vh;
    }

    mesh.garbage_collection(vPtrs, trash1, trash2);

    // // TODO save epicenter info somewhere where it makes sense and move visualization stuff there
    // uint numFrames = mesh.data(*(mesh.vertices_begin())).node->positions.rows();
    // for (uint frame = 0; frame < numFrames; frame++)
    // {
    //     std::cout << meanDistsFromEpicenters(frame) << std::endl;
    // }
    // Node::Ptr epiNode1 = std::make_shared<Node>(-1, epicenters);
    // for (uint frame = 0; frame < numFrames; frame++)
    // {
    //     epicenters(frame, 0) -= meanDistsFromEpicenters(frame);
    //     epicenters(frame, 2) += meanDistsFromEpicenters(frame);
    // }
    // Node::Ptr epiNode2 = std::make_shared<Node>(-1, epicenters);
    // for (uint frame = 0; frame < numFrames; frame++)
    // {
    //     epicenters(frame, 0) += 2 * meanDistsFromEpicenters(frame);
    // }
    // Node::Ptr epiNode3 = std::make_shared<Node>(-1, epicenters);
    // OMVec3 pos(epicenters.coeff(0,0), epicenters.coeff(0,1), epicenters.coeff(0,2));
    // VHandle v1 = mesh.add_vertex(pos), v2 = mesh.add_vertex(pos), v3 = mesh.add_vertex(pos);
    // mesh.data(v1).node = epiNode1;
    // mesh.data(v2).node = epiNode2;
    // mesh.data(v3).node = epiNode3;
    // mesh.add_face(std::vector<VHandle>({v1, v2, v3}));
    // MeshAnalyzer::render(mesh);
    // Fix duplicate vertex handles
    for (const VHandle& vh : vs)
        mesh.data(vh).duplicate = *oldIdx2ptr[mesh.data(vh).duplicate.idx()];
}

bool MeshDecimater::decimateSimple(std::vector<Part::Ptr>& parts)
{
    return decimatePartsErrorBound(parts, 1000.0, 10.0, 10.0, 3, 9, 20);
}

bool MeshDecimater::decimatePartsErrorBound(std::vector<Part::Ptr>& parts,
                                            float maxQuadricError,
                                            float maxNormalAngle,
                                            float maxBoundaryAngle,
                                            uint frameSkipQuadric,
                                            uint frameSkipNormal,
                                            uint frameSkipBoundary)
{
    uint v_before = 0;
    uint v_after = 0;
    uint f_before = 0;
    uint f_after = 0;

    uint maxVLog = 10000000;
    uint maxVRender = 300000;
    uint minVLog, minVRender;
    queryLogParts(minVLog, minVRender);
    for (Part::Ptr& partptr : parts)
    {
        CMesh& mesh = partptr->mesh;

        if (mesh.n_vertices() == 0)
            continue;

        v_before += mesh.n_vertices();
        f_before += mesh.n_faces();

        for (VHandle v : mesh.vertices())
        {
            mesh.status(v).set_locked(parts.size() > 1 && mesh.data(v).node->referencingParts > 1
                                      || mesh.data(v).node->referencingParts == std::numeric_limits<uint>::max());
        }

        bool log = renderAndLog(mesh, true, minVLog, maxVLog, minVRender, maxVRender);

        MatX3 epicenters;
        VecX meanDistsFromEpicenters;
        // MeshAnalyzer::getEpicenter(mesh, epicenters, meanDistsFromEpicenters);

        decimate(mesh,
                 epicenters,
                 meanDistsFromEpicenters,
                 maxQuadricError,
                 maxNormalAngle,
                 maxBoundaryAngle,
                 frameSkipQuadric,
                 frameSkipNormal,
                 frameSkipBoundary);

        v_after += mesh.n_vertices();
        f_after += mesh.n_faces();

        if (log)
            renderAndLog(mesh, false, 0, maxVLog, 0, maxVRender);
    }

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

// TODO make this instance based instead of class based with so many params
bool MeshDecimater::decimateScene(Scene::Ptr scene,
                                  uint nFaces,
                                  float maxNormalAngle,
                                  float maxBoundaryAngle,
                                  uint frameSkipQuadric,
                                  uint frameSkipNormal,
                                  uint frameSkipBoundary)
{

    uint v_before = 0;
    uint v_after = 0;
    uint f_before = 0;
    uint f_after = 0;

    uint maxVLog = 10000000;
    uint maxVRender = 300000;
    uint minVLog, minVRender;
    queryLogParts(minVLog, minVRender);

    CMesh& mesh = scene->mesh;

    v_before += mesh.n_vertices();
    f_before += mesh.n_faces();

    // Check for PLINKS and BEAMS connected to vertices
    for (VHandle v : mesh.vertices())
        mesh.status(v).set_locked(mesh.data(v).node->referencingParts == std::numeric_limits<uint>::max());

    bool log = renderAndLog(mesh, true, minVLog, maxVLog, minVRender, maxVRender);

    MatX3 epicenters;
    VecX meanDistsFromEpicenters;
    MeshAnalyzer::getEpicenter(mesh, epicenters, meanDistsFromEpicenters);

    decimate(mesh,
             epicenters,
             meanDistsFromEpicenters,
             FLT_MAX,
             maxNormalAngle,
             maxBoundaryAngle,
             frameSkipQuadric,
             frameSkipNormal,
             frameSkipBoundary,
             nFaces);

    v_after += mesh.n_vertices();
    f_after += mesh.n_faces();

    if (log)
        renderAndLog(mesh, false, 0, maxVLog, 0, maxVRender);

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

} // namespace c2m
