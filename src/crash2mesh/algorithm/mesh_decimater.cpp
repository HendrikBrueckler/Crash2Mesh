#include <crash2mesh/algorithm/mesh_decimater.hpp>

#include <crash2mesh/algorithm/decimater/robust_decimater.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/modules/mod_boundary.hpp>
#include <crash2mesh/algorithm/modules/mod_normal.hpp>
#include <crash2mesh/algorithm/modules/mod_quadric.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>

#include <algorithm>
#include <execution>
#include <set>

namespace c2m
{
using ModAspectRatio = OpenMesh::Decimater::ModAspectRatioT<CMesh>;

using std::set;

std::mutex MeshDecimater::mutLog = std::mutex();
std::mutex MeshDecimater::mutRender = std::mutex();

void MeshDecimater::queryLogParts()
{
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

bool MeshDecimater::decimateSimple(std::vector<Part::Ptr>& parts)
{
    static MeshDecimater deci;
    static bool init(false);
    if (!init)
    {
        deci.useQuadric = true;
        deci.framesQuadric = 10;
        deci.maxQuadricError = 5000;
        deci.useNormalDeviation = true;
        deci.framesNormalDeviation = 10;
        deci.maxNormalDeviation = 10;
        deci.useBoundaryDeviation = true;
        deci.framesBoundaryDeviation = 3;
        deci.maxBoundaryDeviation = 10;
        deci.useAspectRatio = true;
        deci.maxAspectRatio = 20;
        deci.minVLog = 100;
        deci.minVRender = 1000;
        deci.maxVLog = 10000000;
        deci.maxVRender = 200000;
        init = true;
    }
    return deci.decimateParts(parts);
}

bool MeshDecimater::decimateParts(std::vector<Part::Ptr>& parts) const
{
    size_t v_before = 0;
    size_t v_after = 0;
    size_t f_before = 0;
    size_t f_after = 0;
#ifdef C2M_PARALLEL
    std::mutex mutVars;
#endif

    auto decimatePart = [&](Part::Ptr& partptr) {
        CMesh& mesh = partptr->mesh;

        if (mesh.n_vertices() == 0)
            return;

        for (VHandle v : mesh.vertices())
        {
            mesh.status(v).set_locked(parts.size() > 1 && mesh.data(v).node->referencingParts > 1
                                      || mesh.data(v).node->referencingParts == std::numeric_limits<uint>::max());
        }

#ifdef C2M_PARALLEL
        mutVars.lock();
#endif
        v_before += mesh.n_vertices();
        f_before += mesh.n_faces();
#ifdef C2M_PARALLEL
        mutVars.unlock();
#endif
        bool logged = log(mesh, true, false);
        bool rendered = render(mesh, true, false);
        decimate(mesh, 0, 0);
#ifdef C2M_PARALLEL
        mutVars.lock();
#endif
        v_after += mesh.n_vertices();
        f_after += mesh.n_faces();
#ifdef C2M_PARALLEL
        mutVars.unlock();
#endif
        log(mesh, false, logged);
        render(mesh, false, rendered);
    };

#ifndef C2M_PARALLEL
    for (Part::Ptr& partptr : parts)
        decimatePart(partptr);
#else
    std::for_each(std::execution::par_unseq, parts.begin(), parts.end(), decimatePart);
#endif

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

bool MeshDecimater::decimateScene(Scene::Ptr scene, uint nFaces, uint nVertices) const
{
    size_t v_before = 0;
    size_t v_after = 0;
    size_t f_before = 0;
    size_t f_after = 0;

    CMesh& mesh = scene->mesh;

    v_before += mesh.n_vertices();
    f_before += mesh.n_faces();

    // Check for PLINKS and BEAMS connected to vertices
    for (VHandle v : mesh.vertices())
        mesh.status(v).set_locked(mesh.data(v).node->referencingParts == std::numeric_limits<uint>::max());

    bool logged = log(mesh, true, false);
    bool rendered = render(mesh, true, false);

    decimate(mesh, nFaces, nVertices);

    v_after += mesh.n_vertices();
    f_after += mesh.n_faces();

    log(mesh, false, logged);
    render(mesh, false, rendered);

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

void MeshDecimater::decimate(CMesh& mesh, uint nFaces, uint nVertices) const
{
    // Create decimater and decimation modules
    RobustDecimater decimater(mesh);

    if (useQuadric || true) // Currently no alternative for a continuous module
    {
        ModQuadric::Handle hModFWQuadric;
        decimater.add(hModFWQuadric);
        decimater.module(hModFWQuadric).set_max_err(maxQuadricError, false);
        decimater.module(hModFWQuadric).set_frames(framesQuadric);
        decimater.module(hModFWQuadric).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
    }
    if (useNormalDeviation)
    {
        ModNormal::Handle hModFWNormal;
        decimater.add(hModFWNormal);
        decimater.module(hModFWNormal).set_normal_deviation(maxNormalDeviation);
        decimater.module(hModFWNormal).set_frames(framesNormalDeviation);
        decimater.module(hModFWNormal).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
    }
    if (useBoundaryDeviation)
    {
        ModBoundary::Handle hModFWBoundary;
        decimater.add(hModFWBoundary);
        decimater.module(hModFWBoundary).set_boundary_angle(maxBoundaryDeviation);
        decimater.module(hModFWBoundary).set_frames(framesBoundaryDeviation);
        decimater.module(hModFWBoundary).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
    }
    if (useAspectRatio)
    {
        ModAspectRatio::Handle hModAspectRatio;
        decimater.add(hModAspectRatio);
        decimater.module(hModAspectRatio).set_binary(true);
        decimater.module(hModAspectRatio).set_aspect_ratio(20);
    }

    // Init and decimate
    decimater.initialize();
    decimater.decimate_to_faces(nVertices, nFaces);

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

    // Do garbage collection
    mesh.garbage_collection(vPtrs, trash1, trash2);

    // Fix duplicate vertex handles, they lose validity when garbagecollecting
    for (const VHandle& vh : vs)
        mesh.data(vh).duplicate = *oldIdx2ptr[mesh.data(vh).duplicate.idx()];
}

bool MeshDecimater::log(const CMesh& mesh, bool preDecimation, bool force) const
{
    if (force || mesh.n_vertices() >= minVLog && mesh.n_vertices() <= maxVLog)
    {
#ifdef C2M_PARALLEL
        mutLog.lock();
#endif
        Logger& lout = Logger::lout(Logger::INFO);
        if (preDecimation)
            lout << "Meshinfo before decimation:" << std::endl;
        else
            lout << "Meshinfo after decimation:" << std::endl;
        lout << MeshAnalyzer::getInfo(mesh).print();
#ifdef C2M_PARALLEL
        mutLog.unlock();
#endif
        return true;
    }
    return false;
}

bool MeshDecimater::render(const CMesh& mesh, bool epicenterVis, bool force) const
{
    if (force || mesh.n_vertices() >= minVRender && mesh.n_vertices() <= maxVRender)
    {
#ifdef C2M_PARALLEL
        mutRender.lock();
#endif
        if (epicenterVis)
            MeshAnalyzer::render(mesh, &epicenters, &meanDistsFromEpicenters);
        else
            MeshAnalyzer::render(mesh, nullptr, nullptr);
#ifdef C2M_PARALLEL
        mutRender.unlock();
#endif
        return true;
    }
    return false;
}

} // namespace c2m
