#include <crash2mesh/algorithm/mesh_decimater.hpp>

#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/mesh_builder.hpp>
#include <crash2mesh/decimater/modules/mod_boundary.hpp>
#include <crash2mesh/decimater/modules/mod_normal.hpp>
#include <crash2mesh/decimater/modules/mod_quadric.hpp>
#include <crash2mesh/decimater/modules/mod_quadric_normal.hpp>
#include <crash2mesh/decimater/robust_decimater.hpp>
#include <crash2mesh/util/logger.hpp>
#include <crash2mesh/util/par_for.hpp>

#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>

#include <set>

namespace c2m
{
using ModAspectRatio = OpenMesh::Decimater::ModAspectRatioT<CMesh>;

using std::set;

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
std::mutex MeshDecimater::mutLog = std::mutex();
std::mutex MeshDecimater::mutRender = std::mutex();
#endif

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
        std::cout << "Enter maximum amount of vertices for logging " << std::endl;
        std::cin >> maxVLog;
        std::cout << "Enter maximum amount of vertices for rendering " << std::endl;
        std::cin >> maxVRender;
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
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
    std::mutex mutVars;
    Eigen::initParallel();
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

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutVars.lock();
#endif
        v_before += mesh.n_vertices();
        f_before += mesh.n_faces();
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutVars.unlock();
#endif
        bool logged = log(mesh, true, false, partptr->ID, partptr->userID);
        bool rendered = render(mesh, true, false);
        decimate(mesh, 0, 0, partptr->userID, true);
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutVars.lock();
#endif
        v_after += mesh.n_vertices();
        f_after += mesh.n_faces();
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutVars.unlock();
#endif
        log(mesh, false, logged, partptr->ID, partptr->userID);
        render(mesh, false, rendered);
    };

    parallel_for_each(parts, decimatePart);

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

    decimate(mesh, nFaces, nVertices, 0, true);

    v_after += mesh.n_vertices();
    f_after += mesh.n_faces();

    log(mesh, false, logged);
    render(mesh, false, rendered);

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

void MeshDecimater::decimate(CMesh& mesh, uint nFaces, uint nVertices, entid_t /*puid*/, bool forceRemove) const
{
    // Create decimater and decimation modules
    RobustDecimater decimater(mesh, quadricPositionOptimization);

    ModQuadricNormal::Handle hModFWQuadricNormal;
    ModQuadric::Handle hModFWQuadric;
    ModNormal::Handle hModFWNormal;
    ModBoundary::Handle hModFWBoundary;
    ModAspectRatio::Handle hModAspectRatio;

    if (useQuadric && useNormalDeviation && combineQuadricNormal)
    {
        decimater.add(hModFWQuadricNormal);
        // Shared stuff
        if (framesQuadric != framesNormalDeviation)
            throw std::logic_error("quadric and normal frames have to be the same when combining metrics!");
        decimater.module(hModFWQuadricNormal).set_num_frames(framesQuadric);
        decimater.module(hModFWQuadricNormal).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
        decimater.module(hModFWQuadricNormal).set_relative_weights(combineQuadricWeight, combineNormalWeight);
        // Quadric stuff
        decimater.module(hModFWQuadricNormal).set_max_err(maxQuadricError, false);
        decimater.module(hModFWQuadricNormal).set_area_weighting(quadricAreaWeighting);
        decimater.module(hModFWQuadricNormal).set_optimize_position(quadricPositionOptimization);
        decimater.module(hModFWQuadricNormal).set_boundary_quadrics(boundaryQuadrics);
        decimater.module(hModFWQuadricNormal).set_feature_quadrics(featureQuadrics);
        // Normal stuff
        decimater.module(hModFWQuadricNormal).set_max_normal_deviation(maxNormalDeviation);
        decimater.module(hModFWQuadricNormal).set_quadric_optimize_position(quadricPositionOptimization);
        decimater.module(hModFWQuadricNormal).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
    }
    else
    {
        if (useQuadric || true) // Currently no alternative for a continuous module
        {
            decimater.add(hModFWQuadric);
            decimater.module(hModFWQuadric).set_max_err(maxQuadricError, false);
            decimater.module(hModFWQuadric).set_num_frames(framesQuadric);
            decimater.module(hModFWQuadric).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
            decimater.module(hModFWQuadric).set_area_weighting(quadricAreaWeighting);
            decimater.module(hModFWQuadric).set_optimize_position(quadricPositionOptimization);
            decimater.module(hModFWQuadric).set_boundary_quadrics(boundaryQuadrics);
            decimater.module(hModFWQuadric).set_feature_quadrics(featureQuadrics);
            decimater.module(hModFWQuadric).set_binary(quadricExcludeOnly);
        }
        if (useNormalDeviation)
        {
            decimater.add(hModFWNormal);
            // TODO collect these magic numbers in collectors.hpp as static vars
            decimater.module(hModFWNormal).set_max_normal_deviation(maxNormalDeviation);
            decimater.module(hModFWNormal).set_num_frames(framesNormalDeviation);
            decimater.module(hModFWNormal).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
            decimater.module(hModFWNormal).set_binary(normalExcludeOnly);
        }
    }

    if (useBoundaryDeviation)
    {
        decimater.add(hModFWBoundary);
        decimater.module(hModFWBoundary).set_max_boundary_angle(maxBoundaryDeviation);
        decimater.module(hModFWBoundary).set_num_frames(framesBoundaryDeviation);
        decimater.module(hModFWBoundary).set_epicenter_vars(epicenters, meanDistsFromEpicenters);
        decimater.module(hModFWBoundary).set_binary(true);
    }
    if (useAspectRatio)
    {
        decimater.add(hModAspectRatio);
        decimater.module(hModAspectRatio).set_binary(true);
        decimater.module(hModAspectRatio).set_aspect_ratio(maxAspectRatio);
        decimater.module(hModAspectRatio).set_binary(true);
    }

    // Init and decimate
    decimater.initialize();
    decimater.decimate_to_faces(nVertices, nFaces);

    if (useQuadric && !quadricPositionOptimization && quadricPostProcessOptimize)
    {
        // This produces insane flickering, DONT USE
#if 0
        for (VHandle v : mesh.vertices())
        {
            if (!mesh.status(v).locked() && !mesh.data(v).duplicate.is_valid())
            {
                const auto& module = decimater.module(hModFWQuadric);
                uint numFrames = module.num_frames();
                for (size_t frame = 0; frame < numFrames; frame++)
                {
                    Vec3 optPos = mesh.data(v).node->positions.row(frame).transpose();
                    module.optimal_position(mesh.data(v).quadrics[frame], optPos);
                    mesh.data(v).node->positions.row(frame) = optPos.transpose();
                }
            }
        }
#endif
    }

    // TODO FIX THIS
    if (forceRemove)
    {
        CMesh::FaceVertexIter fv_it;
        VHandle v0, v1, v2;
        for (FHandle f: mesh.faces())
        {
            if (mesh.status(f).deleted())
                continue;
            fv_it = mesh.fv_begin(f);
            v0 = *(fv_it++);
            v1 = *(fv_it++);
            v2 = *fv_it;
            for (size_t frame = 0; frame < mesh.data(f).element->active.size(); frame++)
            {
                // TODO do this properly
                if (!mesh.data(f).element->active[frame])
                {
                    mesh.delete_face(f, true);
                    break;
                }
            }
        }
    }

    // Do garbage collection
    mesh.garbage_collection();

    // Relink duplicate chains
    MeshBuilder::relink(mesh);
}

bool MeshDecimater::log(const CMesh& mesh, bool preDecimation, bool force, partid_t pid, entid_t uid) const
{
    if (force || mesh.n_vertices() >= minVLog && mesh.n_vertices() <= maxVLog)
    {
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutLog.lock();
#endif
        Logger& lout = Logger::lout(Logger::INFO);
        if (preDecimation)
        {
            lout << "Part " << pid << " (" << uid << ") Meshinfo before decimation:" << std::endl;
        }
        else
        {
            lout << "Part " << pid << " (" << uid << ") Meshinfo after decimation:" << std::endl;
        }
        lout << MeshAnalyzer::getInfo(mesh).print();
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
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
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutRender.lock();
#endif
        if (epicenterVis)
            MeshAnalyzer::render(mesh, &epicenters, &meanDistsFromEpicenters);
        else
            MeshAnalyzer::render(mesh, nullptr, nullptr);
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mutRender.unlock();
#endif
        return true;
    }
    return false;
}

} // namespace c2m
