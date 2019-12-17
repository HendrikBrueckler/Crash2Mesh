#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
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

bool MeshDecimater::decimateSimple(std::vector<Part::Ptr> parts)
{
    uint v_before = 0;
    uint v_after = 0;
    uint f_before = 0;
    uint f_after = 0;

    std::set<uint> partNums;
    std::cout << "Wanna visualize parts ? (y/N)" << std::endl;
    char query = 'n';
    std::cin >> query;
    while (query == 'y' || query == 'Y')
    {
        uint partNum = -1;
        std::cout << "Enter part to visualize " << std::endl;
        std::cin >> partNum;
        partNums.emplace(partNum);
        std::cout << "Wanna visualize more parts ? (y/N)" << std::endl;
        std::cin >> query;
    }

    for (Part::Ptr& partptr : parts)
    {
        CMesh& mesh = partptr->mesh;

        if (mesh.n_vertices() == 0)
            continue;

        mesh.request_face_normals();
        mesh.update_face_normals();
        for (VHandle v : mesh.vertices())
        {
            mesh.status(v).set_locked(mesh.status(v).fixed_nonmanifold() || (parts.size() > 1 && mesh.data(v).node->referencingParts > 1));
        }

        bool log = false;
        if (partNums.find(partptr->ID) != partNums.end())
        {
            log = true;
            Logger& lout = Logger::lout(Logger::DEBUG);
            lout << "Part " << partptr->ID << " meshinfo before decimation:" << std::endl
                 << MeshAnalyzer::getInfo(mesh).print();
            if (mesh.n_vertices() < 200000)
            {
                MeshAnalyzer::render(mesh);
            }
        }

        Decimater decimater(mesh);
        // ModQuadricH hModQuadric;
        // decimater.add(hModQuadric);
        // decimater.module(hModQuadric).set_max_err(5000000, false);
        // ModHausdorffH hModHausdorff;
        // decimater.add(hModHausdorff);
        // decimater.module(hModHausdorff).set_binary(true);
        // decimater.module(hModHausdorff).set_tolerance(2);
        // ModNormalFlippingH hModNormalFlipping;
        // decimater.add(hModNormalFlipping);
        // decimater.module(hModNormalFlipping).set_max_normal_deviation(5);
        // ModNormalDeviationH hModNormalDeviation;
        // decimater.add(hModNormalDeviation);
        // decimater.module(hModNormalDeviation).set_normal_deviation(10);
        // ModRoundnessH hModRoundness;
        // decimater.add(hModRoundness);
        // decimater.module(hModRoundness).set_min_angle(5, true);
        ModAspectRatioH hModAspectRatio;
        decimater.add(hModAspectRatio);
        decimater.module(hModAspectRatio).set_binary(true);
        decimater.module(hModAspectRatio).set_aspect_ratio(20);
        ModFramewiseQuadricH hModFramewiseQuadric;
        decimater.add(hModFramewiseQuadric);
        decimater.module(hModFramewiseQuadric).set_max_err(1000, false);
        decimater.module(hModFramewiseQuadric).set_frame_skip(3);
        ModFramewiseNormalH hModFramewiseNormal;
        decimater.add(hModFramewiseNormal);
        decimater.module(hModFramewiseNormal).set_normal_deviation(10);
        decimater.module(hModFramewiseNormal).set_frame_skip(9);
        ModFramewiseBoundaryH hModFramewiseBoundary;
        decimater.add(hModFramewiseBoundary);
        decimater.module(hModFramewiseBoundary).set_boundary_angle(10);
        decimater.module(hModFramewiseBoundary).set_frame_skip(39);

        v_before += mesh.n_vertices();
        f_before += mesh.n_faces();

        decimater.initialize();
        decimater.decimate();
        mesh.garbage_collection();

        v_after += mesh.n_vertices();
        f_after += mesh.n_faces();

        if (log)
        {
            Logger& lout = Logger::lout(Logger::DEBUG);
            lout << "Part " << partptr->ID << " meshinfo after decimation:" << std::endl
                 << MeshAnalyzer::getInfo(mesh).print();
            if (mesh.n_vertices() < 200000)
            {
                MeshAnalyzer::render(mesh);
            }
        }
    }

    Logger::lout(Logger::INFO) << "Reduced meshes from a total of " << v_before << " vertices and " << f_before
                               << " faces to a total of " << v_after << " vertices and " << f_after << " faces."
                               << std::endl;

    return true;
}

} // namespace c2m
