#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
#include <crash2mesh/algorithm/modules/framewise_quadric.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>
#include <OpenMesh/Tools/Decimater/ModHausdorffT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalFlippingT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

namespace c2m
{
using Decimater = OpenMesh::Decimater::DecimaterT<Mesh>;
using ModQuadricH = OpenMesh::Decimater::ModQuadricT<Mesh>::Handle;
using ModAspectRatioH = OpenMesh::Decimater::ModAspectRatioT<Mesh>::Handle;
using ModHausdorffH = OpenMesh::Decimater::ModHausdorffT<Mesh>::Handle;
using ModNormalFlippingH = OpenMesh::Decimater::ModNormalFlippingT<Mesh>::Handle;
using ModNormalDeviationH = OpenMesh::Decimater::ModNormalDeviationT<Mesh>::Handle;
using ModFramewiseQuadricH = ModFramewiseQuadricT<Mesh>::Handle;

bool MeshDecimater::decimateSimple(std::vector<Part::Ptr> parts)
{

    for (Part::Ptr& partptr : parts)
    {
        Mesh& mesh = partptr->mesh;

        if (mesh.n_vertices() == 0)
            continue;


        mesh.request_face_normals();
        mesh.update_face_normals();
        for (VHandle v : mesh.vertices())
        {
            mesh.status(v).set_locked(mesh.data(v).fixed || mesh.is_boundary(v));
        }

        bool log = false;
        bool draw = false;
        if (mesh.n_vertices() > 10)
        {
            log = true;
            Logger& lout = Logger::lout(Logger::DEBUG);
            lout << "Part " << partptr->ID << " meshinfo before decimation:" << std::endl
                 << MeshAnalyzer::getInfo(mesh).print();
            if (mesh.n_vertices() > 10)
            {
                draw = true;
                MeshAnalyzer::render(mesh);
            }
        }

        Decimater decimater(mesh);
        // ModQuadricH hModQuadric;
        // decimater.add(hModQuadric);
        // decimater.module(hModQuadric).set_max_err(5000000, false);
        ModFramewiseQuadricH hModFramewiseQuadric;
        decimater.add(hModFramewiseQuadric);
        decimater.module(hModFramewiseQuadric).set_max_err(500000000000000, false);
        ModAspectRatioH hModAspectRatio;
        decimater.add(hModAspectRatio);
        decimater.module(hModAspectRatio).set_binary(true);
        decimater.module(hModAspectRatio).set_aspect_ratio(100);
        // ModHausdorffH hModHausdorff;
        // decimater.add(hModHausdorff);
        // decimater.module(hModHausdorff).set_binary(true);
        // decimater.module(hModHausdorff).set_tolerance(2);
        // ModNormalFlippingH hModNormalFlipping;
        // decimater.add(hModNormalFlipping);
        // decimater.module(hModNormalFlipping).set_max_normal_deviation(5);
        ModNormalDeviationH hModNormalDeviation;
        decimater.add(hModNormalDeviation);
        decimater.module(hModNormalDeviation).set_normal_deviation(10);
        decimater.initialize();
        decimater.decimate();
        mesh.garbage_collection();

        if (log)
        {
            Logger& lout = Logger::lout(Logger::DEBUG);
            lout << "Part " << partptr->ID << " meshinfo after decimation:" << std::endl
                 << MeshAnalyzer::getInfo(mesh).print();
            if (draw)
            {
                MeshAnalyzer::render(mesh);
            }
        }
    }

    return true;
}

} // namespace c2m
