#include <crash2mesh/algorithm/mesh_analyzer.hpp>

#include <crash2mesh/util/logger.hpp>
#include <crash2mesh/viewer/animation_viewer.hpp>

#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>

#include <map>
#include <sstream>
#include <vector>

namespace c2m
{
using std::map;
using std::stringstream;
using std::vector;

std::string MeshInfo::print() const
{
    stringstream info;
    info << "=============BEGIN============\n"
         << "\tNumber of nodes (finite element): " << numNodes << "\n"
         << "\tNumber of connected elements (finite element): " << numConnectedElements << "\n\n"

         << "\tNumber of vertices: " << numVertices << "\n"
         << "\tNumber of faces: " << numFaces << "\n\n"

         << "\tNumber of boundary vertices: " << numBoundaryVertices << "\n"
         << "\tNumber of fixed vertices: " << numFixedVertices << "\n"
         << "\tNumber of multi-part vertices: " << numMultiPartVertices << "\n\n"

         << "\tSize (x, y, z): " << bboxSize(0) << ", " << bboxSize(1) << ", " << bboxSize(2) << "\n\n";

    if (meanPlasticStrain.size() != 0)
    {
        info << "\tmeanPlasticStrains: " << meanPlasticStrain(0);
        for (uint i = 1; i < meanPlasticStrain.size(); i += meanPlasticStrain.size() / 10)
        {
            info << ", " << meanPlasticStrain(i);
        }
        info << "\n";
    }
    if (maxPlasticStrain.size() != 0)
    {
        info << "\tmaxPlasticStrains: " << maxPlasticStrain(0);
        for (uint i = 1; i < maxPlasticStrain.size(); i += maxPlasticStrain.size() / 10)
        {
            info << ", " << maxPlasticStrain(i);
        }
        info << "\n";
    }

    info << "==============END=============\n";
    return info.str();
}

MeshInfo MeshAnalyzer::getInfo(const CMesh& mesh)
{
    MeshInfo info;

    info.numVertices = mesh.n_vertices();
    if (info.numVertices == 0)
        return info;
    info.numFaces = mesh.n_faces();

    info.numBoundaryVertices = 0;
    info.numFixedVertices = 0;
    info.numMultiPartVertices = 0;
    std::set<Node::Ptr> nodes;
    for (VHandle v : mesh.vertices())
    {
        nodes.emplace(mesh.data(v).node);
        if (mesh.is_boundary(v))
        {
            info.numBoundaryVertices++;
        }
        if (mesh.data(v).fixed)
        {
            info.numFixedVertices++;
        }
        if (mesh.data(v).node->referencingParts > 1)
        {
            info.numMultiPartVertices++;
        }
    }
    info.numNodes = nodes.size();

    info.bboxMin = info.bboxMax = (*nodes.begin())->positions;
    for (const Node::Ptr& node : nodes)
    {
        const MatX3& positions = node->positions;
        info.bboxMin = info.bboxMin.cwiseMin(positions);
        info.bboxMax = info.bboxMax.cwiseMax(positions);
    }
    info.bboxSize = (info.bboxMax.row(0) - info.bboxMin.row(0)).transpose();

    info.meanPlasticStrain = VecX::Zero((*nodes.begin())->positions.rows());
    info.minPlasticStrain = VecX::Zero((*nodes.begin())->positions.rows());
    info.maxPlasticStrain = VecX::Zero((*nodes.begin())->positions.rows());
    float sumsurface = 0;
    std::set<Element2D::Ptr> surfaceElements;
    for (FHandle f : mesh.all_faces())
    {
        const Element2D::Ptr& elem = mesh.data(f).element;
        surfaceElements.emplace(elem);
        float faceArea = mesh.calc_face_area(f);
        info.meanPlasticStrain += faceArea * elem->plasticStrains;
        info.minPlasticStrain = info.minPlasticStrain.cwiseMin(elem->plasticStrains);
        info.maxPlasticStrain = info.maxPlasticStrain.cwiseMax(elem->plasticStrains);
        sumsurface += faceArea;
    }
    info.meanPlasticStrain /= sumsurface;
    info.numConnectedElements = surfaceElements.size();

    return info;
}

void MeshAnalyzer::render(const CMesh& mesh)
{
    if (mesh.n_vertices() == 0)
        return;

    std::cout.setstate(std::ios_base::failbit);
    AnimationViewer viewer("Mesh View");
    std::cout.clear();

    for (uint i = 0; i < mesh.data(*mesh.vertices_begin()).node->positions.rows(); i++)
    {
        easy3d::SurfaceMesh* drawableMesh = new easy3d::SurfaceMesh;
        easy3d::SurfaceMesh::VertexProperty colors = drawableMesh->add_vertex_property<easy3d::vec3>("v:color");
        easy3d::SurfaceMesh::VertexProperty strains = drawableMesh->add_vertex_property<float>("v:strain");
        easy3d::SurfaceMesh::FaceProperty fstrains = drawableMesh->add_face_property<float>("f:strain");
        map<VHandle, easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex;
        for (VHandle v : mesh.vertices())
        {
            Vec3 position = mesh.data(v).node->positions.row(i).transpose();
            vertexToDrawableVertex[v] = drawableMesh->add_vertex(easy3d::vec3(position(0), position(1), position(2)));
            float strain = 0;
            for (FHandle f : mesh.vf_range(v))
            {
                strain = std::max(strain, mesh.data(f).element->plasticStrains(i));
            }
            strains[vertexToDrawableVertex[v]] = strain * 20;
            if (mesh.status(v).fixed_nonmanifold())
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.8, 0.0, 0.0);
            }
            else if (mesh.data(v).node->referencingParts > 1)
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.9, 0.6, 0.0);
            }
            else if (mesh.is_boundary(v))
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.8, 0.8, 0.0);
            }
            else
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.0, 0.6, 0.0);
            }
        }

        for (FHandle f : mesh.faces())
        {
            vector<easy3d::SurfaceMesh::Vertex> vs;
            for (VHandle v : mesh.fv_range(f))
            {
                vs.emplace_back(vertexToDrawableVertex[v]);
            }
            easy3d::SurfaceMesh::Face fd = drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
            fstrains[fd] = mesh.data(f).element->plasticStrains(i) * 20;
        }

        easy3d::PointsDrawable* drawablePoints = drawableMesh->add_points_drawable("points");
        drawablePoints->set_point_size(5);
        drawablePoints->set_per_vertex_color(true);
        drawablePoints->update_vertex_buffer(drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector());
        drawablePoints->update_color_buffer(drawableMesh->get_vertex_property<easy3d::vec3>("v:color").vector());

        // TODO visualize
        easy3d::TrianglesDrawable* drawableTriangles = drawableMesh->add_triangles_drawable("surface");

        bool faceNormals = true;
        if (faceNormals)
        {
            vector<easy3d::vec3> points;
            vector<easy3d::vec3> normals;
            vector<easy3d::vec3> strainColors;
            for (easy3d::SurfaceMesh::Face f : drawableMesh->faces())
            {
                easy3d::SurfaceMesh::Halfedge he = drawableMesh->halfedge(f);
                easy3d::vec3 normal = drawableMesh->compute_face_normal(f);
                for (int j = 0; j < 3; j++)
                {
                    points.emplace_back(drawableMesh->position(drawableMesh->to_vertex(he)));
                    float strain = fstrains[f];
                    strainColors.emplace_back(easy3d::vec3(1.0, 1.0 - strain, 1.0 - strain));
                    normals.emplace_back(normal);
                    he = drawableMesh->next_halfedge(he);
                }
            }
            drawableTriangles->update_vertex_buffer(points);
            drawableTriangles->update_normal_buffer(normals);
            drawableTriangles->update_color_buffer(strainColors);
            drawableTriangles->set_per_vertex_color(true);

            vector<unsigned int> indices(3 * drawableMesh->faces_size());
            std::iota(indices.begin(), indices.end(), 0);
            drawableTriangles->update_index_buffer(indices);
        }
        else
        {
            drawableTriangles->update_vertex_buffer(
                drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector());
            drawableMesh->update_vertex_normals();
            drawableTriangles->update_normal_buffer(
                drawableMesh->get_vertex_property<easy3d::vec3>("v:normal").vector());
            vector<easy3d::vec3> strainColors;
            for (easy3d::SurfaceMesh::Vertex v : drawableMesh->vertices())
            {
                strainColors.emplace_back(easy3d::vec3(strains[v], 1.0 - strains[v], 1.0 - strains[v]));
            }
            drawableTriangles->update_color_buffer(strainColors);

            vector<unsigned int> indices;
            for (auto f : drawableMesh->faces())
            {
                vector<unsigned int> vts;
                for (auto v : drawableMesh->vertices(f))
                    vts.push_back(v.idx());

                indices.insert(indices.end(), vts.begin(), vts.end());
            }
            drawableTriangles->update_index_buffer(indices);
        }

        viewer.add_model(drawableMesh);
    }
    viewer.run();
}

} // namespace c2m
