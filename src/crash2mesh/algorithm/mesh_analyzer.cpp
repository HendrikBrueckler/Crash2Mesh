#include <crash2mesh/algorithm/mesh_analyzer.hpp>

#include <crash2mesh/util/logger.hpp>
#include <crash2mesh/viewer/animation_viewer.hpp>
#include <crash2mesh/viewer/imgui_viewer.hpp>

#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/viewer/drawable_triangles.h>

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

         << "\tNumber of duplicate faces: " << numDuplicateFaces << "\n\n"

         << "\tNumber of multi-part vertices: " << numMultiPartVertices << "\n"
         << "\tNumber of boundary vertices: " << numBoundaryVertices << "\n"
         << "\tNumber of complex vertices: " << numComplexVertices << "\n"
         << "\tNumber of locked vertices: " << numLockedVertices << "\n\n"

         << "\tSize (x, y, z): " << bboxSize(0) << ", " << bboxSize(1) << ", " << bboxSize(2) << "\n\n";

    if (meanPlasticStrain.size() != 0)
    {
        info << "\tmeanPlasticStrains: " << meanPlasticStrain(0);
        for (int i = 1; i < meanPlasticStrain.size(); i += std::max(1, static_cast<int>(meanPlasticStrain.size()) / 10))
        {
            info << ", " << meanPlasticStrain(i);
        }
        info << "\n";
    }
    if (maxPlasticStrain.size() != 0)
    {
        info << "\tmaxPlasticStrains: " << maxPlasticStrain(0);
        for (int i = 1; i < maxPlasticStrain.size(); i += std::max(1, static_cast<int>(maxPlasticStrain.size()) / 10))
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
    info.numLockedVertices = 0;
    info.numComplexVertices = 0;
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
            info.numComplexVertices++;
        }
        if (mesh.status(v).locked())
        {
            info.numLockedVertices++;
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
    for (FHandle f : mesh.faces())
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

    info.numDuplicateFaces = 0;
    for (const FHandle& f : mesh.all_faces())
    {
        std::set<Node::Ptr> fNodes;
        for (const VHandle& v : mesh.fv_range(f))
        {
            fNodes.emplace(mesh.data(v).node);
        }

        vector<vector<HEHandle>> dupes;
        vector<vector<HEHandle>> oppositeDupes;
        for (const HEHandle& he: mesh.fh_range(f))
        {
            dupes.emplace_back(MeshAnalyzer::dupes(mesh, he));
            oppositeDupes.emplace_back(MeshAnalyzer::dupes(mesh, mesh.opposite_halfedge_handle(he)));
        }

        std::set<FHandle> neighborFaces;
        for (const  vector<vector<HEHandle>>& edgeDupes: {dupes, oppositeDupes})
            for (const vector<HEHandle>& edgeDupesPart: edgeDupes)
                for (const HEHandle& edgeDupe: edgeDupesPart)
                    neighborFaces.emplace(mesh.face_handle(edgeDupe));

        for (const FHandle& fOther : neighborFaces)
        {
            if (fOther == f || !fOther.is_valid())
                continue;

            std::set<Node::Ptr> fOtherNodes;
            for (const VHandle& v : mesh.fv_range(fOther))
            {
                fOtherNodes.emplace(mesh.data(v).node);
            }
            if (fNodes == fOtherNodes)
            {
                info.numDuplicateFaces += 1;
            }
        }
    }
    info.numDuplicateFaces /= 2;

    return info;
}

void MeshAnalyzer::render(const CMesh& _mesh, const MatX3* epis, const VecX* meanDists)
{
    if (_mesh.n_vertices() == 0)
        return;

    CMesh mesh(_mesh);

    // Add epicenter triangle if info was supplied
    if (epis != nullptr && meanDists != nullptr && epis->size() != 0 && meanDists->size() != 0)
    {
        MatX3 epicenters = *epis;
        VecX meanDistsFromEpicenters = *meanDists;

        uint numFrames = mesh.data(*(mesh.vertices_begin())).node->positions.rows();
        Node::Ptr epiNode1 = std::make_shared<Node>(-1, epicenters);
        for (uint frame = 0; frame < numFrames; frame++)
        {
            epicenters(frame, 0) -= meanDistsFromEpicenters(frame);
            epicenters(frame, 2) += meanDistsFromEpicenters(frame);
        }
        Node::Ptr epiNode2 = std::make_shared<Node>(-1, epicenters);
        for (uint frame = 0; frame < numFrames; frame++)
        {
            epicenters(frame, 0) += 2 * meanDistsFromEpicenters(frame);
        }
        Node::Ptr epiNode3 = std::make_shared<Node>(-1, epicenters);
        OMVec3 pos(epicenters.coeff(0,0), epicenters.coeff(0,1), epicenters.coeff(0,2));
        VHandle v1 = mesh.add_vertex(pos), v2 = mesh.add_vertex(pos), v3 = mesh.add_vertex(pos);
        mesh.data(v1).node = epiNode1;
        mesh.data(v2).node = epiNode2;
        mesh.data(v3).node = epiNode3;
        mesh.add_face(std::vector<VHandle>({v1, v2, v3}));
    }

    // Silence annyoing messages
    std::cout.setstate(std::ios_base::failbit);
    std::cerr.setstate(std::ios_base::failbit);
    ImGuiViewer viewer("Mesh View");
    std::cout.clear();
    std::cerr.clear();

    // For each frame...
    for (uint i = 0; i < mesh.data(*mesh.vertices_begin()).node->positions.rows(); i++)
    {
        // Create a drawable model
        easy3d::SurfaceMesh* drawableMesh = new easy3d::SurfaceMesh;
        easy3d::SurfaceMesh::VertexProperty colors = drawableMesh->add_vertex_property<easy3d::vec3>("v:marking");
        easy3d::SurfaceMesh::VertexProperty strains = drawableMesh->add_vertex_property<float>("v:strain");
        easy3d::SurfaceMesh::FaceProperty fstrains = drawableMesh->add_face_property<float>("f:strain");

        // Copy over vertices and their properties to drawable model
        map<VHandle, easy3d::SurfaceMesh::Vertex> vertexToDrawableVertex;
        for (VHandle v : mesh.vertices())
        {
            Vec3 position = mesh.data(v).node->positions.row(i).transpose();
            vertexToDrawableVertex[v] = drawableMesh->add_vertex(easy3d::vec3(position(0), position(1), position(2)));
            float strain = 0;
            for (FHandle f : mesh.vf_range(v))
            {
                if (mesh.data(f).element)
                    strain = std::max(strain, mesh.data(f).element->plasticStrains(i));
                else
                    strain = 1.0;
            }
            strains[vertexToDrawableVertex[v]] = strain * 20;
            if (mesh.status(v).fixed_nonmanifold())
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.8f, 0.0f, 0.0f);
            }
            else if (mesh.data(v).node->referencingParts > 1)
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.9f, 0.6f, 0.0f);
            }
            else if (mesh.is_boundary(v))
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.8f, 0.8f, 0.0f);
            }
            else
            {
                colors[vertexToDrawableVertex[v]] = easy3d::vec3(0.0f, 0.6f, 0.0f);
            }
        }

        // Copy over faces and their properties to drawable model
        for (FHandle f : mesh.faces())
        {
            vector<easy3d::SurfaceMesh::Vertex> vs;
            for (VHandle v : mesh.fv_range(f))
            {
                vs.emplace_back(vertexToDrawableVertex[v]);
            }
            easy3d::SurfaceMesh::Face fd = drawableMesh->add_triangle(vs[0], vs[1], vs[2]);
            if (mesh.data(f).element)
                fstrains[fd] = mesh.data(f).element->plasticStrains(i) * 20;
            else
                fstrains[fd] = 1.0;
        }

        // Visualize points as billboards
        easy3d::PointsDrawable* drawablePoints = drawableMesh->add_points_drawable("points");
        drawablePoints->set_point_size(5);
        drawablePoints->set_per_vertex_color(true);
        drawablePoints->update_vertex_buffer(drawableMesh->get_vertex_property<easy3d::vec3>("v:point").vector());
        drawablePoints->update_color_buffer(drawableMesh->get_vertex_property<easy3d::vec3>("v:marking").vector());

        // Visualize triangles with face normals (vertex normal code is below but shouldnt ever be used)
        easy3d::TrianglesDrawable* drawableTriangles = drawableMesh->add_triangles_drawable("surface");

#if 1
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
#else
            // Keep the code here just in case...
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
#endif

        // Silence annyoing messages
        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
        viewer.add_model(drawableMesh);
        std::cout.clear();
        std::cerr.clear();
    }
    // Silence annyoing messages
    std::cout.setstate(std::ios_base::failbit);
    std::cerr.setstate(std::ios_base::failbit);
    viewer.run();
    std::cout.clear();
    std::cerr.clear();
}

void MeshAnalyzer::getEpicenter(CMesh& mesh, MatX3& epicenters, VecX& meanDists)
{
    if (mesh.n_vertices() == 0)
        return;

    Logger::lout(Logger::INFO) << "Calculating epicenter of crash" << std::endl;
    uint numFrames = mesh.data(*(mesh.vertices_begin())).node->positions.rows();

    // Calc epicenter of crash
    VecX sumOfWeights = VecX::Zero(numFrames);
    epicenters = MatX3::Zero(numFrames, 3);
    // for (VHandle v : mesh.vertices())
    // {
    //     const MatX3& positions = mesh.data(v).node->positions;
    //     MatX3 relativeDisplacements(MatX3::Zero(numFrames, 3));
    //     VecX sumOfInverseEdgeLengths(VecX::Zero(numFrames));
    //     for (VHandle vNeighbor : mesh.vv_range(v))
    //     {
    //         const MatX3& positionsNeighbor = mesh.data(vNeighbor).node->positions;
    //         VecX edgeLengths = (positions - positionsNeighbor).rowwise().norm();
    //         sumOfInverseEdgeLengths += edgeLengths.cwiseInverse();
    //         const MatX3 displacementsNeighbor = (positionsNeighbor.rowwise() - positionsNeighbor.row(0)).array().colwise() / edgeLengths.array();
    //         relativeDisplacements -= displacementsNeighbor;
    //     }
    //     relativeDisplacements.array().colwise() /= sumOfInverseEdgeLengths.array();
    //     relativeDisplacements += positions.rowwise() - positions.row(0);
    //     VecX weights = relativeDisplacements.rowwise().norm();
    //     epicenters += MatX3(positions.array().colwise() * weights.array());
    //     sumOfWeights += weights;
    // }
    for (FHandle f : mesh.faces())
    {
        MatX3 positions = mesh.data(*(mesh.fv_begin(f))).node->positions;
        const VecX& strains = mesh.data(f).element->plasticStrains;
        VecX weight = VecX::Zero(strains.rows());
        float area = mesh.calc_face_area(f);
        for (uint frame = 1; frame < numFrames; frame++)
        {
            weight(frame) = (strains(frame) - strains(frame - 1)) * area;
        }
        for (uint frame = 0; frame < numFrames; frame++)
        {
            epicenters.row(frame) += weight(frame) * positions.row(frame);
            sumOfWeights(frame) += weight(frame);
        }
    }
    for (long frame = numFrames-1; frame >= 0; frame--)
    {
        // TODO more sensible value here
        if (sumOfWeights(frame) > 20.0)
            epicenters.row(frame) /= sumOfWeights(frame);
        else if (static_cast<uint>(frame + 1) < numFrames)
            epicenters.row(frame) = epicenters.row(frame+1);
    }
    meanDists = VecX::Zero(numFrames);
    for (VHandle v : mesh.vertices())
    {
        MatX3 position = mesh.data(v).node->positions;
        for (uint frame = 0; frame < numFrames; frame++)
        {
            if (epicenters.row(frame).squaredNorm() == 0.0)
                continue;
            meanDists(frame) += (position.row(frame) - epicenters.row(frame)).norm();
        }
    }
    meanDists /= static_cast<float>(mesh.n_vertices());
    Logger::lout(Logger::INFO) << "Finished calculating epicenter of crash" << std::endl;
}

void MeshAnalyzer::getEpicenter(std::vector<Part::Ptr>& parts, MatX3& epicenters, VecX& meanDists)
{
    if (parts.size() == 0)
        return;

    Logger::lout(Logger::INFO) << "Calculating epicenter of crash" << std::endl;

    // Get number of frames
    uint numFrames = 0;
    for (Part::Ptr& partptr: parts)
    {
        CMesh& mesh = partptr->mesh;
        if (mesh.n_vertices() == 0)
            continue;
        numFrames = mesh.data(*(mesh.vertices_begin())).node->positions.rows();
        break;
    }

    // Calc epicenter of crash
    VecX sumOfWeights = VecX::Zero(numFrames);
    epicenters = MatX3::Zero(numFrames, 3);
    for (Part::Ptr& partptr: parts)
    {
        CMesh& mesh = partptr->mesh;
        if (mesh.n_vertices() == 0)
            continue;
        // for (VHandle v : mesh.vertices())
        // {
        //     const MatX3& positions = mesh.data(v).node->positions;
        //     MatX3 relativeDisplacements(MatX3::Zero(numFrames, 3));
        //     VecX sumOfInverseEdgeLengths(VecX::Zero(numFrames));
        //     for (VHandle vNeighbor : mesh.vv_range(v))
        //     {
        //         const MatX3& positionsNeighbor = mesh.data(vNeighbor).node->positions;
        //         VecX edgeLengths = (positions - positionsNeighbor).rowwise().norm();
        //         sumOfInverseEdgeLengths += edgeLengths.cwiseInverse();
        //         const MatX3 displacementsNeighbor = (positionsNeighbor.rowwise() - positionsNeighbor.row(0)).array().colwise() / edgeLengths.array();
        //         relativeDisplacements -= displacementsNeighbor;
        //     }
        //     relativeDisplacements.array().colwise() /= sumOfInverseEdgeLengths.array();
        //     relativeDisplacements += positions.rowwise() - positions.row(0);
        //     VecX weights = relativeDisplacements.rowwise().norm();
        //     epicenters += MatX3(positions.array().colwise() * weights.array());
        //     sumOfWeights += weights;
        // }
        for (FHandle f : mesh.faces())
        {
            MatX3 positions = mesh.data(*(mesh.fv_begin(f))).node->positions;
            const VecX& strains = mesh.data(f).element->plasticStrains;
            VecX weight = VecX::Zero(strains.rows());
            float area = mesh.calc_face_area(f);
            for (uint frame = 1; frame < numFrames; frame++)
            {
                weight(frame) = (strains(frame) - strains(frame - 1)) * area;
            }
            for (uint frame = 0; frame < numFrames; frame++)
            {
                epicenters.row(frame) += weight(frame) * positions.row(frame);
                sumOfWeights(frame) += weight(frame);
            }
        }
    }
    for (long frame = numFrames-1; frame >= 0; frame--)
    {
        // TODO more sensible value here
        if (sumOfWeights(frame) > 100)
            epicenters.row(frame) /= sumOfWeights(frame);
        else if (static_cast<uint>(frame + 1) < numFrames)
            epicenters.row(frame) = epicenters.row(frame+1);
        else if (frame - 1 > 0 && sumOfWeights(frame - 1) > 100)
            epicenters.row(frame) = epicenters.row(frame - 1) / sumOfWeights(frame - 1);
        else
            epicenters.row(frame) = Vec3::Zero().transpose();
    }
    meanDists = VecX::Zero(numFrames);
    long nVertices = 0;
    for (Part::Ptr& partptr: parts)
    {
        CMesh& mesh = partptr->mesh;
        if (mesh.n_vertices() == 0)
            continue;
        nVertices += mesh.n_vertices();
        for (VHandle v : mesh.vertices())
        {
            MatX3 position = mesh.data(v).node->positions;
            for (uint frame = 0; frame < numFrames; frame++)
            {
                if (epicenters.row(frame).squaredNorm() == 0.0)
                    continue;
                meanDists(frame) += (position.row(frame) - epicenters.row(frame)).norm();
            }
        }
    }
    meanDists /= static_cast<float>(nVertices);
    Logger::lout(Logger::INFO) << "Finished calculating epicenter of crash" << std::endl;
}

std::vector<VHandle> MeshAnalyzer::dupes(const CMesh& mesh, const VHandle& vh)
{
    std::vector<VHandle> vDupes;
    VHandle vDupe = vh;
    assert(vDupe.is_valid());
    do
    {
        vDupes.emplace_back(vDupe);
        vDupe = mesh.data(vDupe).duplicate;
    } while (vDupe.is_valid() && vDupe != vh);

    return vDupes;
}

std::vector<HEHandle> MeshAnalyzer::dupes(const CMesh& mesh, const HEHandle& heh)
{
    assert(heh.is_valid());
    assert(mesh.from_vertex_handle(heh).is_valid());
    assert(mesh.to_vertex_handle(heh).is_valid());
    std::vector<VHandle> v0Dupes = dupes(mesh, mesh.from_vertex_handle(heh));
    std::vector<VHandle> v1Dupes = dupes(mesh, mesh.to_vertex_handle(heh));
    std::vector<HEHandle> heDupes;
    HEHandle heDupe = heh;
    for (const VHandle& v0Dupe : v0Dupes)
    {
        for (const VHandle& v1Dupe : v1Dupes)
        {
            heDupe = mesh.find_halfedge(v0Dupe, v1Dupe);
            if (heDupe.is_valid())
                heDupes.emplace_back(heDupe);
        }
    }

    return heDupes;
}

void MeshAnalyzer::calcPartCenters(std::vector<Part::Ptr>& parts)
{
    for (auto part: parts)
    {
        const CMesh& mesh = part->mesh;
        if (mesh.n_vertices() == 0)
            continue;

        part->centers = MatX3::Zero(mesh.data(*mesh.vertices_begin()).node->positions.rows(), 3);
        float sumOfAreas = 0.0f;
        for (FHandle f: mesh.faces())
        {
            MatX3 centroid(MatX3::Zero(mesh.data(*mesh.vertices_begin()).node->positions.rows(), 3));
            for (VHandle v: mesh.fv_range(f))
            {
                centroid += mesh.data(v).node->positions;
            }
            float area = mesh.calc_face_area(f);
            part->centers += 1.0f/3.0f * centroid * area;
            sumOfAreas += area;
        }
        part->centers /= sumOfAreas;
    }
}


} // namespace c2m
