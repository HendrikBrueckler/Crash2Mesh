#include <crash2mesh/algorithm/mesh_builder.hpp>

#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Geometry/MathDefs.hh>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/viewer.h>

#include <list>
#include <map>
#include <set>
#include <memory>

#include <algorithm>
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
#include <execution>
#endif

namespace c2m
{
using std::list;
using std::map;
using std::set;
using std::vector;

bool MeshBuilder::build(vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    int meshes = 0;

    std::vector<Part::Ptr> sortedParts(parts);
    std::stable_sort(sortedParts.begin(), sortedParts.end(), [](const Part::Ptr& a, const Part::Ptr& b) -> bool {
        return a->elements2D.size() + a->surfaceElements.size() > b->elements2D.size() + b->surfaceElements.size();
    });

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
    static std::mutex mut;
#endif

    auto buildPart = [&](Part::Ptr& partptr) {
        std::set<Triangle, Triangle::Less> uniqueTriangles;
        map<C2MEdge, vector<size_t>, C2MEdge::Less> edgeTriangleIndices;
        triangulateAll(partptr, uniqueTriangles, deleteMeshedElements);

        vector<Triangle> allTriangles(uniqueTriangles.begin(), uniqueTriangles.end());

        for (uint triangleIndex = 0; triangleIndex < allTriangles.size(); triangleIndex++)
        {
            const Triangle& f = allTriangles[triangleIndex];
            for (uint edgeNumber = 0; edgeNumber < f.edges.size(); edgeNumber++)
            {
                edgeTriangleIndices[f.edges[edgeNumber]].emplace_back(triangleIndex);
            }
        }

        vector<vector<size_t>> submeshes;
        for (uint i = 0; i < allTriangles.size(); i++)
        {
            if (!allTriangles[i].mark)
            {
                floodFlip(i, allTriangles, edgeTriangleIndices, submeshes);
            }
        }


        CMesh& mesh = partptr->mesh;
        int deltaValid = 0;
        int deltaInvalid = 0;
        assembleMeshFromTriangles(mesh, allTriangles, submeshes, deltaValid, deltaInvalid);
        despike(mesh);

#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mut.lock();
#endif
        validTriangles += deltaValid;
        invalidTriangles += deltaInvalid;
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
        mut.unlock();
#endif

        meshes++;
    };

    std::cerr.setstate(std::ios_base::failbit);
#if defined(C2M_PARALLEL) && defined(__cpp_lib_parallel_algorithm)
    std::for_each(std::execution::par_unseq, parts.begin(), parts.end(), buildPart);
#else
    for (Part::Ptr& partptr : sortedParts)
        buildPart(partptr);
#endif
    std::cerr.clear();

    Logger::lout(Logger::INFO) << "Non-manifold triangles: " << invalidTriangles
                               << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built " << meshes << " meshes from " << parts.size()
                               << " parts, rest didnt contain faces" << std::endl;

    return true;
}

Scene::Ptr MeshBuilder::merge(std::vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    Scene::Ptr scene = std::make_shared<Scene>(parts);
    CMesh& smesh(scene->mesh);
    map<set<Node::Ptr>, FHandle> nodes2face;

    std::vector<Part::Ptr> sortedParts(parts);
    std::stable_sort(sortedParts.begin(), sortedParts.end(), [](const Part::Ptr& a, const Part::Ptr& b) -> bool {
        return a->elements2D.size() + a->surfaceElements.size() > b->elements2D.size() + b->surfaceElements.size();
    });

    for (Part::Ptr& partptr: sortedParts)
    {
        CMesh& pmesh(partptr->mesh);

        map<VHandle, VHandle> old2new;
        for (VHandle v: pmesh.vertices())
        {
            VHandle vs = smesh.add_vertex(pmesh.point(v));
            old2new[v] = vs;
            smesh.data(vs).node = pmesh.data(v).node;
            smesh.data(vs).quadrics = pmesh.data(v).quadrics;
            smesh.data(vs).duplicate = VHandle();
            smesh.data(vs).mustRemove = pmesh.data(v).mustRemove;
            smesh.data(vs).fixed = false;
        }

        for (FHandle f: pmesh.faces())
        {
            set<Node::Ptr> nodes;
            for (VHandle v: pmesh.fv_range(f))
                nodes.insert(pmesh.data(v).node);
            auto it = nodes2face.find(nodes);
            if (it != nodes2face.end())
            {
                std::list<Element2D::Ptr>& addElems = smesh.data(it->second).additionalElements;
                addElems.emplace_back(pmesh.data(f).element);
                if (deleteMeshedElements)
                {
                    addElems.splice(addElems.end(), pmesh.data(f).additionalElements);
                }
                else
                {
                    addElems.insert(addElems.end(), pmesh.data(f).additionalElements.begin(), pmesh.data(f).additionalElements.end());
                }
            }
            else
            {
                std::vector<VHandle> vertices;
                for (VHandle v: pmesh.fv_range(f))
                {
                    vertices.emplace_back(old2new[v]);
                }
                FHandle fs = smesh.add_face(vertices);
                smesh.data(fs).element = pmesh.data(f).element;
                smesh.data(fs).normalCones = pmesh.data(f).normalCones;
                nodes2face[nodes] = fs;
            }
        }

        if (deleteMeshedElements)
        {
            pmesh.clear();
        }
    }

    // Delete duplicate vertices
    smesh.delete_isolated_vertices();
    smesh.garbage_collection();

    relink(smesh);

    // map<Node::Ptr, vector<VHandle>> node2duplicates;
    // for (const VHandle& v : smesh.vertices())
    // {
    //     node2duplicates[smesh.data(v).node].emplace_back(v);
    // }
    // for (const auto kv : node2duplicates)
    // {
    //     const vector<VHandle>& duplicates = kv.second;
    //     if (duplicates.size() < 2)
    //     {
    //         continue;
    //     }
    //     for (uint i = 0; i < duplicates.size(); i++)
    //     {
    //         smesh.data(duplicates[i]).duplicate = duplicates[(i + 1) % duplicates.size()];
    //         smesh.data(duplicates[i]).fixed = true;
    //     }
    //     smesh.data(duplicates.front()).fixed = false;
    // }

    return scene;
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// ------------------------------- PRIVATE HELPER METHODS ------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

static Vec3 nodes2normal(std::vector<Node::Ptr>& nodes)
{
    Vec3 p, q, r;
    p = nodes[0]->positions.row(0).transpose();
    q = nodes[1]->positions.row(0).transpose();
    r = nodes[2]->positions.row(0).transpose();

    Vec3 normal = ((q-p).cross(r-p));
    float norm = normal.norm();
    return norm < FLT_MIN ? normal * 0.0f : normal * (1.0f / norm);
}

void MeshBuilder::floodFlip(size_t seedIndex,
                            vector<Triangle>& allTriangles,
                            map<C2MEdge, vector<size_t>, C2MEdge::Less>& edgeTriangleIndices,
                            vector<vector<size_t>>& submeshes)
{
    if (allTriangles[seedIndex].mark)
        return;

    std::list<size_t> triangleIndices;
    size_t nextSeedIndex = seedIndex;

    while (allTriangles[nextSeedIndex].mark == false)
    {
        vector<size_t>& sortedTriangles = submeshes.emplace_back(vector<size_t>());
        allTriangles[nextSeedIndex].mark = true;
        triangleIndices.emplace_back(nextSeedIndex);

        while (!triangleIndices.empty())
        {
            size_t triangleIndex = triangleIndices.front();
            triangleIndices.pop_front();
            sortedTriangles.emplace_back(triangleIndex);
            std::vector<Node::Ptr> vertices1;
            std::vector<Node::Ptr> vertices2;
            for (C2MEdge& e : allTriangles[triangleIndex].edges)
            {
                vector<size_t> linkedTriangles = edgeTriangleIndices[e];
                for (size_t f2i : linkedTriangles)
                {
                    if (f2i != triangleIndex && !allTriangles[f2i].mark)
                    {
                        if (linkedTriangles.size() > 2)
                        {
                            if (allTriangles[nextSeedIndex].mark == false)
                                continue;
                            if (vertices1.size() == 0)
                                for (C2MEdge& e1 : allTriangles[triangleIndex].edges)
                                    vertices1.emplace_back(e1.from);
                            vertices2.clear();
                            for (C2MEdge& e2 : allTriangles[f2i].edges)
                                vertices2.emplace_back(e2.from);
                            Vec3 normal1 = nodes2normal(vertices1);
                            Vec3 normal2 = nodes2normal(vertices2);

                            if (abs(normal1.transpose() * normal2) < 0.866) // angle > 30°
                                continue;
                        }

                        auto e2it = std::find(allTriangles[f2i].edges.begin(), allTriangles[f2i].edges.end(), e);
                        if (e2it != allTriangles[f2i].edges.end() && e2it->from->ID == e.from->ID)
                        {
                            for (C2MEdge& e2 : allTriangles[f2i].edges)
                            {
                                std::swap(e2.from, e2.to);
                            }
                            std::swap(allTriangles[f2i].edges.front(), allTriangles[f2i].edges.back());
                        }
                        if (linkedTriangles.size() > 2)
                        {
                            if (allTriangles[nextSeedIndex].mark == false)
                                nextSeedIndex = f2i;
                        }
                        else
                        {
                            triangleIndices.emplace_back(f2i);
                            allTriangles[f2i].mark = true;
                        }
                    }
                }
            }
        }
    }
}

size_t MeshBuilder::triangulate(const Element2D::Ptr& elem,
                                set<Triangle, Triangle::Less>& allTriangles)
{
    vector<vector<Node::Ptr>> triangles;
    if (elem->nodes.size() < 3)
    {
        throw std::logic_error("Non-meshable element encountered (< 3 nodes)");
    }
    else
    {
        // Triangle fan. If there are many higher-vertex-count polygons, this should be optimized.
        const size_t indexNode0 = 0;
        size_t indexNode1 = 1;
        size_t indexNode2 = 2;
        while (indexNode2 < elem->nodes.size())
            triangles.emplace_back(
                vector<Node::Ptr>({elem->nodes[indexNode0], elem->nodes[indexNode1++], elem->nodes[indexNode2++]}));
    }

    size_t numAdded = 0;
    for (const vector<Node::Ptr>& triangle : triangles)
    {
        Triangle f;
        f.elem = elem;
        for (uint i = 0; i < triangle.size(); i++)
        {
            C2MEdge e(triangle[i], triangle[(i + 1) % triangle.size()]);
            f.edges.emplace_back(e);
        }

        // Only add new unique triangles
        if (allTriangles.insert(f).second)
        {
            numAdded++;
        }
    }

    return numAdded;
}

size_t MeshBuilder::triangulateAll(Part::Ptr& partptr,
                                   set<Triangle, Triangle::Less>& allTriangles,
                                   bool deleteMeshedElements)
{
    if (partptr->elements2D.empty() && partptr->surfaceElements.empty())
        return 0;

    size_t numTriangles = 0;
    for (const Element2D::Ptr& elem : partptr->elements2D)
    {
        numTriangles += triangulate(elem, allTriangles);
    }
    for (const SurfaceElement::Ptr& elem : partptr->surfaceElements)
    {
        numTriangles += triangulate(elem, allTriangles);
    }

    if (deleteMeshedElements)
    {
        partptr->elements2D.clear();
        partptr->surfaceElements.clear();
    }
    return numTriangles;
}

VHandle MeshBuilder::getDuplicate(VHandle& v, map<VHandle, VHandle>& orig2dupe, CMesh& mesh)
{
    auto it = orig2dupe.find(v);
    if (orig2dupe.find(v) == orig2dupe.end())
    {
        CMesh::Point pt = mesh.point(v);
        VHandle duplicate = mesh.add_vertex(pt);
        mesh.data(duplicate).node = mesh.data(v).node;
        mesh.data(duplicate).quadrics = mesh.data(v).quadrics;
        mesh.data(duplicate).duplicate = mesh.data(v).duplicate;
        mesh.data(duplicate).mustRemove = mesh.data(v).mustRemove;
        mesh.data(duplicate).fixed = true;
        mesh.status(duplicate).set_fixed_nonmanifold(true);
        orig2dupe[v] = duplicate;
        return duplicate;
    }
    else
    {
        return it->second;
    }
}

void MeshBuilder::assembleMeshFromTriangles(CMesh& mesh,
                                            const vector<Triangle>& allTriangles,
                                            const vector<vector<size_t>>& submeshes,
                                            int& validTris,
                                            int& invalidTris)
{
    mesh.clear();
    for (const std::vector<size_t>& triangleOrdering: submeshes)
    {
        map<nodeid_t, VHandle> nodeToVertex;
        map<VHandle, VHandle> orig2dupe;
        map<set<Node::Ptr>, FHandle> nodes2face;

        for (size_t fi = 0; fi < triangleOrdering.size(); fi++)
        {
            const Triangle& f = allTriangles[triangleOrdering[fi]];
            set<Node::Ptr> nodes;
            for (uint i = 0; i < 3; i++)
            {
                nodes.insert(f.edges[i].from);
            }
            auto it = nodes2face.find(nodes);
            if (it != nodes2face.end())
            {
                FHandle fh = it->second;
                mesh.data(fh).additionalElements.emplace_back(f.elem);
            }
            vector<VHandle> vertices;
            for (uint i = 0; i < 3; i++)
            {
                const Node::Ptr& nodeptr = f.edges[i].from;
                if (nodeToVertex.find(nodeptr->ID) == nodeToVertex.end())
                {
                    VHandle v = mesh.add_vertex(OMVec3(
                        nodeptr->positions.coeff(0, 0), nodeptr->positions.coeff(0, 1), nodeptr->positions.coeff(0, 2)));
                    nodeToVertex[nodeptr->ID] = v;
                    mesh.data(v).node = nodeptr;
                    mesh.data(v).fixed = false;
                    mesh.data(v).mustRemove = false;
                    mesh.data(v).duplicate = VHandle();
                    vertices.emplace_back(v);
                }
                else
                {
                    vertices.emplace_back(nodeToVertex[nodeptr->ID]);
                }
            }

            CMesh::FaceHandle fh = mesh.add_face(vertices);
            if (fh.is_valid())
            {
                mesh.data(fh).element = f.elem;
                nodes2face[nodes] = fh;
                validTris++;
            }
            else
            {
                invalidTris++;
                // Duplicate one vertex at a time until face is valid
                bool fixed = false;
                vector<VHandle> prevVertices = vertices;
                const vector<vector<bool>> permutations({{true, false, false},
                                                            {false, true, false},
                                                            {false, false, true},
                                                            {true, true, false},
                                                            {true, false, true},
                                                            {false, true, true},
                                                            {true, true, true}});
                while (!fixed)
                {
                    vector<VHandle> dupeTri(3);
                    for (const vector<bool>& dupePermutation : permutations)
                    {
                        for (int vi = 0; vi < 3; vi++)
                        {
                            dupeTri[vi]
                                = dupePermutation[vi] ? getDuplicate(prevVertices[vi], orig2dupe, mesh) : vertices[vi];
                        }
                        fh = mesh.add_face(dupeTri);
                        if (fh.is_valid())
                        {
                            nodes2face[nodes] = fh;
                            mesh.data(fh).element = f.elem;
                            mesh.status(fh).set_fixed_nonmanifold(true);
                            for (int vi = 0; vi < 3; vi++)
                            {
                                if (dupePermutation[vi])
                                {
                                    mesh.data(vertices[vi]).fixed = true;
                                    mesh.status(vertices[vi]).set_fixed_nonmanifold(true);
                                }
                            }
                            fixed = true;
                            break;
                        }
                    }
                    if (!fixed)
                    {
                        prevVertices = dupeTri; // All three duplicates
                    }
                }
            }
        }
    }

    // Delete duplicate vertices
    mesh.delete_isolated_vertices();
    mesh.garbage_collection();

    relink(mesh);
}

void MeshBuilder::despike(CMesh& mesh)
{
    // TODO do this properly
}


void MeshBuilder::relink(CMesh& mesh)
{
    map<Node::Ptr, vector<VHandle>> node2duplicates;
    for (const VHandle& v : mesh.vertices())
    {
        // if (mesh.data(v).fixed || mesh.data(v).node->referencingParts > 1 ||  mesh.data(v).duplicate.is_valid())
        // {
            node2duplicates[mesh.data(v).node].emplace_back(v);
        // }
    }
    for (const auto kv : node2duplicates)
    {
        const vector<VHandle>& duplicates = kv.second;
        if (duplicates.size() == 1)
        {
            mesh.data(duplicates.front()).duplicate = VHandle();
            mesh.data(duplicates.front()).fixed = false;
            continue;
        }
        for (uint i = 0; i < duplicates.size(); i++)
        {
            mesh.data(duplicates[i]).duplicate = duplicates[(i + 1) % duplicates.size()];
            mesh.data(duplicates[i]).fixed = true;
        }
        mesh.data(duplicates.front()).fixed = false;
    }
#if 1
    for (VHandle v : mesh.vertices())
    {
        VHandle dupe = mesh.data(v).duplicate;
        if (dupe.is_valid())
        {
            if (dupe == v)
            {
                throw std::logic_error("dupe pointing to itself impossible");
            }
            while (dupe != v)
            {
                dupe = mesh.data(dupe).duplicate;
                if (!dupe.is_valid())
                {
                    throw std::logic_error("cyclic chain error :/");
                }
            }
        }
    }
#endif
}

// First implementation worked really poorly, and no time so far to improve it. Alternative exists.
#if 0
vector<list<C2MEdge>> MeshBuilder::nonManifoldPaths(map<C2MEdge, vector<size_t>, C2MEdge::Less>& edgeTriangleIndices)
{
    vector<list<C2MEdge>> nMfPaths;
    list<C2MEdge> nMfC2MEdges;
    for (auto kv : edgeTriangleIndices)
        if (kv.second.size() > 2)
            nMfC2MEdges.emplace_back(kv.first);

    while (!nMfC2MEdges.empty())
    {
        C2MEdge e1 = nMfC2MEdges.front();
        nMfC2MEdges.pop_front();
        list<C2MEdge>& path = nMfPaths.emplace_back(list<C2MEdge>({e1}));
        Node::Ptr frontNode = e1.from;
        Node::Ptr backNode = e1.to;
        bool reset;
        for (auto it = nMfC2MEdges.begin(); it != nMfC2MEdges.end();)
        {
            C2MEdge e2 = *it;
            reset = false;
            if (e2.from == frontNode || e2.to == frontNode || e2.from == backNode || e2.to == backNode)
            {
                reset = true;
                nMfC2MEdges.erase(it);
                if (e2.from == frontNode)
                    frontNode = path.emplace_front(e2).to;
                else if (e2.to == frontNode)
                    frontNode = path.emplace_front(e2).from;
                else if (e2.from == backNode)
                    backNode = path.emplace_back(e2).to;
                else if (e2.to == backNode)
                    backNode = path.emplace_back(e2).from;
            }

            if (reset)
                it = nMfC2MEdges.begin();
            else
                it++;
        }
    }

    return nMfPaths;
}

void MeshBuilder::sortFromNonManifoldPaths(vector<list<C2MEdge>>& nMfPaths,
                                     vector<Triangle>& allTriangles,
                                     map<C2MEdge, vector<size_t>, C2MEdge::Less>& edgeTriangleIndices)
{
    vector<Triangle> sortedTriangles;
    vector<bool> inserted(allTriangles.size(), false);
    for (list<C2MEdge>& nMfPath : nMfPaths)
    {
        for (const C2MEdge& e: nMfPath)
        {
            vector<size_t>& triangleIndices = edgeTriangleIndices[e];
            assert(triangleIndices.size() > 2);
            vector<OMVec3> normals;
            for (size_t triangleIndex : triangleIndices)
            {
                Vec3 p0 = allTriangles[triangleIndex].edges[0].from->positions.row(0);
                Vec3 p1 = allTriangles[triangleIndex].edges[1].from->positions.row(0);
                Vec3 p2 = allTriangles[triangleIndex].edges[2].from->positions.row(0);
                OMVec3 op0(p0[0], p0[1], p0[2]);
                OMVec3 op1(p1[0], p1[1], p1[2]);
                OMVec3 op2(p2[0], p2[1], p2[2]);
                normals.emplace_back(((op1 - op0) % (op2 - op0)).normalize());
            }
            bool foundPair = false;
            for (uint i = 0; i < triangleIndices.size() && !foundPair; i++)
            {
                for (uint j = i; j < triangleIndices.size() && !foundPair; j++)
                {
                    OMVec3& normal1 = normals[i], normal2 = normals[j];
                    if (OpenMesh::rad_to_deg(acos(normal1 | normal2)) < 30)
                    {
                        foundPair = true;
                        if (!inserted[triangleIndices[i]])
                        {
                            inserted[triangleIndices[i]] = true;
                            sortedTriangles.emplace_back(allTriangles[triangleIndices[i]]);
                        }
                        if (!inserted[triangleIndices[j]])
                        {
                            inserted[triangleIndices[j]] = true;
                            sortedTriangles.emplace_back(allTriangles[triangleIndices[j]]);
                        }
                    }
                }
            }
        }
    }
    for (uint i = 0; i < allTriangles.size(); i++)
        if (!inserted[i])
            sortedTriangles.emplace_back(allTriangles[i]);
    assert(allTriangles.size() == sortedTriangles.size());
    allTriangles = sortedTriangles;
}
#endif

} // namespace c2m
