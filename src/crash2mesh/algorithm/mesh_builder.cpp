#include <crash2mesh/algorithm/mesh_builder.hpp>

#include <crash2mesh/util/logger.hpp>

#include <OpenMesh/Core/Geometry/MathDefs.hh>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/viewer.h>

#include <list>
#include <map>
#include <memory>

namespace c2m
{
using std::list;
using std::map;
using std::set;
using std::vector;

struct Edge
{
    using Ptr = std::shared_ptr<Edge>;
    Edge(Node::Ptr _from, Node::Ptr _to) : from(_from), to(_to)
    {
    }

    Node::Ptr from;
    Node::Ptr to;

    bool operator==(const Edge& other)
    {
        return (from->ID == other.from->ID && to->ID == other.to->ID)
               || (from->ID == other.to->ID && to->ID == other.from->ID);
    }

    struct CmpLess
    {
        bool operator()(const Edge& lhs, const Edge& rhs) const
        {
            nodeid_t lhsfromID = lhs.from->ID;
            nodeid_t lhstoID = lhs.to->ID;
            nodeid_t rhsfromID = rhs.from->ID;
            nodeid_t rhstoID = rhs.to->ID;
            if (lhstoID < lhsfromID)
            {
                std::swap(lhsfromID, lhstoID);
            }
            if (rhstoID < rhsfromID)
            {
                std::swap(rhstoID, rhsfromID);
            }
            return (lhsfromID < rhsfromID) || ((lhsfromID == rhsfromID) && (lhstoID < rhstoID));
        }
    };
};

class Triangle
{
  public:
    Triangle() : mark(false), edges(), elem()
    {
    }
    bool mark;
    vector<Edge> edges;
    Element2D::Ptr elem;
};

static void floodFlip(size_t fi,
                      vector<Triangle>& allTriangles,
                      map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices,
                      vector<size_t>& sortedTriangles)
{
    if (allTriangles[fi].mark)
        return;

    allTriangles[fi].mark = true;
    std::list<size_t> triangleIndices({fi});

    while (!triangleIndices.empty())
    {
        size_t triangleIndex = triangleIndices.front();
        triangleIndices.pop_front();
        sortedTriangles.emplace_back(triangleIndex);
        for (Edge& e : allTriangles[triangleIndex].edges)
        {
            vector<size_t> linkedTriangles = edgeTriangleIndices[e];
            if (linkedTriangles.size() > 2)
            {
                continue;
            }
            for (size_t f2i : linkedTriangles)
            {
                if (f2i != triangleIndex && !allTriangles[f2i].mark)
                {
                    auto e2it = std::find(allTriangles[f2i].edges.begin(), allTriangles[f2i].edges.end(), e);
                    if (e2it != allTriangles[f2i].edges.end() && e2it->from->ID == e.from->ID)
                    {
                        for (Edge& e2 : allTriangles[f2i].edges)
                        {
                            std::swap(e2.from, e2.to);
                        }
                        std::reverse(allTriangles[f2i].edges.begin(), allTriangles[f2i].edges.end());
                    }
                    triangleIndices.emplace_back(f2i);
                    allTriangles[f2i].mark = true;
                }
            }
        }
    }
}

static size_t triangulate(const Element2D::Ptr& elem,
                        vector<Triangle>& allTriangles,
                        map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices)
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

    for (const vector<Node::Ptr>& triangle : triangles)
    {
        Triangle& f = allTriangles.emplace_back(Triangle());
        f.elem = elem;
        for (uint i = 0; i < triangle.size(); i++)
        {
            Edge e(triangle[i], triangle[(i + 1) % triangle.size()]);
            f.edges.emplace_back(e);
            edgeTriangleIndices[e].emplace_back(allTriangles.size() - 1);
        }
    }

    return triangles.size();
}

static size_t triangulateAll(Part::Ptr& partptr,
                           vector<Triangle>& allTriangles,
                           map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices,
                           bool deleteMeshedElements)
{
    if (partptr->elements2D.empty() && partptr->surfaceElements.empty())
        return 0;

    size_t numTriangles = 0;
    for (const Element2D::Ptr& elem : partptr->elements2D)
    {
        numTriangles += triangulate(elem, allTriangles, edgeTriangleIndices);
    }
    for (const SurfaceElement::Ptr& elem : partptr->surfaceElements)
    {
        numTriangles += triangulate(elem, allTriangles, edgeTriangleIndices);
    }

    if (deleteMeshedElements)
    {
        partptr->elements2D.clear();
        partptr->surfaceElements.clear();
    }
    return numTriangles;
}

static VHandle getDuplicate(VHandle& v, map<VHandle, VHandle>& orig2dupe, CMesh& mesh)
{
    auto it = orig2dupe.find(v);
    if (orig2dupe.find(v) == orig2dupe.end())
    {
        CMesh::Point pt = mesh.point(v);
        VHandle duplicate = mesh.add_vertex(pt);
        mesh.data(duplicate) = mesh.data(v);
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

static void assembleMeshFromTriangles(CMesh& mesh, const vector<Triangle>& allTriangles, const vector<size_t>& triangleOrdering, int& validTris, int& invalidTris)
{
        mesh.clear();
        map<nodeid_t, VHandle> nodeToVertex;
        map<VHandle, VHandle> orig2dupe;

        for (size_t fi = 0; fi < triangleOrdering.size(); fi++)
        {
            const Triangle& f = allTriangles[triangleOrdering[fi]];
            vector<VHandle> vertices;
            for (uint i = 0; i < 3; i++)
            {
                const Node::Ptr& nodeptr = f.edges[i].from;
                if (nodeToVertex.find(nodeptr->ID) == nodeToVertex.end())
                {
                    VHandle v = mesh.add_vertex(OMVec3(nodeptr->positions.coeff(0, 0),
                                                       nodeptr->positions.coeff(0, 1),
                                                       nodeptr->positions.coeff(0, 2)));
                    nodeToVertex[nodeptr->ID] = v;
                    mesh.data(v).node = nodeptr;
                    mesh.data(v).fixed = false;
                    mesh.data(v).duplicate = VHandle();
                    vertices.emplace_back(v);
                }
                else
                {
                    vertices.emplace_back(nodeToVertex[nodeptr->ID]);
                }
            }
            std::cerr.setstate(std::ios_base::failbit);
            CMesh::FaceHandle fh = mesh.add_face(vertices);
            if (fh.is_valid())
            {
                mesh.data(fh).element = f.elem;
                validTris++;
            }
            else
            {
                invalidTris++;
                // Duplicate one vertex at a time until face is valid
                bool fixed = false;
                vector<VHandle> prevVertices = vertices;
                while (!fixed)
                {
                    const vector<vector<bool>> permutations({{true, false, false},
                                                             {false, true, false},
                                                             {false, false, true},
                                                             {true, true, false},
                                                             {true, false, true},
                                                             {false, true, true},
                                                             {true, true, true}});
                    vector<VHandle> dupeTri(3);
                    for (const vector<bool>& dupePermutation : permutations)
                    {
                        for (int vi = 0; vi < 3; vi++)
                        {
                            dupeTri[vi] = dupePermutation[vi] ? getDuplicate(prevVertices[vi], orig2dupe, mesh) : vertices[vi];
                        }
                        fh = mesh.add_face(dupeTri);
                        if (fh.is_valid())
                        {
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
            std::cerr.clear();
        }

        // Delete duplicate vertices
        mesh.delete_isolated_vertices();
        mesh.garbage_collection();

        map<Node::Ptr, vector<VHandle>> node2duplicates;
        for (const VHandle& v : mesh.vertices())
        {
            if (mesh.data(v).fixed)
            {
                node2duplicates[mesh.data(v).node].emplace_back(v);
            }
        }
        for (const auto kv : node2duplicates)
        {
            const vector<VHandle>& duplicates = kv.second;
            if (duplicates.size() < 2)
            {
                throw std::logic_error("Incomplete duplicate chain!");
            }
            for (uint i = 0; i < duplicates.size(); i++)
            {
                mesh.data(duplicates[i]).duplicate = duplicates[(i + 1) % duplicates.size()];
            }
            mesh.data(duplicates.front()).fixed = false;
        }
#ifndef NDEBUG
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

// static vector<list<Edge>> nonManifoldPaths(map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices)
// {
//     vector<list<Edge>> nMfPaths;
//     list<Edge> nMfEdges;
//     for (auto kv : edgeTriangleIndices)
//         if (kv.second.size() > 2)
//             nMfEdges.emplace_back(kv.first);

//     while (!nMfEdges.empty())
//     {
//         Edge e1 = nMfEdges.front();
//         nMfEdges.pop_front();
//         list<Edge>& path = nMfPaths.emplace_back(list<Edge>({e1}));
//         Node::Ptr frontNode = e1.from;
//         Node::Ptr backNode = e1.to;
//         bool reset;
//         for (auto it = nMfEdges.begin(); it != nMfEdges.end();)
//         {
//             Edge e2 = *it;
//             reset = false;
//             if (e2.from == frontNode || e2.to == frontNode || e2.from == backNode || e2.to == backNode)
//             {
//                 reset = true;
//                 nMfEdges.erase(it);
//                 if (e2.from == frontNode)
//                     frontNode = path.emplace_front(e2).to;
//                 else if (e2.to == frontNode)
//                     frontNode = path.emplace_front(e2).from;
//                 else if (e2.from == backNode)
//                     backNode = path.emplace_back(e2).to;
//                 else if (e2.to == backNode)
//                     backNode = path.emplace_back(e2).from;
//             }

//             if (reset)
//                 it = nMfEdges.begin();
//             else
//                 it++;
//         }
//     }

//     return nMfPaths;
// }

// static void sortFromNonManifoldPaths(vector<list<Edge>>& nMfPaths,
//                                      vector<Triangle>& allTriangles,
//                                      map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices)
// {
//     vector<Triangle> sortedTriangles;
//     vector<bool> inserted(allTriangles.size(), false);
//     for (list<Edge>& nMfPath : nMfPaths)
//     {
//         for (const Edge& e: nMfPath)
//         {
//             vector<size_t>& triangleIndices = edgeTriangleIndices[e];
//             assert(triangleIndices.size() > 2);
//             vector<OMVec3> normals;
//             for (size_t triangleIndex : triangleIndices)
//             {
//                 Vec3 p0 = allTriangles[triangleIndex].edges[0].from->positions.row(0);
//                 Vec3 p1 = allTriangles[triangleIndex].edges[1].from->positions.row(0);
//                 Vec3 p2 = allTriangles[triangleIndex].edges[2].from->positions.row(0);
//                 OMVec3 op0(p0[0], p0[1], p0[2]);
//                 OMVec3 op1(p1[0], p1[1], p1[2]);
//                 OMVec3 op2(p2[0], p2[1], p2[2]);
//                 normals.emplace_back(((op1 - op0) % (op2 - op0)).normalize());
//             }
//             bool foundPair = false;
//             for (uint i = 0; i < triangleIndices.size() && !foundPair; i++)
//             {
//                 for (uint j = i; j < triangleIndices.size() && !foundPair; j++)
//                 {
//                     OMVec3& normal1 = normals[i], normal2 = normals[j];
//                     if (OpenMesh::rad_to_deg(acos(normal1 | normal2)) < 30)
//                     {
//                         foundPair = true;
//                         if (!inserted[triangleIndices[i]])
//                         {
//                             inserted[triangleIndices[i]] = true;
//                             sortedTriangles.emplace_back(allTriangles[triangleIndices[i]]);
//                         }
//                         if (!inserted[triangleIndices[j]])
//                         {
//                             inserted[triangleIndices[j]] = true;
//                             sortedTriangles.emplace_back(allTriangles[triangleIndices[j]]);
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     for (uint i = 0; i < allTriangles.size(); i++)
//         if (!inserted[i])
//             sortedTriangles.emplace_back(allTriangles[i]);
//     assert(allTriangles.size() == sortedTriangles.size());
//     allTriangles = sortedTriangles;
// }

bool MeshBuilder::build(vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    int meshes = 0;
    for (Part::Ptr& partptr : parts)
    {
        vector<Triangle> allTriangles;
        map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
        triangulateAll(partptr, allTriangles, edgeTriangleIndices, deleteMeshedElements);

        vector<size_t> sortedTriangles;
        for (uint i = 0; i < allTriangles.size(); i++)
        {
            if (!allTriangles[i].mark)
            {
                floodFlip(i, allTriangles, edgeTriangleIndices, sortedTriangles);
            }
        }

        // vector<list<Edge>> nMfPaths(nonManifoldPaths(edgeTriangleIndices));
        // sortFromNonManifoldPaths(nMfPaths, allTriangles, edgeTriangleIndices);

        CMesh& mesh = partptr->mesh;

        assembleMeshFromTriangles(mesh, allTriangles, sortedTriangles, validTriangles, invalidTriangles);

        meshes++;
    }

    Logger::lout(Logger:INFO) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built " << meshes << " meshes from " << parts.size()
                               << " parts, rest didnt contain faces" << std::endl;

    return true;
}

CMesh MeshBuilder::buildSingle(vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    vector<Triangle> allTriangles;
    map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
    vector<size_t> sortedTriangles;
    uint ffIndex = 0;
    for (Part::Ptr& partptr : parts)
    {
        triangulateAll(partptr, allTriangles, edgeTriangleIndices, deleteMeshedElements);

        for (; ffIndex < allTriangles.size(); ffIndex++)
        {
            if (!allTriangles[ffIndex].mark)
            {
                floodFlip(ffIndex, allTriangles, edgeTriangleIndices, sortedTriangles);
            }
        }
    }

    vector<size_t> sortedTrianglesTrash;
    for (uint i = 0; i < allTriangles.size(); i++)
    {
        allTriangles[i].mark = false;
    }
    for (uint i = 0; i < allTriangles.size(); i++)
    {
        if (!allTriangles[i].mark)
        {
            floodFlip(i, allTriangles, edgeTriangleIndices, sortedTrianglesTrash);
        }
    }

    // vector<list<Edge>> nMfPaths(nonManifoldPaths(edgeTriangleIndices));
    // sortFromNonManifoldPaths(nMfPaths, allTriangles, edgeTriangleIndices);

    CMesh mesh;
    assembleMeshFromTriangles(mesh, allTriangles, sortedTriangles, validTriangles, invalidTriangles);

    Logger::lout(Logger::INFO) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built a single mesh from " << parts.size() << " parts" << std::endl;

    return mesh;
}

Scene::Ptr MeshBuilder::merge(std::vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    vector<Triangle> allTriangles;
    map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
    vector<size_t> sortedTriangles;
    uint ffIndex = 0;
    for (Part::Ptr& partptr : parts)
    {
        vector<vector<Node::Ptr>> triangles;
        vector<Element2D::Ptr> elements;
        for (FHandle f : partptr->mesh.faces())
        {
            triangles.emplace_back(vector<Node::Ptr>());
            elements.emplace_back(partptr->mesh.data(f).element);
            for (VHandle v : partptr->mesh.fv_range(f))
            {
                triangles.back().emplace_back(partptr->mesh.data(v).node);
            }
        }
        for (uint triangle = 0; triangle < triangles.size(); triangle++)
        {
            Triangle& f = allTriangles.emplace_back(Triangle());
            f.elem = elements[triangle];
            for (uint vertex = 0; vertex < 3; vertex++)
            {
                Edge e(triangles[triangle][vertex], triangles[triangle][(vertex + 1) % 3]);
                f.edges.emplace_back(e);
                edgeTriangleIndices[e].emplace_back(allTriangles.size() - 1);
            }
        }

        for (; ffIndex < allTriangles.size(); ffIndex++)
        {
            if (!allTriangles[ffIndex].mark)
            {
                floodFlip(ffIndex, allTriangles, edgeTriangleIndices, sortedTriangles);
            }
        }
        if (deleteMeshedElements)
            partptr->mesh.clear();
    }

    vector<size_t> sortedTrianglesTrash;
    for (uint i = 0; i < allTriangles.size(); i++)
    {
        allTriangles[i].mark = false;
    }
    for (uint i = 0; i < allTriangles.size(); i++)
    {
        if (!allTriangles[i].mark)
        {
            floodFlip(i, allTriangles, edgeTriangleIndices, sortedTrianglesTrash);
        }
    }

    // vector<list<Edge>> nMfPaths(nonManifoldPaths(edgeTriangleIndices));
    // sortFromNonManifoldPaths(nMfPaths, allTriangles, edgeTriangleIndices);

    Scene::Ptr scene = std::make_shared<Scene>(parts);
    assembleMeshFromTriangles(scene->mesh, allTriangles, sortedTriangles, validTriangles, invalidTriangles);

    Logger::lout(Logger::INFO) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built a single mesh from " << parts.size() << " parts" << std::endl;

    return scene;
}

} // namespace c2m
