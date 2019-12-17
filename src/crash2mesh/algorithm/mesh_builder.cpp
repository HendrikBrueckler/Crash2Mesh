#include <crash2mesh/algorithm/mesh_builder.hpp>

#include <crash2mesh/util/logger.hpp>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable.h>
#include <easy3d/viewer/viewer.h>

#include <map>
#include <memory>

namespace c2m
{
using std::map;
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

static void
floodFlip(size_t fi, vector<Triangle>& allTriangles, std::map<Edge, vector<size_t>, Edge::CmpLess>& edgeTriangleIndices)
{
    if (allTriangles[fi].mark)
        return;

    allTriangles[fi].mark = true;
    for (Edge& e : allTriangles[fi].edges)
    {
        vector<size_t> linkedTriangles = edgeTriangleIndices[e];
        if (linkedTriangles.size() > 2)
        {
            // // Logger::lout(Logger::DEBUG) << "Edge shared between " << linkedTriangles.size()
            // //                            << " faces, can't properly handle this geometry" << std::endl;
            // for (size_t f2i : linkedTriangles)
            // {
            //     allTriangles[f2i].mark = true;
            // }
            continue;
        }
        for (size_t f2i : linkedTriangles)
        {
            if (f2i != fi)
            {
                auto e2it = std::find(allTriangles[f2i].edges.begin(), allTriangles[f2i].edges.end(), e);
                if (e2it != allTriangles[f2i].edges.end() && e2it->from->ID == e.from->ID && !allTriangles[f2i].mark)
                {
                    for (Edge& e2 : allTriangles[f2i].edges)
                    {
                        std::swap(e2.from, e2.to);
                    }
                    std::reverse(allTriangles[f2i].edges.begin(), allTriangles[f2i].edges.end());
                    floodFlip(f2i, allTriangles, edgeTriangleIndices);
                }
            }
        }
    }
}

bool MeshBuilder::build(std::vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    int meshes = 0;
    for (Part::Ptr& partptr : parts)
    {
        if (partptr->elements2D.empty() && partptr->surfaceElements.empty())
        {
            continue;
        }

        // Try flipping badly oriented faces
        // Create faces for parts
        std::vector<Triangle> allTriangles;
        std::map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
        auto addTriangles = [&](const Element2D::Ptr& elem) {
            vector<vector<Node::Ptr>> triangles;
            if (elem->nodes.size() == 3)
            {
                triangles = {{elem->nodes[0], elem->nodes[1], elem->nodes[2]}};
            }
            else if (elem->nodes.size() == 4)
            {
                triangles = {{elem->nodes[0], elem->nodes[1], elem->nodes[2]},
                             {elem->nodes[2], elem->nodes[3], elem->nodes[0]}};
            }
            else
            {
                throw std::logic_error("Can only handle triangles and quads (currently)");
            }

            for (const vector<Node::Ptr>& triangle : triangles)
            {
                Triangle& f = allTriangles.emplace_back(Triangle());
                f.elem = elem;
                for (uint i = 0; i < triangle.size(); i++)
                {
                    Edge e(triangle[i], triangle[(i + 1) % triangle.size()]);
                    f.edges.emplace_back(e);
                    auto eit = edgeTriangleIndices.find(e);
                    if (eit == edgeTriangleIndices.end())
                    {
                        edgeTriangleIndices[e] = {allTriangles.size() - 1};
                    }
                    else
                    {
                        eit->second.emplace_back(allTriangles.size() - 1);
                    }
                }
            }
        };
        for (const Element2D::Ptr& elem : partptr->elements2D)
        {
            addTriangles(elem);
        }
        for (const SurfaceElement::Ptr& elem : partptr->surfaceElements)
        {
            addTriangles(elem);
        }

        if (deleteMeshedElements)
        {
            partptr->elements2D.clear();
            partptr->surfaceElements.clear();
        }

        for (uint i = 0; i < allTriangles.size(); i++)
        {
            if (!allTriangles[i].mark)
            {
                floodFlip(i, allTriangles, edgeTriangleIndices);
            }
        }

        CMesh& mesh = partptr->mesh;
        mesh = CMesh();
        map<nodeid_t, VHandle> nodeToVertex;
        map<VHandle, VHandle> duplicates;

        auto getDuplicate = [&duplicates, &mesh](VHandle& v) -> VHandle {
            auto it = duplicates.find(v);
            if (duplicates.find(v) == duplicates.end())
            {
                VHandle duplicate = mesh.add_vertex(mesh.point(v));
                mesh.data(duplicate) = mesh.data(v);
                mesh.data(duplicate).fixed = true;
                mesh.status(duplicate).set_fixed_nonmanifold(true);
                duplicates[v] = duplicate;
                return duplicate;
            }
            else
            {
                return it->second;
            }
        };

        for (size_t fi = 0; fi < allTriangles.size(); fi++)
        {
            const Triangle& f = allTriangles[fi];
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
                    mesh.data(v).fixed = nodeptr->referencingParts > 1;
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
                validTriangles++;
            }
            else
            {
                invalidTriangles++;
                // Duplicate one vertex
                bool fixed = false;
                vector<VHandle> prevVertices = vertices;
                while (!fixed)
                {
                    fixed = true;
                    vector<VHandle> dupeTri = {getDuplicate(prevVertices[0]), vertices[1], vertices[2]};
                    fh = mesh.add_face(dupeTri);
                    if (fh.is_valid())
                    {
                        mesh.data(prevVertices[0]).fixed = true;
                        mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                        mesh.data(fh).element = f.elem;
                        mesh.status(fh).set_fixed_nonmanifold(true);
                    }
                    else
                    {
                        dupeTri = {vertices[0], getDuplicate(prevVertices[1]), vertices[2]};
                        fh = mesh.add_face(dupeTri);
                        if (fh.is_valid())
                        {
                            mesh.data(prevVertices[1]).fixed = true;
                            mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                            mesh.data(fh).element = f.elem;
                            mesh.status(fh).set_fixed_nonmanifold(true);
                        }
                        else
                        {
                            dupeTri = {vertices[0], vertices[1], getDuplicate(prevVertices[2])};
                            fh = mesh.add_face(dupeTri);
                            if (fh.is_valid())
                            {
                                mesh.data(fh).element = f.elem;
                                mesh.data(prevVertices[2]).fixed = true;
                                mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                mesh.status(fh).set_fixed_nonmanifold(true);
                            }
                            else
                            {
                                // Duplicate two vertices
                                dupeTri = {getDuplicate(prevVertices[0]), getDuplicate(prevVertices[1]), vertices[2]};
                                fh = mesh.add_face(dupeTri);
                                if (fh.is_valid())
                                {
                                    mesh.data(fh).element = f.elem;
                                    mesh.data(prevVertices[0]).fixed = true;
                                    mesh.data(prevVertices[1]).fixed = true;
                                    mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                    mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                    mesh.status(fh).set_fixed_nonmanifold(true);
                                }
                                else
                                {
                                    dupeTri
                                        = {getDuplicate(prevVertices[0]), vertices[1], getDuplicate(prevVertices[2])};
                                    fh = mesh.add_face(dupeTri);
                                    if (fh.is_valid())
                                    {
                                        mesh.data(fh).element = f.elem;
                                        mesh.data(prevVertices[0]).fixed = true;
                                        mesh.data(prevVertices[2]).fixed = true;
                                        mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                        mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                        mesh.status(fh).set_fixed_nonmanifold(true);
                                    }
                                    else
                                    {
                                        dupeTri = {
                                            vertices[0], getDuplicate(prevVertices[1]), getDuplicate(prevVertices[2])};
                                        if (fh.is_valid())
                                        {
                                            mesh.data(fh).element = f.elem;
                                            mesh.data(prevVertices[1]).fixed = true;
                                            mesh.data(prevVertices[2]).fixed = true;
                                            mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                            mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                            mesh.status(fh).set_fixed_nonmanifold(true);
                                        }
                                        else
                                        {
                                            // Duplicate all three vertices
                                            dupeTri = {getDuplicate(prevVertices[0]),
                                                       getDuplicate(prevVertices[1]),
                                                       getDuplicate(prevVertices[2])};
                                            fh = mesh.add_face(dupeTri);
                                            if (fh.is_valid())
                                            {
                                                mesh.data(fh).element = f.elem;
                                                mesh.data(prevVertices[0]).fixed = true;
                                                mesh.data(prevVertices[1]).fixed = true;
                                                mesh.data(prevVertices[2]).fixed = true;
                                                mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                                mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                                mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                                mesh.status(fh).set_fixed_nonmanifold(true);
                                            }
                                            else
                                            {
                                                Logger::lout(Logger::WARN) << "Non-manifold element of complexity > {2 "
                                                                              "touching manifold meshes} encountered"
                                                                           << std::endl;
                                                prevVertices = dupeTri;
                                                fixed = false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            std::cerr.clear();
        }

        // Delete duplicate vertices
        mesh.delete_isolated_vertices();
        mesh.garbage_collection();

        meshes++;
    }

    Logger::lout(Logger::DEBUG) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built " << meshes << " meshes from " << parts.size()
                               << " parts, rest didnt contain faces" << std::endl;

    return true;
}

CMesh MeshBuilder::buildSingle(std::vector<Part::Ptr>& parts, bool deleteMeshedElements)
{
    int validTriangles = 0;
    int invalidTriangles = 0;
    std::vector<Triangle> allTriangles;
    std::map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
    for (Part::Ptr& partptr : parts)
    {
        if (partptr->elements2D.empty() && partptr->surfaceElements.empty())
        {
            continue;
        }

        // Try flipping badly oriented faces
        // Create faces for parts
        auto addTriangles = [&](const Element2D::Ptr& elem) {
            vector<vector<Node::Ptr>> triangles;
            if (elem->nodes.size() == 3)
            {
                triangles = {{elem->nodes[0], elem->nodes[1], elem->nodes[2]}};
            }
            else if (elem->nodes.size() == 4)
            {
                triangles = {{elem->nodes[0], elem->nodes[1], elem->nodes[2]},
                             {elem->nodes[2], elem->nodes[3], elem->nodes[0]}};
            }
            else
            {
                throw std::logic_error("Can only handle triangles and quads (currently)");
            }

            for (const vector<Node::Ptr>& triangle : triangles)
            {
                Triangle& f = allTriangles.emplace_back(Triangle());
                f.elem = elem;
                for (uint i = 0; i < triangle.size(); i++)
                {
                    Edge e(triangle[i], triangle[(i + 1) % triangle.size()]);
                    f.edges.emplace_back(e);
                    auto eit = edgeTriangleIndices.find(e);
                    if (eit == edgeTriangleIndices.end())
                    {
                        edgeTriangleIndices[e] = {allTriangles.size() - 1};
                    }
                    else
                    {
                        eit->second.emplace_back(allTriangles.size() - 1);
                    }
                }
            }
        };
        for (const Element2D::Ptr& elem : partptr->elements2D)
        {
            addTriangles(elem);
        }
        for (const SurfaceElement::Ptr& elem : partptr->surfaceElements)
        {
            addTriangles(elem);
        }

        if (deleteMeshedElements)
        {
            partptr->elements2D.clear();
            partptr->surfaceElements.clear();
        }
    }

    for (uint i = 0; i < allTriangles.size(); i++)
    {
        if (!allTriangles[i].mark)
        {
            floodFlip(i, allTriangles, edgeTriangleIndices);
        }
    }

    CMesh mesh;
    mesh = CMesh();
    map<nodeid_t, VHandle> nodeToVertex;
    map<VHandle, VHandle> duplicates;

    auto getDuplicate = [&duplicates, &mesh](VHandle& v) -> VHandle {
        auto it = duplicates.find(v);
        if (duplicates.find(v) == duplicates.end())
        {
            VHandle duplicate = mesh.add_vertex(mesh.point(v));
            mesh.data(duplicate) = mesh.data(v);
            mesh.data(duplicate).fixed = true;
            mesh.status(duplicate).set_fixed_nonmanifold(true);
            duplicates[v] = duplicate;
            return duplicate;
        }
        else
        {
            return it->second;
        }
    };

    for (size_t fi = 0; fi < allTriangles.size(); fi++)
    {
        const Triangle& f = allTriangles[fi];
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
            validTriangles++;
        }
        else
        {
            invalidTriangles++;
            // Duplicate one vertex
            bool fixed = false;
            vector<VHandle> prevVertices = vertices;
            while (!fixed)
            {
                fixed = true;
                vector<VHandle> dupeTri = {getDuplicate(prevVertices[0]), vertices[1], vertices[2]};
                fh = mesh.add_face(dupeTri);
                if (fh.is_valid())
                {
                    mesh.data(prevVertices[0]).fixed = true;
                    mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                    mesh.data(fh).element = f.elem;
                    mesh.status(fh).set_fixed_nonmanifold(true);
                }
                else
                {
                    dupeTri = {vertices[0], getDuplicate(prevVertices[1]), vertices[2]};
                    fh = mesh.add_face(dupeTri);
                    if (fh.is_valid())
                    {
                        mesh.data(prevVertices[1]).fixed = true;
                        mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                        mesh.data(fh).element = f.elem;
                        mesh.status(fh).set_fixed_nonmanifold(true);
                    }
                    else
                    {
                        dupeTri = {vertices[0], vertices[1], getDuplicate(prevVertices[2])};
                        fh = mesh.add_face(dupeTri);
                        if (fh.is_valid())
                        {
                            mesh.data(fh).element = f.elem;
                            mesh.data(prevVertices[2]).fixed = true;
                            mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                            mesh.status(fh).set_fixed_nonmanifold(true);
                        }
                        else
                        {
                            // Duplicate two vertices
                            dupeTri = {getDuplicate(prevVertices[0]), getDuplicate(prevVertices[1]), vertices[2]};
                            fh = mesh.add_face(dupeTri);
                            if (fh.is_valid())
                            {
                                mesh.data(fh).element = f.elem;
                                mesh.data(prevVertices[0]).fixed = true;
                                mesh.data(prevVertices[1]).fixed = true;
                                mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                mesh.status(fh).set_fixed_nonmanifold(true);
                            }
                            else
                            {
                                dupeTri = {getDuplicate(prevVertices[0]), vertices[1], getDuplicate(prevVertices[2])};
                                fh = mesh.add_face(dupeTri);
                                if (fh.is_valid())
                                {
                                    mesh.data(fh).element = f.elem;
                                    mesh.data(prevVertices[0]).fixed = true;
                                    mesh.data(prevVertices[2]).fixed = true;
                                    mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                    mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                    mesh.status(fh).set_fixed_nonmanifold(true);
                                }
                                else
                                {
                                    dupeTri
                                        = {vertices[0], getDuplicate(prevVertices[1]), getDuplicate(prevVertices[2])};
                                    if (fh.is_valid())
                                    {
                                        mesh.data(fh).element = f.elem;
                                        mesh.data(prevVertices[1]).fixed = true;
                                        mesh.data(prevVertices[2]).fixed = true;
                                        mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                        mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                        mesh.status(fh).set_fixed_nonmanifold(true);
                                    }
                                    else
                                    {
                                        // Duplicate all three vertices
                                        dupeTri = {getDuplicate(prevVertices[0]),
                                                   getDuplicate(prevVertices[1]),
                                                   getDuplicate(prevVertices[2])};
                                        fh = mesh.add_face(dupeTri);
                                        if (fh.is_valid())
                                        {
                                            mesh.data(fh).element = f.elem;
                                            mesh.data(prevVertices[0]).fixed = true;
                                            mesh.data(prevVertices[1]).fixed = true;
                                            mesh.data(prevVertices[2]).fixed = true;
                                            mesh.status(prevVertices[0]).set_fixed_nonmanifold(true);
                                            mesh.status(prevVertices[1]).set_fixed_nonmanifold(true);
                                            mesh.status(prevVertices[2]).set_fixed_nonmanifold(true);
                                            mesh.status(fh).set_fixed_nonmanifold(true);
                                        }
                                        else
                                        {
                                            Logger::lout(Logger::WARN) << "Non-manifold element of complexity > {2 "
                                                                          "touching manifold meshes} encountered"
                                                                       << std::endl;
                                            prevVertices = dupeTri;
                                            fixed = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        std::cerr.clear();
    }

    // Delete duplicate vertices
    Logger::lout(Logger::DEBUG) << "Non-Manifold vertices: " << duplicates.size() - mesh.delete_isolated_vertices()
                                << std::endl;
    mesh.garbage_collection();

    Logger::lout(Logger::DEBUG) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built a single mesh from " << parts.size() << " parts" << std::endl;

    return mesh;
}

} // namespace c2m
