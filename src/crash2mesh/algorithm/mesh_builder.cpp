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
    ConnectedElement::Ptr elem;
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
                if (e2it != allTriangles[f2i].edges.end() && e2it->from->ID == e.from->ID
                    && !allTriangles[f2i].mark)
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

        // easy3d::Viewer viewer(std::string("Part ") + std::to_string(partptr->ID)); // DEBUG

        // Try flipping badly oriented faces
        // Create faces for parts
        std::vector<Triangle> allTriangles;
        std::map<Edge, vector<size_t>, Edge::CmpLess> edgeTriangleIndices;
        auto addTriangles = [&](const ConnectedElement::Ptr& elem) {
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

        for (uint i = 0; i < allTriangles.size(); i++)
        {
            if (!allTriangles[i].mark)
            {
                floodFlip(i, allTriangles, edgeTriangleIndices);
            }
        }

        Mesh& mesh = partptr->mesh;
        mesh = Mesh();
        map<nodeid_t, VHandle> nodeToVertex;
        // easy3d::SurfaceMesh* drawableMesh = new easy3d::SurfaceMesh;     // DEBUG
        // map<nodeid_t, easy3d::SurfaceMesh::Vertex> nodeToDrawableVertex; // DEBUG

        for (size_t fi = 0; fi < allTriangles.size(); fi++)
        {
            const Triangle& f = allTriangles[fi];
            vector<VHandle> vertices;
            // vector<easy3d::SurfaceMesh::Vertex> drawableVertices; // DEBUG
            for (uint i = 0; i < 3; i++)
            {
                const Node::Ptr& nodeptr = f.edges[i].from;
                if (nodeToVertex.find(nodeptr->ID) == nodeToVertex.end())
                {
                    VHandle v = mesh.add_vertex(OMVec3(nodeptr->coord(0), nodeptr->coord(1), nodeptr->coord(2)));
                    nodeToVertex[nodeptr->ID] = v;
                    mesh.data(v).node = nodeptr;
                    mesh.data(v).fixed = nodeptr->referencingParts > 1;
                    vertices.emplace_back(v);
                    // easy3d::SurfaceMesh::Vertex vDrawable = drawableMesh->add_vertex(
                    //     easy3d::vec3(nodeptr->coord[0], nodeptr->coord[1], nodeptr->coord[2])); // DEBUG
                    // drawableVertices.emplace_back(vDrawable);                                   // DEBUG
                    // nodeToDrawableVertex[nodeptr->ID] = vDrawable;                              // DEBUG
                }
                else
                {
                    vertices.emplace_back(nodeToVertex[nodeptr->ID]);
                    // drawableVertices.emplace_back(nodeToDrawableVertex[nodeptr->ID]); // DEBUG
                }
            }
            std::cerr.setstate(std::ios_base::failbit);
            Mesh::FaceHandle fh = mesh.add_face(vertices);
            if (fh.is_valid())
            {
                // drawableMesh->add_triangle(drawableVertices[0], drawableVertices[1], drawableVertices[2]); // DEBUG
                mesh.data(fh).element = f.elem;
                validTriangles++;
            }
            else
            {
                // Try duplicating and fixing until valid...this is bad....
                vector<VHandle> duplicates(3);
                // vector<easy3d::SurfaceMesh::Vertex> duplicatesDrawable(3); // DEBUG
                for (uint i = 0; i < 3; i++)
                {
                    // duplicatesDrawable[i]
                    //     = drawableMesh->add_vertex(drawableMesh->position(drawableVertices[i])); // DEBUG
                    duplicates[i] = mesh.add_vertex(mesh.point(vertices[i]));
                    mesh.data(duplicates[i]) = mesh.data(vertices[i]);
                    mesh.status(duplicates[i]).set_fixed_nonmanifold(true);
                }
                // Duplicate one vertex
                vector<VHandle> dupeTri = {duplicates[0], vertices[1], vertices[2]};
                fh = mesh.add_face(dupeTri);
                if (fh.is_valid())
                {
                    // drawableMesh->add_triangle(
                    //     duplicatesDrawable[0], drawableVertices[1], drawableVertices[2]); // DEBUG
                    mesh.data(duplicates[0]).fixed = mesh.data(vertices[0]).fixed = true;
                    mesh.data(fh).element = f.elem;
                    validTriangles++;
                }
                else
                {
                    invalidTriangles++;
                    dupeTri = {vertices[0], duplicates[1], vertices[2]};
                    fh = mesh.add_face(dupeTri);
                    if (fh.is_valid())
                    {
                        // drawableMesh->add_triangle(
                        //     drawableVertices[0], duplicatesDrawable[1], drawableVertices[2]); // DEBUG
                        mesh.data(vertices[1]).fixed = true;
                        mesh.status(vertices[1]).set_fixed_nonmanifold(true);
                        mesh.data(fh).element = f.elem;
                        mesh.status(fh).set_fixed_nonmanifold(true);
                    }
                    else
                    {
                        dupeTri = {vertices[0], vertices[1], duplicates[2]};
                        fh = mesh.add_face(dupeTri);
                        if (fh.is_valid())
                        {
                            // drawableMesh->add_triangle(
                            //     drawableVertices[0], drawableVertices[1], duplicatesDrawable[2]); // DEBUG
                            mesh.data(fh).element = f.elem;
                            mesh.data(vertices[2]).fixed = true;
                            mesh.status(vertices[2]).set_fixed_nonmanifold(true);
                            mesh.status(fh).set_fixed_nonmanifold(true);
                        }
                        else
                        {
                            // Duplicate two vertices
                            dupeTri = {duplicates[0], duplicates[1], vertices[2]};
                            fh = mesh.add_face(dupeTri);
                            if (fh.is_valid())
                            {
                                // drawableMesh->add_triangle(
                                //     duplicatesDrawable[0], duplicatesDrawable[1], drawableVertices[2]); // DEBUG
                                mesh.data(fh).element = f.elem;
                                mesh.data(vertices[0]).fixed = true;
                                mesh.data(vertices[1]).fixed = true;
                                mesh.status(vertices[0]).set_fixed_nonmanifold(true);
                                mesh.status(vertices[1]).set_fixed_nonmanifold(true);
                                mesh.status(fh).set_fixed_nonmanifold(true);
                            }
                            else
                            {
                                dupeTri = {duplicates[0], vertices[1], duplicates[2]};
                                fh = mesh.add_face(dupeTri);
                                if (fh.is_valid())
                                {
                                    // drawableMesh->add_triangle(
                                    //     duplicatesDrawable[0], drawableVertices[1], duplicatesDrawable[2]); // DEBUG
                                    mesh.data(fh).element = f.elem;
                                    mesh.data(vertices[0]).fixed = true;
                                    mesh.data(vertices[2]).fixed = true;
                                    mesh.status(vertices[0]).set_fixed_nonmanifold(true);
                                    mesh.status(vertices[2]).set_fixed_nonmanifold(true);
                                    mesh.status(fh).set_fixed_nonmanifold(true);
                                }
                                else
                                {
                                    dupeTri = {vertices[0], duplicates[1], duplicates[2]};
                                    if (fh.is_valid())
                                    {
                                        // drawableMesh->add_triangle(
                                        //     duplicatesDrawable[0], duplicatesDrawable[1], drawableVertices[2]); //
                                        //     DEBUG
                                        mesh.data(fh).element = f.elem;
                                        mesh.data(vertices[1]).fixed = true;
                                        mesh.data(vertices[2]).fixed = true;
                                        mesh.status(vertices[1]).set_fixed_nonmanifold(true);
                                        mesh.status(vertices[2]).set_fixed_nonmanifold(true);
                                        mesh.status(fh).set_fixed_nonmanifold(true);
                                    }
                                    else
                                    {
                                        // Duplicate all three vertices
                                        dupeTri = {duplicates[0], duplicates[1], duplicates[2]};
                                        fh = mesh.add_face(dupeTri);
                                        if (fh.is_valid())
                                        {
                                            // drawableMesh->add_triangle(duplicatesDrawable[0],
                                            //                            duplicatesDrawable[1],
                                            //                            duplicatesDrawable[2]); // DEBUG
                                            mesh.data(fh).element = f.elem;
                                            mesh.data(vertices[0]).fixed = true;
                                            mesh.data(vertices[1]).fixed = true;
                                            mesh.data(vertices[2]).fixed = true;
                                            mesh.status(vertices[0]).set_fixed_nonmanifold(true);
                                            mesh.status(vertices[1]).set_fixed_nonmanifold(true);
                                            mesh.status(vertices[2]).set_fixed_nonmanifold(true);
                                            mesh.status(fh).set_fixed_nonmanifold(true);
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

        if (deleteMeshedElements)
        {
            partptr->elements2D.clear();
            partptr->surfaceElements.clear();
        }

        meshes++;
    }
    Logger::lout(Logger::DEBUG) << "Non-manifold triangles: " << invalidTriangles
                                << ", valid triangles: " << validTriangles << std::endl;

    Logger::lout(Logger::INFO) << "Built " << meshes << " meshes from " << parts.size()
                               << " parts, rest didnt contain faces" << std::endl;

    return true;
}

} // namespace c2m
