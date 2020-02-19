#include <crash2mesh/io/c2m/c2m_writer.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace c2m
{
using std::ofstream;
using std::string;
using std::stringstream;
using std::vector;
using std::set;
using std::map;

struct OutputTriangle
{
    OutputTriangle(elemid_t _ID, elemid_t _strainID, nodeid_t _n1, nodeid_t _n2, nodeid_t _n3)
        : ID(_ID), strainID(_strainID), n1(_n1), n2(_n2), n3(_n3)
    {
    }

    elemid_t ID;
    elemid_t strainID;
    nodeid_t n1;
    nodeid_t n2;
    nodeid_t n3;
};

struct Rod
{
    Rod(elemid_t _ID, nodeid_t _n1, nodeid_t _n2) : ID(_ID), n1(_n1), n2(_n2)
    {
    }
    elemid_t ID;
    nodeid_t n1;
    nodeid_t n2;
};

struct NodePtrLess
{
    bool operator()(const Node::Ptr &lhs, const Node::Ptr &rhs) const
    {
        return lhs->ID < rhs->ID;
    }
};

struct Element1DPtrLess
{
    bool operator()(const Element1D::Ptr &lhs, const Element1D::Ptr &rhs) const
    {
        return lhs->elem1dID < rhs->elem1dID;
    }
};

struct Element2DPtrLess
{
    bool operator()(const Element2D::Ptr &lhs, const Element2D::Ptr &rhs) const
    {
        return lhs->elem2dID < rhs->elem2dID;
    }
};

void C2MWriter::write(const string& filename, const Scene::Ptr& scene, bool binary)
{
    ofstream out;
    if (binary)
        out = ofstream(filename, std::ios::binary);
    else
        out = ofstream(filename);

    // collect
    set<Node::Ptr, NodePtrLess> nodes;
    set<Element1D::Ptr, Element1DPtrLess> elements1D;
    set<Element2D::Ptr, Element2DPtrLess> elements2D;
    map<partid_t, set<elemid_t>> partID2RodIDs;
    map<partid_t, set<elemid_t>> partID2TriangleIDs;

    vector<Rod> rods;
    vector<OutputTriangle> triangles;

    for (const Part::Ptr& partptr : scene->parts)
    {
        for (const Element1D::Ptr& element1Dptr : partptr->elements1D)
        {
            elements1D.emplace(element1Dptr);
        }
    }
    for (const Element1D::Ptr element1Dptr : elements1D)
    {
        partID2RodIDs[element1Dptr->partID].emplace(element1Dptr->elem1dID);
        rods.emplace_back(element1Dptr->elem1dID, element1Dptr->nodes[0]->ID, element1Dptr->nodes[1]->ID);
    }

    const CMesh& mesh(scene->mesh);
    for (const VHandle& v : mesh.vertices())
    {
        nodes.emplace(mesh.data(v).node);
    }

    for (const FHandle& f : mesh.faces())
    {
        const Element2D::Ptr& element = mesh.data(f).element;
        elements2D.emplace(element);
        CMesh::ConstFaceVertexIter fv = mesh.cfv_begin(f);
        VHandle v1 = *(fv++);
        VHandle v2 = *(fv++);
        VHandle v3 = *fv;
        nodeid_t n1 = mesh.data(v1).node->ID;
        nodeid_t n2 = mesh.data(v2).node->ID;
        nodeid_t n3 = mesh.data(v3).node->ID;
        partID2TriangleIDs[element->partID].emplace(f.idx());
        triangles.emplace_back(static_cast<uint>(f.idx()), element->elem2dID, n1, n2, n3);
    }

    uint nFrames = nodes.size() == 0 ? 0 : static_cast<uint>((*nodes.begin())->positions.rows());
    // output
    out << "$$SECTION HEADER$$\n"
        << "version " << 0.1 << "\n"
        << "binary " << binary << "\n"
        << "frames " << nFrames << "\n"
        << "vertices " << nodes.size() << "\n"
        << "facestrains " << elements2D.size() << "\n"
        << "rods " << rods.size() << "\n"
        << "triangles " << triangles.size() << "\n"
        << "$$ENDSECTION HEADER$$" << std::endl;

    if (!binary)
    {
        out << "$$SECTION VERTICES$$\n";
        for (const Node::Ptr& nodeptr : nodes)
        {
            out << nodeptr->ID << "\n";
            for (uint i = 0; i < nFrames; i++)
            {
                out << "    " << nodeptr->positions.coeff(i, 0) << " " << nodeptr->positions.coeff(i, 1) << " "
                    << nodeptr->positions.coeff(i, 2) << "\n";
            }
        }
        out << "$$ENDSECTION VERTICES$$" << std::endl;

        out << "$$SECTION FACESTRAINS$$\n";
        for (const Element2D::Ptr& elem2Dptr : elements2D)
        {
            out << elem2Dptr->elem2dID << "\n";
            for (uint i = 0; i < nFrames; i++)
            {
                out << "    " << elem2Dptr->plasticStrains.row(i) << "\n";
            }
        }
        out << "$$ENDSECTION FACESTRAINS$$" << std::endl;

        out << "$$SECTION RODS$$\n";
        for (const Rod& rod : rods)
        {
            out << rod.ID << " " << rod.n1 << " " << rod.n2 << "\n";
        }
        out << "$$ENDSECTION RODS$$" << std::endl;

        out << "$$SECTION TRIANGLES$$\n";
        for (const OutputTriangle& triangle : triangles)
        {
            out << triangle.ID << " " << triangle.strainID << " " << triangle.n1 << " " << triangle.n2 << " "
                << triangle.n3 << "\n";
        }
        out << "$$ENDSECTION TRIANGLES$$" << std::endl;

        out << "$$SECTION PARTS$$\n";
        partid_t partID;
        elemid_t numRods, numTriangles;
        for (const Part::Ptr& partptr : scene->parts)
        {
            partID = partptr->ID;
            numRods = static_cast<elemid_t>(partID2RodIDs[partID].size());
            numTriangles = static_cast<elemid_t>(partID2TriangleIDs[partID].size());
            out << partID << " " << numRods << " " << numTriangles << "\n";
            bool first;
            if (numRods != 0)
            {
                out << "    ";
                first = true;
                for (const elemid_t rodID: partID2RodIDs[partID])
                {
                    out << (first ? "" : " ") << rodID;
                    first = false;
                }
                out << "\n";
            }
            if (numTriangles != 0)
            {
                out << "    ";
                first = true;
                for (const elemid_t triangleID: partID2TriangleIDs[partID])
                {
                    out << (first ? "" : " ") << triangleID;
                    first = false;
                }
                out << "\n";
            }
        }
        out << "$$ENDSECTION PARTS$$" << std::endl;
    }
    else
    {
        uint id;
        float x, y, z, strain;
        out << "$$SECTION VERTICES$$\n";
        for (const Node::Ptr& nodeptr : nodes)
        {
            id = nodeptr->ID;
            out.write(reinterpret_cast<char*>(&id), sizeof(uint));
            for (uint i = 0; i < nFrames; i++)
            {
                x = nodeptr->positions.coeff(i, 0);
                y = nodeptr->positions.coeff(i, 1);
                z = nodeptr->positions.coeff(i, 2);
                out.write(reinterpret_cast<char*>(&x), sizeof(float));
                out.write(reinterpret_cast<char*>(&y), sizeof(float));
                out.write(reinterpret_cast<char*>(&z), sizeof(float));
            }
        }
        out << "\n$$ENDSECTION VERTICES$$" << std::endl;

        out << "$$SECTION FACESTRAINS$$\n";
        for (const Element2D::Ptr& elem2Dptr : elements2D)
        {
            id = elem2Dptr->elem2dID;
            out.write(reinterpret_cast<char*>(&id), sizeof(uint));
            for (uint i = 0; i < nFrames; i++)
            {
                strain = elem2Dptr->plasticStrains.coeff(i);
                out.write(reinterpret_cast<char*>(&strain), sizeof(float));
            }
        }
        out << "\n$$ENDSECTION FACESTRAINS$$" << std::endl;

        out << "$$SECTION RODS$$\n";
        for (const Rod& rod : rods)
        {
            out.write(reinterpret_cast<const char*>(&rod), sizeof(Rod));
            id++;
        }
        out << "\n$$ENDSECTION RODS$$" << std::endl;

        out << "$$SECTION TRIANGLES$$\n";
        for (const OutputTriangle& triangle : triangles)
        {
            out.write(reinterpret_cast<const char*>(&triangle), sizeof(OutputTriangle));
        }
        out << "\n$$ENDSECTION TRIANGLES$$" << std::endl;

        out << "$$SECTION PARTS$$\n";
        partid_t partID;
        elemid_t numRods, numTriangles;
        for (const Part::Ptr& partptr : scene->parts)
        {
            partID = partptr->ID;
            numRods = static_cast<elemid_t>(partID2RodIDs[partID].size());
            numTriangles = static_cast<elemid_t>(partID2TriangleIDs[partID].size());
            out.write(reinterpret_cast<const char*>(&partID), sizeof(partid_t));
            out.write(reinterpret_cast<const char*>(&numRods), sizeof(elemid_t));
            out.write(reinterpret_cast<const char*>(&numTriangles), sizeof(elemid_t));
            for (const elemid_t rodID: partID2RodIDs[partID])
                out.write(reinterpret_cast<const char*>(&rodID), sizeof(elemid_t));
            for (const elemid_t triangleID: partID2TriangleIDs[partID])
                out.write(reinterpret_cast<const char*>(&triangleID), sizeof(elemid_t));

        }
        out << "\n$$ENDSECTION PARTS$$" << std::endl;
    }
}

} // namespace c2m
