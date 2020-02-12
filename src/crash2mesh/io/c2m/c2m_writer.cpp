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

// TODO
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

    uint nFrames = nodes.size() == 0 ? 0 : (*nodes.begin())->positions.rows();
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
            numRods = partID2RodIDs[partID].size();
            numTriangles = partID2TriangleIDs[partID].size();
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
            numRods = partID2RodIDs[partID].size();
            numTriangles = partID2TriangleIDs[partID].size();
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

/*
C2M Output Format Specification VERSION 1.0

============================
General Structure:
============================
$$SECTION HEADER$$
    ...essential ascii information for parsing rest of sections...
$$ENDSECTION HEADER$$
$$SECTION VERTICES$$
    ...binary or ascii mapping of vertex id to per-frame xyz-coordinates...
$$ENDSECTION VERTICES$$
$$SECTION FACESTRAINS$$
    ...binary or ascii mapping of facestrain id to per-frame plastic strain values...
$$ENDSECTION FACESTRAINS$$
$$SECTION RODS$$
    ...binary or ascii mapping of rod/line id to its 2 vertex ids (source and target)...
$$ENDSECTION RODS$$
$$SECTION TRIANGLES$$
    ...binary or ascii mapping of triangle id to its facestrain id and its 3 vertex ids ...
$$ENDSECTION TRIANGLES$$
$$SECTION PARTS$$
    ...binary or ascii mapping of part id to its number of rods, its number of triangles,
            its list of rod ids and its list of triangle ids...
$$ENDSECTION PARTS$$

============================
Section specification:
============================
Info1: in verbatim content marked with <...> is variable, rest is FIXED
Info2: there is a single newline between the end of one section and the beginning of a new one
Info3: the exact amount of whitespace in ascii-format may be subject to change!

------------------------
Header section verbatim:
------------------------
$$SECTION HEADER$$
version 1.0
binary <0 if other sections in ascii, 1 if other sections in binary>
frames <number of frames>
vertices <number of vertices>
facestrains <number of surface plastic strains>
rods <number of 1D elements, i.e. lines which are not part of any triangle>
triangles <number of triangles>
parts <number of parts>
$$ENDSECTION HEADER$$
------------------------
Header section info:
------------------------
- header section is always the first section
- whitespace exactly as depicted, i.e. a space between identifier and value
  and a newline after each identifier-value-pair
- all numbers marked with <...> are ascii-strings

------------------------
Vertices section verbatim:
------------------------
$$SECTION VERTICES$$
<VERTEX1_ID>
    <x-position frame0> <y-position frame0> <z-position frame0>
    <x-position frame1> <y-position frame1> <z-position frame1>
    <x-position frame2> <y-position frame2> <z-position frame2>
    <...repeated for a total of "frames" times (see header)...>
<VERTEX2_ID>
    <x-position frame0> <y-position frame0> <z-position frame0>
    <x-position frame1> <y-position frame1> <z-position frame1>
    <x-position frame2> <y-position frame2> <z-position frame2>
    <...repeated for a total of "frames" times (see header)...>
<...repeated for a total of "vertices" times (see header)...>
$$ENDSECTION VERTICES$$
------------------------
Vertices section info:
------------------------
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION VERTICES$$, after each <VERTEX_ID> and after
        each set of xyz-coordinates
    - four spaces before each set of xyz-coordinates
    - a single space between x and y coordinates, y and z coordinates
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION VERTICES$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
        $$ENDSECTION VERTICES$$
- in binary format VERTEX_ID is a 4 byte unsigned integer, each x/y/z coordinate
  is a 4 byte float (i.e. NOT double precision)

------------------------
Facestrains section verbatim:
------------------------
$$SECTION FACESTRAINS$$
<FACESTRAIN1_ID>
    <plastic strain1 at frame1>
    <plastic strain1 at frame2>
    <plastic strain1 at frame3>
    <...repeated for a total of "frames" times (see header)...>
<FACESTRAIN2_ID>
    <plastic strain2 at frame1>
    <plastic strain2 at frame2>
    <plastic strain2 at frame3>
    <...repeated for a total of "frames" times (see header)...>
<...repeated for a total of "facestrains" times (see header)...>
$$ENDSECTION FACESTRAINS$$
------------------------
Facestrains section info:
------------------------
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION FACESTRAINS$$, after each <FACESTRAIN_ID> and after
        each plastic strain value
    - four spaces before each plastic strain value
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION FACESTRAINS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
        $$ENDSECTION FACESTRAINS$$
- in binary format FACESTRAIN_ID is a 4 byte unsigned integer, plastic strain
  is a 4 byte float (i.e. NOT double precision)

------------------------
Rods section verbatim:
------------------------
$$SECTION RODS$$
<ROD1_ID> <ROD1_VERTEX1_ID> <ROD1_VERTEX2_ID>
<ROD2_ID> <ROD2_VERTEX1_ID> <ROD2_VERTEX2_ID>
<...repeated for a total of "rods" times (see header)>
$$ENDSECTION RODS$$
------------------------
Rods section info:
------------------------
- VERTEX_IDs reference the Vertices section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION RODS$$ and after each specified Rod
    - a single space in between ROD_ID and each of the two following VERTEX_IDs
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION RODS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION RODS$$
- in binary format all IDs are 4byte unsigned integers

------------------------
TRIANGLES section verbatim:
------------------------
$$SECTION TRIANGLES$$
<TRIANGLE1_ID> <TRIANGLE1_FACESTRAIN_ID> <TRIANGLE1_VERTEX1_ID> <TRIANGLE1_VERTEX2_ID> <TRIANGLE1_VERTEX3_ID>
<TRIANGLE2_ID> <TRIANGLE2_FACESTRAIN_ID> <TRIANGLE2_VERTEX1_ID> <TRIANGLE2_VERTEX2_ID> <TRIANGLE2_VERTEX3_ID>
<...repeated for a total of "triangles" times (see header)>
$$ENDSECTION TRIANGLES$$
------------------------
Triangles section info:
------------------------
- VERTEX_IDs reference the Vertices section entries by their specified indices
- FACESTRAIN_IDs reference the Facestrains section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION TRIANGLES$$, and after each specified triangle
    - a single space in between each subsequent ID
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION TRIANGLES$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION TRIANGLES$$
- in binary format all IDs are 4byte unsigned integers

------------------------
Parts section verbatim:
------------------------
$$SECTION PARTS$$
<PART1_ID> <numberOfRods in part1> <numberOfTriangles in part1>
    <PART1_ROD_ID1> <PART1_ROD_ID2> <...repeated a total of numberOfRods times...>
    <PART1_TRIANGLE_ID1> <PART1_TRIANGLE_ID2> <...repeated a total of numberOfTriangles times...>
<PART2_ID> <numberOfRods in part2> <numberOfTriangles in part2>
    <PART2_ROD_ID1> <PART2_ROD_ID2> <...repeated a total of numberOfRods times...>
    <PART2_TRIANGLE_ID1> <PART2_TRIANGLE_ID2> <...repeated a total of numberOfTriangles times...>
<...repeated for a total of "parts" times (see header)>
$$ENDSECTION PARTS$$
------------------------
Parts section info:
------------------------
- ROD_ID reference the rods section entries by their specified indices
- TRIANGLE_IDs reference the triangles section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION PARTS$$, after each PART_ID-number-number triplet,
      after each list of ROD_IDs and after each list of TRIANGLE_IDs
    - four spaces before each list of ROD_IDs and before each list of TRIANGLE_IDs
    - a single space in between PART_ID and its number of rods and its number of triangles,
      as well as between each subsequent entry in the lists of ROD_IDs and TRIANGLE_IDs
    - IMPORTANT: in ascii if numberOfRods or numberOfTriangles is 0, the respective line is missing,
        i.e. there is no empty line.
        example:
        41 0 0              ||<- neither rods nor triangles
        42 0 3
            1 2 3           ||<- only triangles
        43 2 0
            20 22           ||<- only rods
        44 4 2
            4 5 6 7         ||<- rods
            23 24           ||<- ...and triangles
        ...and so on...
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION PARTS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION PARTS$$
- in binary format the number of rods, number of triangles and all IDs are 4byte unsigned integers

*/

} // namespace c2m
