#include <crash2mesh/io/erfh5/reader.hpp>
#include <crash2mesh/algorithm/surface_extractor.hpp>

#include <crash2mesh/util/logger.hpp>

using namespace std;
using namespace c2m;
using namespace erfh5;

void testSurfaceExtraction(const map<elemid_t, std::vector<Element3D::Ptr>>& volumes)
{
    map<partid_t, std::vector<SurfaceElement::Ptr>> partIDToSurfaceElements;
    if (!SurfaceExtractor::extract(volumes, partIDToSurfaceElements))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting surfaceextraction failed!" << endl;
    }
}

void testReader(const Reader& reader)
{
    Logger::lout(Logger::INFO) << "testReader(): starting" << endl;

    map<nodeid_t, Node::Ptr> id2pos;
    if (!reader.readNodes(id2pos))
        Logger::lout(Logger::ERROR) << "\t\ttesting readNodes() failed!" << endl;

    map<partid_t, vector<Element1D::Ptr>> partIDToElements1D;
    if (!reader.read1DElements(id2pos, partIDToElements1D))
        Logger::lout(Logger::ERROR) << "\t\ttesting read1DElements() failed!" << endl;

    map<partid_t, vector<Element2D::Ptr>> partIDToElements2D;
    if (!reader.read2DElements(id2pos, partIDToElements2D))
        Logger::lout(Logger::ERROR) << "\t\ttesting read2DElements() failed!" << endl;

    map<partid_t, vector<Element3D::Ptr>> partIDToElements3D;
    if (!reader.read3DElements(id2pos, partIDToElements3D))
        Logger::lout(Logger::ERROR) << "\t\ttesting read3DElements() failed!" << endl;

    if (!partIDToElements3D.empty())
    {
        testSurfaceExtraction(partIDToElements3D);
    }

    vector<Part::Ptr> parts;
    if (!reader.readParts(parts))
        Logger::lout(Logger::ERROR) << "\t\ttesting readParts() failed!" << endl;

    Logger::lout(Logger::INFO) << "testReader(): finished" << endl;
}

int main(int argc, char** argv)
{
    if (argc != 2)
        return -1;

    Reader reader(argv[1]);

    testReader(reader);

    return 0;
}
