#include <crash2mesh/algorithm/mesh_builder.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/surface_extractor.hpp>
#include <crash2mesh/io/erfh5/reader.hpp>
#include <crash2mesh/io/c2m/c2m_writer.hpp>

#include <crash2mesh/util/logger.hpp>

using namespace std;
using namespace c2m;
using namespace erfh5;

void testReader(const Reader& reader, vector<Part::Ptr>& parts)
{
    Logger::lout(Logger::INFO) << "testReader(): starting" << endl;

    // map<nodeid_t, Node::Ptr> id2pos;
    // if (!reader.readNodes(id2pos))
    //     Logger::lout(Logger::ERROR) << "\t\ttesting readNodes() failed!" << endl;

    // map<partid_t, vector<Element1D::Ptr>> partIDToElements1D;
    // if (!reader.read1DElements(id2pos, partIDToElements1D))
    //     Logger::lout(Logger::ERROR) << "\t\ttesting read1DElements() failed!" << endl;

    // map<partid_t, vector<Element2D::Ptr>> partIDToElements2D;
    // if (!reader.read2DElements(id2pos, partIDToElements2D))
    //     Logger::lout(Logger::ERROR) << "\t\ttesting read2DElements() failed!" << endl;

    // if (!reader.read3DElements(id2pos, partIDToElements3D))
    //     Logger::lout(Logger::ERROR) << "\t\ttesting read3DElements() failed!" << endl;

    if (!reader.readParts(parts))
        Logger::lout(Logger::ERROR) << "\t\ttesting readParts() failed!" << endl;

    Logger::lout(Logger::INFO) << "testReader(): finished" << endl;
}

void testSurfaceExtraction(vector<Part::Ptr>& parts)
{
    Logger::lout(Logger::INFO) << "SurfaceExtractor::extract(): starting" << endl;
    if (!SurfaceExtractor::extract(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting SurfaceExtractor::extract() failed!" << endl;
    }
    Logger::lout(Logger::INFO) << "SurfaceExtractor::extract(): finished" << endl;
}

void testMeshBuilding(vector<Part::Ptr>& parts)
{
    Logger::lout(Logger::INFO) << "MeshBuilder::build: starting" << endl;

    std::vector<CMesh> meshes;
    if (!MeshBuilder::build(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting surfaceextraction failed!" << endl;
    }

    Logger::lout(Logger::INFO) << "MeshBuilder::build: finished" << endl;
}

void testMeshDecimation(vector<Part::Ptr>& parts)
{
    Logger::lout(Logger::INFO) << "MeshDecimater::decimate(): starting" << endl;

    std::vector<CMesh> meshes;
    if (!MeshDecimater::decimateSimple(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting mesh decimation failed!" << endl;
    }

    Logger::lout(Logger::INFO) << "MeshDecimater::decimate(): finished" << endl;
}

void tests(std::string filename)
{
    Reader reader(filename);

    vector<Part::Ptr> parts;
    testReader(reader, parts);

    if (!parts.empty())
    {
        testSurfaceExtraction(parts);
        testMeshBuilding(parts);
        testMeshDecimation(parts);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
        return -1;

    return 0;
}
