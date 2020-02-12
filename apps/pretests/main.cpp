#include <crash2mesh/algorithm/mesh_builder.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
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

    map<partid_t, std::vector<SurfaceElement::Ptr>> partIDToSurfaceElements;
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

    vector<Part::Ptr> parts;
    Reader reader(argv[1]);

    if (!reader.readParts(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting readParts() failed!" << endl;
        return -1;
    }
    if (argc == 3)
    {
        int nFaces = 0;
        if (argv[2][0] == 'e')
        {
            Part::Ptr merged = std::make_shared<Part>(0);
            merged->mesh = MeshBuilder::buildSingle(parts);

            for (Part::Ptr& partptr : parts)
                partptr->markElement1DNodes();

            vector<Part::Ptr> dummyParts({merged});
            if (!MeshDecimater::decimateSimple(dummyParts))
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting mesh decimation failed!" << endl;
                return -1;
            }
        }
        else
        {
            nFaces = atoi(argv[2]);

            if (!MeshBuilder::build(parts))
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting mesh building failed!" << endl;
                return -1;
            }
            if (!MeshDecimater::decimatePartsErrorBound(parts, 100, 5, 5, 3, 3, 10))
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting errorbound part decimation failed!" << endl;
                return -1;
            }
            Scene::Ptr scene = MeshBuilder::merge(parts);
            if (!scene)
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting scene merging failed!" << endl;
                return -1;
            }
            if (!MeshDecimater::decimateScene(scene, nFaces, 20, 20, 5, 5, 20))
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting scene decimation failed!" << endl;
                return -1;
            }

            // TODO choose proper filename
            // C2MWriter::write("test.c2m", scene, true);
        }

    }
    else
    {
        if (!MeshBuilder::build(parts))
        {
            Logger::lout(Logger::ERROR) << "\t\ttesting mesh building failed!" << endl;
            return -1;
        }
        if (!MeshDecimater::decimateSimple(parts))
        {
            Logger::lout(Logger::ERROR) << "\t\ttesting mesh decimation failed!" << endl;
            return -1;
        }
    }


    return 0;
}
