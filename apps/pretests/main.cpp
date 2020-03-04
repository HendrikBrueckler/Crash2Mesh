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

int main(int argc, char** argv)
{
    if (argc < 2)
        return -1;

    vector<Part::Ptr> parts;
    std::string filename = argv[1];
    Reader reader(filename);

    if (!reader.readParts(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting readParts() failed!" << endl;
        return -1;
    }
    if (!SurfaceExtractor::extract(parts))
    {
        Logger::lout(Logger::ERROR) << "\t\ttesting SurfaceExtractor::extract() failed!" << endl;
    }
    if (argc == 3)
    {
        int nFaces = 0;
        if (argv[2][0] == 'e')
        {
            Part::Ptr merged = std::make_shared<Part>(0, 0);
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
            MeshDecimater deci;
            MeshAnalyzer::getEpicenter(parts, deci.epicenters, deci.meanDistsFromEpicenters);
            deci.useQuadric = true;
            deci.framesQuadric = 1000;
            deci.maxQuadricError = 100;
            deci.quadricAreaWeighting = false;
            deci.quadricPositionOptimization = false;
            deci.useNormalDeviation = true;
            deci.framesNormalDeviation = 20;
            deci.maxNormalDeviation = 5;
            deci.useBoundaryDeviation = true;
            deci.framesBoundaryDeviation = 5;
            deci.maxBoundaryDeviation = 5;
            deci.useAspectRatio = true;
            deci.maxAspectRatio = 20;
            deci.maxVLog = 10000000;
            deci.maxVRender = 100000;
            deci.minVLog = 1000000000;
            deci.minVRender = 1000000000;
            deci.queryLogParts();
            if (!deci.decimateParts(parts))
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
            deci.useQuadric = true;
            deci.framesQuadric = 10;
            deci.maxQuadricError = FLT_MAX;
            deci.quadricAreaWeighting = false; // this has no effect, as quadrics are used from previous decimation
            deci.quadricPositionOptimization = false;
            deci.useNormalDeviation = true;
            deci.framesNormalDeviation = 5;
            deci.maxNormalDeviation = 30;
            deci.useBoundaryDeviation = true;
            deci.framesBoundaryDeviation = 3;
            deci.maxBoundaryDeviation = 15;
            deci.useAspectRatio = true;
            deci.maxAspectRatio = 20;
            deci.minVLog = 1000000000;
            deci.minVRender = 1000000000;
            deci.maxVLog = 10000000;
            deci.maxVRender = 200000;
            deci.queryLogParts();
            if (!deci.decimateScene(scene, nFaces))
            {
                Logger::lout(Logger::ERROR) << "\t\ttesting scene decimation failed!" << endl;
                return -1;
            }

            C2MWriter::write(filename + ".c2m", scene, true);
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
