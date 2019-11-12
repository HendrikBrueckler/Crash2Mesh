#include <crash2mesh/fileio/erf_hdf5_reader.hpp>
#include <crash2mesh/util/logger.hpp>

using namespace std;
using namespace c2m;

void testReader(const ERFHDF5Reader& reader)
{
    Logger::lout(Logger::INFO) << "testReader(): starting" << endl;
    map<entid_t, VolVec3> id2pos;
    if (!reader.readVertices(id2pos))
        Logger::lout(Logger::ERROR) << "\t\ttesting readVertices() failed!" << endl;

    Logger::lout(Logger::INFO) << "testReader(): finished" << endl;
}

int main(int argc, char** argv)
{
    if (argc != 2)
        return -1;

    ERFHDF5Reader reader(argv[1]);

    testReader(reader);

    return 0;
}
