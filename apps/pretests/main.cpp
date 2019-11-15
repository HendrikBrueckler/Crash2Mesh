#include <crash2mesh/fileio/erfh5/reader.hpp>
#include <crash2mesh/util/logger.hpp>

using namespace std;
using namespace c2m;
using namespace erfh5;

void testReader(const Reader& reader)
{
    Logger::lout(Logger::INFO) << "testReader(): starting" << endl;

    map<entid_t, VolVec3> id2pos;
    if (!reader.readVertices(id2pos))
        Logger::lout(Logger::ERROR) << "\t\ttesting readVertices() failed!" << endl;

    std::map<const FEType*, std::map<entid_t, std::vector<entid_t>>> elementType2Elements;
    if (!reader.readFiniteElements(elementType2Elements))
        Logger::lout(Logger::ERROR) << "\t\ttesting read2DElements() failed!" << endl;

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
