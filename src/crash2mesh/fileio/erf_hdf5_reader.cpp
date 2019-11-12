#include <crash2mesh/fileio/erf_hdf5_reader.hpp>

#include <crash2mesh/fileio/erf_hdf5_path.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

#include <highfive/H5Easy.hpp>

#include <map>
#include <stdexcept>

namespace c2m
{
using std::vector;

bool ERFHDF5Reader::readVertices(std::map<entid_t, VolVec3>& vertexEntityID2Coordinates) const
{
    vertexEntityID2Coordinates.clear();

    vector<entid_t> vertexEntityIDs;
    vector<vector<double>> vertexCoordinates;

    ERFHDF5Path path(ERFHDF5Path::Group::CSMEXPL);
    path.descend(ERFHDF5Path::Group::CONSTANT)
        .descend(ERFHDF5Path::Group::ENTITYRESULTS)
        .descend(ERFHDF5Path::Group::NODE)
        .descend(ERFHDF5Path::Group::COORDINATE)
        .descend(ERFHDF5Path::Group::ZONE1_SET0)
        .dataset(ERFHDF5Path::Dataset::ENTITY_ID_BY_ENTITY);
    try
    {
        vertexEntityIDs = H5Easy::load<vector<entid_t>>(m_file, path.pathToData());
    }
    catch (H5Exception)
    {
        Logger::lout(Logger::ERROR) << "Couldn't read vertex indices, "
                                    << "maybe path \"" << path.pathToData() << "\" didn't exist in " << m_file.getName()
                                    << "?" << std::endl;
        return false;
    }

    path.dataset(ERFHDF5Path::Dataset::RESULT_BY_ENTITY);
    try
    {
        vertexCoordinates = H5Easy::load<vector<vector<double>>>(m_file, path.pathToData());
    }
    catch (H5Exception)
    {
        Logger::lout(Logger::ERROR) << "Couldn't read vertex Coordinatess, "
                                    << "maybe path \"" << path.pathToData() << "\" didn't exist in " << m_file.getName()
                                    << "?" << std::endl;
        return false;
    }

    if (vertexCoordinates.size() == 0)
    {
        Logger::lout(Logger::INFO) << "Path " << path.pathToData() << "existed,"
                                   << " but was empty " << std::endl;
        return false;
    }

    if (vertexCoordinates[0].size() != 3 || vertexEntityIDs.size() != vertexCoordinates.size())
    {
        Logger::lout(Logger::INFO) << "Ill-formed dimension of vertex IDs or coordinates" << std::endl;
        return false;
    }

    for (uint i = 0; i < vertexEntityIDs.size(); i++)
    {
        vertexEntityID2Coordinates[vertexEntityIDs[i]]
            = VolVec3(vertexCoordinates[i][0], vertexCoordinates[i][1], vertexCoordinates[i][2]);
    }

    return true;
}
} // namespace c2m
