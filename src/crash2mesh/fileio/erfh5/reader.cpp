#include <crash2mesh/fileio/erfh5/reader.hpp>

#include <crash2mesh/fileio/erfh5/filecontents.hpp>
#include <crash2mesh/util/logger.hpp>

#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

#include <highfive/H5Easy.hpp>

#include <map>
#include <stdexcept>

namespace c2m
{
namespace erfh5
{
using std::vector;

void Reader::logPathError(const std::string& msg, const std::string& path) const
{
    Logger::lout(Logger::ERROR) << msg << " (path \"" << path << "\" of file " << m_file.getName() << std::endl;
}

void Reader::logPathInfo(const std::string& msg, const std::string& path) const
{
    Logger::lout(Logger::INFO) << msg << " (path \"" << path << "\" of file " << m_file.getName() << std::endl;
}

bool Reader::readVertices(std::map<entid_t, VolVec3>& vertexEntityID2Coordinates) const
{
    vertexEntityID2Coordinates.clear();

    if (!m_file.isValid())
    {
        logPathError("Invalid file \"", "N/A");
    }

    vector<entid_t> vertexEntityIDs;
    vector<vector<double>> vertexCoordinates;

    std::string vertexIDpath = FEType::NODE.pathToResult(ResultType::COORDINATE, DataType::ENTITY_IDS);
    std::string vertexCoordPath = FEType::NODE.pathToResult(ResultType::COORDINATE, DataType::RESULTS);
    if (m_file.exist(vertexIDpath)
        && m_file.exist(vertexCoordPath))
    {
        try
        {
            vertexEntityIDs = H5Easy::load<vector<entid_t>>(m_file, vertexIDpath);
            vertexCoordinates = H5Easy::load<vector<vector<double>>>(m_file, vertexCoordPath);
        }
        catch (H5Exception)
        {
            logPathError("HDF5 reading error ", vertexCoordPath);
            return false;
        }
        if (vertexCoordinates.size() == 0)
        {
            logPathError("Path existed but was empty ", vertexCoordPath);
            return false;
        }
    }
    else
    {
        logPathError("Path to vertex coordinates does not exist", vertexCoordPath);
        return false;
    }

    for (uint i = 0; i < vertexEntityIDs.size(); i++)
    {
        vertexEntityID2Coordinates[vertexEntityIDs[i]]
            = VolVec3(vertexCoordinates[i][0], vertexCoordinates[i][1], vertexCoordinates[i][2]);
    }

    Logger::lout(Logger::INFO) << "Successfully read " << vertexEntityID2Coordinates.size() << " vertices" << std::endl;

    return true;
}

bool Reader::readFiniteElements(
    std::map<const FEType*, std::map<entid_t, std::vector<entid_t>>>& elementType2Elements) const
{
    elementType2Elements.clear();

    int dim = 1;
    for (const vector<const FEType*>& elementTypes : {FEType::all1D, FEType::all2D, FEType::all3D})
    {
        for (const FEType* elemType : elementTypes)
        {
            std::string elementIDPath = elemType->pathToConnectivity(DataType::ELEMENT_IDS);
            std::string nodeIDPath = elemType->pathToConnectivity(DataType::CONNECTED_NODE_IDS);
            if (!m_file.exist(elementIDPath) || !m_file.exist(nodeIDPath))
            {
                continue;
            }

            vector<entid_t> elementIDs;
            vector<vector<entid_t>> nodeIDs;
            try
            {
                elementIDs = H5Easy::load<vector<entid_t>>(m_file, elementIDPath);
                nodeIDs = H5Easy::load<vector<vector<entid_t>>>(m_file, nodeIDPath);
            }
            catch (H5Exception)
            {
                logPathError("HDF5 reading error ", nodeIDPath);
                elementType2Elements.clear();
                return false;
            }

            for (uint i = 0; i < elementIDs.size(); i++)
            {
                elementType2Elements[elemType][elementIDs[i]] = nodeIDs[i];
            }
        }
        dim++;
    }

    if (elementType2Elements.size() == 0)
    {
        logPathError("No finite elements found", Group::CONNECTIVITIES.path());
        return false;
    }

    Logger::lout(Logger::INFO) << "Successfully read " << elementType2Elements.size()
                               << " types of finite elements:" << std::endl;
    for (auto kv : elementType2Elements)
    {
        Logger::lout(Logger::INFO) << "\t " << kv.second.size() << " of type " << kv.first->name << " ("
                                   << kv.first->family.name << ", dim " << kv.first->family.dimension << ")"
                                   << std::endl;
    }

    return true;
}

} // namespace erfh5

} // namespace c2m
