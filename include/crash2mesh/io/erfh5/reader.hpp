#ifndef C2M_ERFH5_READER_HPP
#define C2M_ERFH5_READER_HPP

#include <crash2mesh/core/collectors.hpp>
#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/core/types.hpp>
#include <crash2mesh/io/erfh5/file_contents.hpp>
#include <crash2mesh/util/logger.hpp>
#include <highfive/H5Easy.hpp>

#include <map>

namespace c2m
{
namespace erfh5
{

class Reader
{
  public:
    /**
     * @brief Create a reader that reads from a given ERF-HDF5 file
     *
     * @param filename name of file to read from
     */
    Reader(const std::string& filename);

    size_t getNumStates() const;

    /**
     * @brief Reads all parts separately
     *
     * @param parts read parts
     * @return true if successful
     * @return false else
     */
    bool readParts(std::vector<Part::Ptr>& parts) const;

    /**
     * @brief Read all nodes from an ERF-HDF5 file
     *
     * @param nodeIDToNode map from a nodes identifier to a shared node object
     * @return true if successful
     * @return false else
     */
    bool readNodes(std::map<nodeid_t, Node::Ptr>& nodeIDToNode) const;

    /**
     * @brief Read all 1D elements from an ERF-HDF5 file, given the node information from that file (required for direct
     *       association)
     *
     * @param nodeIDToNode node information from this file
     * @param partIDTo1DElements map from part identifiers to all 1D elements within that part
     * @return true if successful
     * @return false else
     */
    bool read1DElements(const std::map<nodeid_t, Node::Ptr>& nodeIDToNode,
                        std::map<partid_t, std::vector<Element1D::Ptr>>& partIDTo1DElements) const;

    /**
     * @brief Read all 2D elements from an ERF-HDF5 file, given the node information from that file (required for direct
     *       association)
     *
     * @param nodeIDToNode node information from this file
     * @param partIDTo2DElements map from part identifiers to all 2D elements within that part
     * @return true if successful
     * @return false else
     */
    bool read2DElements(const std::map<nodeid_t, Node::Ptr>& nodeIDToNode,
                        std::map<partid_t, std::vector<Element2D::Ptr>>& partIDTo2DElements) const;

    /**
     * @brief Read all 3D elements from an ERF-HDF5 file, given the node information from that file (required for direct
     *       association)
     *
     * @param nodeIDToNode node information from this file
     * @param partIDTo3DElements map from part identifiers to all 3D elements within that part
     * @return true if successful
     * @return false else
     */
    bool read3DElements(const std::map<nodeid_t, Node::Ptr>& nodeID2Node,
                        std::map<partid_t, std::vector<Element3D::Ptr>>& partIDTo3DElements) const;

    /**
     * @brief Read the mapping from internal entity id to predefined user id for a generic finite element type
     *
     * @param feGenType generic finite element type
     * @param id2userid mapping from internal entity id to predefined user id
     * @return true if successful
     * @return false else
     */
    bool readIdentifiers(const FEGenericType& feGenType, std::map<entid_t, entid_t>& id2userid) const;

  private:
    /**
     * @brief Auxiliary function to read all state-dependent results for a given finite element type and result type
     *
     * @tparam ID_T identifier c++ datatype
     * @tparam RESULT_T result c++ datatype
     * @param elemType finite element type to query results for
     * @param resultType result type
     * @param entityIDToAllResults the read result data for each entity
     * @return true if successful
     * @return false else
     */
    template <typename ID_T, typename Scalar, int DIM>
    bool readPerStateResults(const FEType& elemType,
                             const ResultType& resultType,
                             std::map<ID_T, Mat<Scalar, Dynamic, DIM>>& entityIDToAllResults) const
    {
        std::vector<std::string> states = {};
        if (m_file.exist(Group::SINGLESTATE.path()))
        {
            HighFive::Group singlestateGroup = m_file.getGroup(Group::SINGLESTATE.path());
            states = singlestateGroup.listObjectNames();
        }

        std::vector<ID_T> entityIDs;
        Mat<Scalar, Dynamic, DIM> results = Mat<Scalar, Dynamic, DIM>::Zero(entityIDs.size(), DIM);
        std::vector<Mat<Scalar, Dynamic, DIM>> perStateResults(entityIDs.size(),
                                                               Mat<Scalar, Dynamic, DIM>::Zero(states.size(), DIM));
        for (size_t stateIndex = 0; stateIndex < states.size(); stateIndex++)
        {
            std::string state = states[stateIndex];
            std::string entityIDPath(elemType.pathToPerStateResults(state, resultType, DataType::ENTITY_IDS));
            std::string resultPath(elemType.pathToPerStateResults(state, resultType, DataType::RESULTS));

            if ((entityIDs.empty() && !readData(entityIDPath, entityIDs)) || !readData(resultPath, results)
                || entityIDs.empty() || results.rows() != static_cast<long>(entityIDs.size()))
            {
                logFileInfo(Logger::ERROR,
                            "Could not read per-state results",
                            elemType.pathToPerStateResults(state, resultType));
                return false;
            }
            if (perStateResults.empty())
            {
                perStateResults = std::vector<Mat<Scalar, Dynamic, DIM>>(
                    entityIDs.size(), Mat<Scalar, Dynamic, DIM>::Zero(states.size(), DIM));
            }
            for (uint i = 0; i < entityIDs.size(); i++)
            {
                perStateResults[i].row(stateIndex) = results.row(i);
            }

            // TODO test on big datasets if entityIDs are really the same for all states!!!
            // entityIDs.clear();
            // results.resize(0, DIM);
        }

        for (uint i = 0; i < entityIDs.size(); i++)
        {
            entityIDToAllResults[entityIDs[i]] = perStateResults[i];
        }

        return true;
    }

    /**
     * @brief Auxiliary function for exception-safe simple reading of any data from
     *        a path within this reader's file
     *
     * @tparam DATA c++ type to read into
     * @param path path to read from
     * @param data the read data
     * @return true if successful
     * @return false else
     */
    template <typename DATA> bool readData(const std::string& path, DATA& data) const
    {
        if (m_file.exist(path))
        {
            try
            {
                data = H5Easy::load<DATA>(m_file, path);
                return true;
            }
            catch (H5Exception& e)
            {
                logFileInfo(Logger::ERROR, "HDF5 reading error ", path);
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Auxiliary function to read all connectivities of a given finite element type
     *
     * @param elemType finite element type for which to read connectivities
     * @param elementIDs read element identifiers
     * @param partIDs read part identifiers
     * @param nodeIDs read connected node identifiers
     * @return true if successful
     * @return false else
     */
    bool readConnectivities(const FEType* elemType,
                            std::vector<elemid_t>& elementIDs,
                            std::vector<partid_t>& partIDs,
                            std::vector<std::vector<nodeid_t>>& nodeIDs) const;

    /**
     * @brief Auxiliary function to log info including the filename and path within the file
     *
     * @param severity message severity
     * @param msg message
     * @param path path within the file
     */
    void logFileInfo(Logger::Level severity, const std::string& msg, const std::string& path) const;

    /**
     * @brief Auxiliary function to log info including the filename
     *
     * @param severity message severity
     * @param msg message
     */
    void logFileInfo(Logger::Level severity, const std::string& msg) const;

    H5File m_file; //!< ERF-HDF5 file handle
};

} // namespace erfh5
} // namespace c2m

#endif
