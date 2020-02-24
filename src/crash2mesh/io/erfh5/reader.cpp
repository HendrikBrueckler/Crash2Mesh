#include <crash2mesh/io/erfh5/reader.hpp>

#include <crash2mesh/algorithm/surface_extractor.hpp>
#include <crash2mesh/io/erfh5/file_contents.hpp>

#include <highfive/H5Easy.hpp>

#include <map>
#include <memory>
#include <set>
#include <stdexcept>

namespace c2m
{
namespace erfh5
{
using std::endl;
using std::map;
using std::pair;
using std::set;
using std::string;
using std::vector;

Reader::Reader(const std::string& filename) : m_file(filename)
{
}

void Reader::logFileInfo(Logger::Level severity, const string& msg, const string& path) const
{
    Logger::lout(severity) << msg << " (path \"" << path << "\" of file " << m_file.getName() << ")" << endl;
}

void Reader::logFileInfo(Logger::Level severity, const string& msg) const
{
    Logger::lout(severity) << msg << " (file " << m_file.getName() << ")" << endl;
}

size_t Reader::getNumStates() const
{
    if (m_file.exist(Group::SINGLESTATE.path()))
    {
        return m_file.getGroup(Group::SINGLESTATE.path()).getNumberObjects();
    }
    return 0;
}

bool Reader::readParts(vector<Part::Ptr>& parts) const
{
    parts.clear();
    if (!m_file.isValid())
    {
        logFileInfo(Logger::ERROR, "Invalid file, aborting reading of parts");
        return false;
    }

    map<nodeid_t, Node::Ptr> nodes;
    if (!readNodes(nodes))
    {
        logFileInfo(Logger::ERROR, "Could not read any nodes, aborting reading of parts");
        return false;
    }

    map<partid_t, vector<Element1D::Ptr>> partIDToElements1D;
    if (!read1DElements(nodes, partIDToElements1D))
    {
        logFileInfo(Logger::ERROR, "Error reading 1D elements");
        return false;
    }

    map<partid_t, vector<Element2D::Ptr>> partIDToElements2D;
    if (!read2DElements(nodes, partIDToElements2D))
    {
        logFileInfo(Logger::ERROR, "Error reading 2D elements");
        return false;
    }

    map<partid_t, vector<Element3D::Ptr>> partIDToElements3D;
    if (!read3DElements(nodes, partIDToElements3D))
    {
        logFileInfo(Logger::ERROR, "Error reading 3D elements");
        return false;
    }

    if (partIDToElements1D.empty() && partIDToElements2D.empty() && partIDToElements3D.empty())
    {
        logFileInfo(Logger::WARN, "No valid parts could be constructed, aborting reading of parts");
        return false;
    }

    if (partIDToElements2D.empty() && partIDToElements3D.empty())
    {
        logFileInfo(Logger::WARN, "No part containing any displayable elements was read, but there were nodes");
    }

    map<entid_t, entid_t> partIDToPartUID;
    if (!readIdentifiers(FEGenericType::PART, partIDToPartUID))
    {
        logFileInfo(Logger::WARN, "Error reading part identifiers, some or all parts will be assigned user id 0!");
    }

    map<partid_t, Part::Ptr> growingParts;
    map<partid_t, set<Node::Ptr>> partToNodes;
    auto createIfNotExists = [&growingParts, &partIDToPartUID](partid_t partID) {
        if (growingParts.find(partID) == growingParts.end())
        {
            growingParts[partID] = std::make_shared<Part>(partID, partIDToPartUID[partID]);
        }
    };

    for (auto [partID, elements] : partIDToElements1D)
    {
        // TODO don't completely remove parts but handle differently in decimation!
        // TODO collect these magic numbers in collectors.hpp as static vars
        // if (partIDToPartUID[partID] >= 9600000 && partIDToPartUID[partID] < 9980000)
        //     continue;
        createIfNotExists(partID);
        growingParts[partID]->elements1D = std::set<Element1D::Ptr>(elements.begin(), elements.end());
        for (const Element1D::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }
    for (auto [partID, elements] : partIDToElements2D)
    {
        // TODO don't completely remove parts but handle differently in decimation!
        // TODO collect these magic numbers in collectors.hpp as static vars
        // if (partIDToPartUID[partID] >= 9600000 && partIDToPartUID[partID] < 9980000)
        //     continue;
        createIfNotExists(partID);
        growingParts[partID]->elements2D = std::set<Element2D::Ptr>(elements.begin(), elements.end());
        for (const Element2D::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }
    for (auto [partID, elements] : partIDToElements3D)
    {
        // TODO don't completely remove parts but handle differently in decimation!
        // TODO collect these magic numbers in collectors.hpp as static vars
        // if (partIDToPartUID[partID] >= 9600000 && partIDToPartUID[partID] < 9980000)
        //     continue;
        createIfNotExists(partID);
        growingParts[partID]->elements3D = std::set<Element3D::Ptr>(elements.begin(), elements.end());
        for (const Element3D::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }

    for (auto part : growingParts)
        parts.emplace_back(part.second);

    for (auto nodesPerPart : partToNodes)
        for (const Node::Ptr& nodeptr : nodesPerPart.second)
            nodeptr->referencingParts++;

    logFileInfo(Logger::INFO, "Successfully read " + std::to_string(parts.size()) + " parts");
    return true;
}

bool Reader::readNodes(map<nodeid_t, Node::Ptr>& nodeIDToNode) const
{
    nodeIDToNode.clear();
    if (!m_file.isValid())
    {
        logFileInfo(Logger::ERROR, "Invalid file, aborting reading of nodes");
        return false;
    }

    vector<nodeid_t> nodeIDs;
    MatX3 nodeCoordinates;
    string nodeIDpath(FEType::NODE.pathToResults(ResultType::COORDINATE, DataType::ENTITY_IDS));
    string nodeCoordPath(FEType::NODE.pathToResults(ResultType::COORDINATE, DataType::RESULTS));

    Logger::lout(Logger::DEBUG) << "Reading node ids and positions..." << std::endl;
    if (!readData(nodeIDpath, nodeIDs) || !readData(nodeCoordPath, nodeCoordinates) || nodeIDs.size() == 0
        || nodeCoordinates.rows() != nodeIDs.size())
    {
        logFileInfo(Logger::ERROR,
                    "Could not read node coordinates, aborting reading of nodes",
                    FEType::NODE.pathToResults(ResultType::COORDINATE));
        return false;
    }

    Logger::lout(Logger::DEBUG) << "Finished reading node ids and positions, now reading displacements..." << std::endl;

    size_t numStates = getNumStates();
    map<nodeid_t, MatX3> nodeDisplacements;
    if (numStates > 0 && !readPerStateResults(FEType::NODE, ResultType::DISPLACEMENT, nodeDisplacements))
    {
        logFileInfo(Logger::ERROR,
                    "Could not read node displacements, aborting reading of nodes",
                    FEType::NODE.pathToPerStateResults("stateXXX", ResultType::DISPLACEMENT));
        return false;
    }

    Logger::lout(Logger::DEBUG) << "Finished reading displacements, now generating c2m::Nodes from data..." << std::endl;

    for (uint i = 0; i < nodeIDs.size(); i++)
    {
        MatX3 positions = nodeDisplacements[nodeIDs[i]].rowwise() + nodeCoordinates.row(i);
        nodeIDToNode[nodeIDs[i]] = std::make_shared<Node>(nodeIDs[i], positions);
    }

    logFileInfo(Logger::INFO, "Successfully read " + std::to_string(nodeIDToNode.size()) + " vertices");

    return true;
}

bool Reader::read1DElements(const map<nodeid_t, Node::Ptr>& nodeIDToNode,
                            map<partid_t, vector<Element1D::Ptr>>& partIDTo1DElements) const
{
    partIDTo1DElements.clear();
    if (!m_file.isValid() || nodeIDToNode.empty())
    {
        logFileInfo(Logger::ERROR, "Invalid file, aborting reading of 1D elements");
        return false;
    }

    for (const FEType* elemType : Element1D::allTypes)
    {
        vector<elemid_t> elementIDs;
        vector<partid_t> partIDs;
        vector<vector<nodeid_t>> nodeIDs;

        if (!readConnectivities(elemType, elementIDs, partIDs, nodeIDs))
        {
            continue;
        }

        for (uint i = 0; i < elementIDs.size(); i++)
        {
            vector<Node::Ptr> nodes;
            for (nodeid_t nodeID : nodeIDs[i])
                nodes.emplace_back(nodeIDToNode.at(nodeID));

            partIDTo1DElements[partIDs[i]].emplace_back(
                std::make_shared<Element1D>(elementIDs[i], *elemType, partIDs[i], nodes));
        }

        logFileInfo(Logger::INFO,
                    "Successfully read " + std::to_string(elementIDs.size()) + " 1D finite elements of type "
                        + elemType->name);
    }

    if (partIDTo1DElements.size() == 0)
    {
        logFileInfo(Logger::INFO, "No 1D elements found");
    }

    return true;
}

bool Reader::read2DElements(const map<nodeid_t, Node::Ptr>& nodeIDToNode,
                            map<partid_t, vector<Element2D::Ptr>>& partIDTo2DElements) const
{
    partIDTo2DElements.clear();
    if (!m_file.isValid() || nodeIDToNode.empty())
    {
        logFileInfo(Logger::ERROR, "Invalid file, aborting reading of 2D elements");
        return false;
    }

    size_t numStates = getNumStates();

    map<const FEGenericType*, map<elemid_t, VecX>> genTypeToPlasticStrains;
    for (const FEType* elemType : Element2D::allTypes)
    {
        vector<elemid_t> elementIDs;
        vector<partid_t> partIDs;
        vector<vector<nodeid_t>> nodeIDs;

        if (!readConnectivities(elemType, elementIDs, partIDs, nodeIDs))
        {
            continue;
        }

        const FEGenericType* genericType(&elemType->genericType);
        if (genTypeToPlasticStrains.find(genericType) == genTypeToPlasticStrains.end())
        {
            if (numStates > 0
                && !readPerStateResults(*elemType, ResultType::PLASTIC_STRAIN, genTypeToPlasticStrains[genericType]))
            {
                logFileInfo(Logger::ERROR,
                            "Could not read 2D element plastic strains",
                            elemType->pathToPerStateResults("stateXXX", ResultType::PLASTIC_STRAIN));
                partIDTo2DElements.clear();
                return false;
            }
        }

        int missingStrains = 0;
        elemid_t elementsConverted = 0;
        for (uint i = 0; i < elementIDs.size(); i++)
        {
            // According to ESI ERF-HDF5 spec, quad4s can be tria3s, if NODE3==NODE4 || NODE4==ID_NULL
            const FEType* actualElemType = elemType;
            if (&elemType->family == &FEFamily::QUAD4)
            {
                if (nodeIDs[i].size() != 4)
                    throw std::logic_error("Quad4 does not reference 4 nodes!");

                if (nodeIDs[i][3] == ID_NULL || nodeIDs[i][3] == nodeIDs[i][2])
                {
                    elementsConverted++;
                    nodeIDs[i].erase(nodeIDs[i].end() - 1);
                    actualElemType = &FEType::SHEL3;
                }
            }
            // Remove higher order nodes
            int numCorners = actualElemType->family.numCorners;
            nodeIDs[i].erase(nodeIDs[i].begin() + numCorners, nodeIDs[i].end());

            vector<Node::Ptr> nodes;
            for (nodeid_t nodeID : nodeIDs[i])
            {
                nodes.emplace_back(nodeIDToNode.at(nodeID));
            }

            VecX& plasticStrains(genTypeToPlasticStrains[genericType][elementIDs[i]]);
            if (plasticStrains.size() == 0)
            {
                missingStrains++;
                plasticStrains = VecX::Zero(numStates);
            }
            partIDTo2DElements[partIDs[i]].emplace_back(
                std::make_shared<Element2D>(elementIDs[i], *actualElemType, partIDs[i], nodes, plasticStrains));
        }

        if (missingStrains != 0)
            logFileInfo(Logger::WARN,
                        "Missing plastic strains for " + std::to_string(missingStrains) + " elements",
                        elemType->pathToPerStateResults("stateXXX", ResultType::PLASTIC_STRAIN));

        logFileInfo(Logger::INFO,
                    "Successfully read " + std::to_string(elementIDs.size()) + " 2D finite elements of type "
                        + elemType->name);
        logFileInfo(Logger::INFO,
                    "Had to convert " + std::to_string(elementsConverted) + " of " + elemType->name + "s to "
                        + FEType::SHEL3.name + ", because they were actually triangles, not quads");
    }

    if (partIDTo2DElements.size() == 0)
    {
        logFileInfo(Logger::INFO, "No 2D elements found");
    }

    return true;
}

bool Reader::read3DElements(const map<nodeid_t, Node::Ptr>& nodeIDToNode,
                            map<partid_t, vector<Element3D::Ptr>>& partIDTo3DElements) const
{
    partIDTo3DElements.clear();
    if (!m_file.isValid() || nodeIDToNode.empty())
    {
        logFileInfo(Logger::ERROR, "Invalid file, aborting reading of 3D elements");
        return false;
    }

    uint numStates = getNumStates();

    map<const FEGenericType*, map<elemid_t, VecX>> genTypeToPlasticStrains;
    for (const FEType* elemType : Element3D::allTypes)
    {
        vector<elemid_t> elementIDs;
        vector<partid_t> partIDs;
        vector<vector<nodeid_t>> nodeIDs;

        if (!readConnectivities(elemType, elementIDs, partIDs, nodeIDs))
        {
            continue;
        }

        const FEGenericType* genericType(&elemType->genericType);
        if (genTypeToPlasticStrains.find(genericType) == genTypeToPlasticStrains.end())
        {
            if (numStates > 0
                && !readPerStateResults(
                       *elemType, ResultType::EQUIVALENT_PLASTIC_STRAIN, genTypeToPlasticStrains[genericType]))
            {
                logFileInfo(Logger::ERROR,
                            "Could not read 3D element equivalent plastic strains",
                            elemType->pathToPerStateResults("stateXXX", ResultType::EQUIVALENT_PLASTIC_STRAIN));
                partIDTo3DElements.clear();
                return false;
            }
        }

        int missingStrains = 0;
        for (uint i = 0; i < elementIDs.size(); i++)
        {
            vector<Node::Ptr> nodes;
            for (nodeid_t nodeID : nodeIDs[i])
                nodes.emplace_back(nodeIDToNode.at(nodeID));

            VecX& plasticStrains(genTypeToPlasticStrains[genericType][elementIDs[i]]);
            if (plasticStrains.size() == 0)
            {
                missingStrains++;
                plasticStrains = VecX::Zero(numStates);
            }
            partIDTo3DElements[partIDs[i]].emplace_back(
                std::make_shared<Element3D>(elementIDs[i], *elemType, partIDs[i], nodes, plasticStrains));
        }

        if (missingStrains != 0)
            logFileInfo(Logger::WARN,
                        "Missing plastic strains for " + std::to_string(missingStrains) + " elements",
                        elemType->pathToPerStateResults("stateXXX", ResultType::PLASTIC_STRAIN));

        logFileInfo(Logger::INFO,
                    "Successfully read " + std::to_string(elementIDs.size()) + " 3D finite elements of type "
                        + elemType->name);
    }

    if (partIDTo3DElements.size() == 0)
    {
        logFileInfo(Logger::INFO, "No 3D elements found");
    }

    return true;
}

bool Reader::readConnectivities(const FEType* elemType,
                                vector<elemid_t>& elementIDs,
                                vector<partid_t>& partIDs,
                                vector<vector<nodeid_t>>& nodeIDs) const
{
    string elementIDPath = elemType->pathToConnectivity(DataType::ELEMENT_IDS);
    string partIDPath = elemType->pathToConnectivity(DataType::PART_IDS);
    string nodeIDPath = elemType->pathToConnectivity(DataType::CONNECTED_NODE_IDS);

    if (!readData(elementIDPath, elementIDs) || !readData(partIDPath, partIDs) || !readData(nodeIDPath, nodeIDs)
        || elementIDs.size() == 0 || partIDs.size() != elementIDs.size() || nodeIDs.size() != elementIDs.size())
    {
        return false;
    }

    return true;
}

bool Reader::readIdentifiers(const FEGenericType& feGenType, std::map<entid_t, entid_t>& id2userid) const
{
    id2userid.clear();

    std::vector<entid_t> entityIDs;
    std::vector<vector<entid_t>> userIDs;

    std::string entityIDPath(feGenType.pathToIdentifiers(DataType::ENTITY_IDS));
    std::string userIDPath(feGenType.pathToIdentifiers(DataType::USER_IDS));

    if (!readData(entityIDPath, entityIDs) || !readData(userIDPath, userIDs) || entityIDs.size() == 0
        || userIDs.size() != entityIDs.size())
    {
        logFileInfo(Logger::ERROR, "Could not read identifiers", feGenType.pathToIdentifiers());
        return false;
    }

    for (size_t i = 0; i < entityIDs.size(); i++)
    {
        id2userid[entityIDs[i]] = userIDs[i].front();
    }

    return true;
}

} // namespace erfh5

} // namespace c2m
