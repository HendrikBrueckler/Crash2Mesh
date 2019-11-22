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

void Reader::logPathInfo(Logger::Level severity, const string& msg, const string& path) const
{
    Logger::lout(severity) << msg << " (path \"" << path << "\" of file " << m_file.getName() << ")" << endl;
}

void Reader::logFileInfo(Logger::Level severity, const string& msg) const
{
    Logger::lout(severity) << msg << " (file " << m_file.getName() << ")" << endl;
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

    map<partid_t, vector<SurfaceElement::Ptr>> partIDToSurfaceElements;
    if (!partIDToElements3D.empty())
    {
        if (!SurfaceExtractor::extract(partIDToElements3D, partIDToSurfaceElements))
        {
            logFileInfo(Logger::WARN, "Extracting surfaces from 3D elements failed");
        }
    }

    if (partIDToElements1D.empty() && partIDToElements2D.empty() && partIDToElements3D.empty()
        && partIDToSurfaceElements.empty())
    {

        logFileInfo(Logger::WARN, "No valid parts could be constructed, aborting reading of parts");
        return false;
    }

    if (partIDToElements2D.empty() && partIDToElements3D.empty())
    {
        logFileInfo(Logger::WARN, "No part containing any displayable elements was read, but there were nodes");
    }

    map<partid_t, Part::Ptr> growingParts;
    map<partid_t, set<Node::Ptr>> partToNodes;
    auto createIfNotExists = [&growingParts](partid_t partID) {
        if (growingParts.find(partID) == growingParts.end())
        {
            growingParts[partID] = std::make_shared<Part>(partID);
        }
    };

    for (auto [partID, elements] : partIDToElements1D)
    {
        createIfNotExists(partID);
        growingParts[partID]->elements1D = std::set<Element1D::Ptr>(elements.begin(), elements.end());
        for (const Element1D::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }
    for (auto [partID, elements] : partIDToElements2D)
    {
        createIfNotExists(partID);
        growingParts[partID]->elements2D = std::set<Element2D::Ptr>(elements.begin(), elements.end());
        for (const Element2D::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }
    for (auto [partID, elements] : partIDToSurfaceElements)
    {
        createIfNotExists(partID);
        growingParts[partID]->surfaceElements = std::set<SurfaceElement::Ptr>(elements.begin(), elements.end());
        for (const SurfaceElement::Ptr& elem : elements)
            partToNodes[partID].insert(elem->nodes.begin(), elem->nodes.end());
    }
    // Choosing not to add 3D elements to parts because removing them would be too cumbersome (and will be done anyway)
    // for (auto [partID, elements] : partIDToElements3D)
    // {
    //     createIfNotExists(partID);
    //     growingParts[partID]->elements3D = std::set<Element3D::Ptr>(elements.begin(), elements.end());
    //     for (const Element3D::Ptr& elem: elements)
    //         addNodes(growingParts[partID], elem);
    // }

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
    vector<vector<float>> nodeCoordinates;
    string nodeIDpath(FEType::NODE.pathToResults(ResultType::COORDINATE, DataType::ENTITY_IDS));
    string nodeCoordPath(FEType::NODE.pathToResults(ResultType::COORDINATE, DataType::RESULTS));

    if (!readData(nodeIDpath, nodeIDs) || !readData(nodeCoordPath, nodeCoordinates) || nodeIDs.size() == 0
        || nodeCoordinates.size() != nodeIDs.size())
    {
        logPathInfo(Logger::ERROR,
                    "Could not read node coordinates, aborting reading of nodes",
                    FEType::NODE.pathToResults(ResultType::COORDINATE));
        return false;
    }

    map<nodeid_t, vector<vector<float>>> nodeDisplacements;
    if (!readPerStateResults(FEType::NODE, ResultType::TRANSLATIONAL_DISPLACEMENT, nodeDisplacements))
    {
        logPathInfo(Logger::ERROR,
                    "Could not read node displacements, aborting reading of nodes",
                    FEType::NODE.pathToPerStateResults("stateXXX", ResultType::TRANSLATIONAL_DISPLACEMENT));
        return false;
    }

    for (uint i = 0; i < nodeIDs.size(); i++)
    {
        Vec3 coord(nodeCoordinates[i][0], nodeCoordinates[i][1], nodeCoordinates[i][2]);
        vector<Vec3> displacements;
        for (uint j = 0; j < nodeDisplacements[nodeIDs[i]].size(); j++)
        {
            displacements.emplace_back(nodeDisplacements[nodeIDs[i]][j][0],
                                       nodeDisplacements[nodeIDs[i]][j][1],
                                       nodeDisplacements[nodeIDs[i]][j][2]);
        }
        nodeIDToNode[nodeIDs[i]] = std::make_shared<Node>(nodeIDs[i], coord, displacements);
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

    map<const FEGenericType*, map<elemid_t, vector<float>>> genericTypeToResults;
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
        if (genericTypeToResults.find(genericType) == genericTypeToResults.end())
        {
            if (!readPerStateResults(*elemType, ResultType::PLASTIC_STRAIN, genericTypeToResults[genericType]))
            {
                logPathInfo(Logger::ERROR,
                            "Could not read 2D element plastic strains",
                            elemType->pathToPerStateResults("stateXXX", ResultType::PLASTIC_STRAIN));
                partIDTo2DElements.clear();
                return false;
            }
        }

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

            const vector<float>& plasticStrains = genericTypeToResults[genericType][elementIDs[i]];

            partIDTo2DElements[partIDs[i]].emplace_back(
                std::make_shared<Element2D>(elementIDs[i], *actualElemType, partIDs[i], nodes, plasticStrains));
        }

        logFileInfo(Logger::INFO,
                    "Successfully read " + std::to_string(elementIDs.size()) + " 2D finite elements of type "
                        + elemType->name);
        logFileInfo(Logger::DEBUG,
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

    map<const FEGenericType*, map<elemid_t, vector<float>>> genericTypeToResults;
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
        if (!readPerStateResults(*elemType, ResultType::EQUIVALENT_PLASTIC_STRAIN, genericTypeToResults[genericType]))
        {
            logPathInfo(Logger::ERROR,
                        "Could not read 3D element equivalent plastic strains",
                        elemType->pathToPerStateResults("stateXXX", ResultType::EQUIVALENT_PLASTIC_STRAIN));
            partIDTo3DElements.clear();
            return false;
        }

        for (uint i = 0; i < elementIDs.size(); i++)
        {
            vector<Node::Ptr> nodes;
            for (nodeid_t nodeID : nodeIDs[i])
                nodes.emplace_back(nodeIDToNode.at(nodeID));

            const vector<float>& plasticStrains = genericTypeToResults[genericType][elementIDs[i]];

            partIDTo3DElements[partIDs[i]].emplace_back(
                std::make_shared<Element3D>(elementIDs[i], *elemType, partIDs[i], nodes, plasticStrains));
        }

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

} // namespace erfh5

} // namespace c2m
