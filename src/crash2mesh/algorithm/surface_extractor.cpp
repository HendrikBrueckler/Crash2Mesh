#include <crash2mesh/algorithm/surface_extractor.hpp>

#include <crash2mesh/util/logger.hpp>

#include <map>
#include <set>

namespace c2m
{
using std::map;
using std::pair;
using std::set;
using std::vector;

bool SurfaceExtractor::FaceCompare::operator()(const vector<Node::Ptr>& a, const vector<Node::Ptr>& b)
{
    if (a.size() < b.size())
        return true;

    if (a.size() > b.size())
        return false;

    set<nodeid_t> nodeSetA;
    set<nodeid_t> nodeSetB;
    for (const Node::Ptr& npA : a)
    {
        nodeSetA.emplace(npA->ID);
    }
    for (const Node::Ptr& npB : b)
    {
        nodeSetB.emplace(npB->ID);
    }

    for (auto itA = nodeSetA.begin(), itB = nodeSetB.begin(); itA != nodeSetA.end() && itB != nodeSetB.end();
         itA++, itB++)
    {
        if (*itA < *itB)
        {
            return true;
        }
        else if (*itA > *itB)
        {
            return false;
        }
    }

    return false;
}

bool SurfaceExtractor::extract(const std::map<partid_t, std::vector<Element3D::Ptr>>& partIDToVolumeElements,
                               std::map<partid_t, std::vector<SurfaceElement::Ptr>>& partIDToSurfaceElements)
{
    static elemid_t maxSurfaceID = 0;

    Logger::lout(Logger::DEBUG) << "Starting surface extraction for " << partIDToVolumeElements.size()
                                << " parts containing 3D elements" << std::endl;

    partIDToSurfaceElements.clear();
    int facesExtracted = 0;
    int facesDiscarded = 0;
    for (auto [partID, volumeElements] : partIDToVolumeElements)
    {
        map<vector<Node::Ptr>, vector<Element3D::Ptr>, FaceCompare> faceToReferencingVolumes;
        for (const Element3D::Ptr& volume : volumeElements)
        {
            for (const vector<int>& faceIndices : volume->type.family.facesCCW)
            {
                vector<Node::Ptr> face;
                for (int nodeIndex : faceIndices)
                {
                    face.emplace_back(volume->nodes[nodeIndex]);
                }
                faceToReferencingVolumes[face].emplace_back(volume);
            }
        }
        for (auto [faceNodes, referencingVolumes] : faceToReferencingVolumes)
        {
            if (referencingVolumes.size() == 1)
            {
                const Element3D::Ptr& volumeElement = referencingVolumes[0];
                const erfh5::FEType* elemType;
                if (faceNodes.size() == 3)
                {
                    elemType = &erfh5::FEType::SURFACE3;
                }
                else if (faceNodes.size() == 4)
                {
                    elemType = &erfh5::FEType::SURFACE4;
                }
                else
                {
                    throw std::logic_error("Unexpected size of 3D element face");
                }
                partIDToSurfaceElements[partID].emplace_back(std::make_shared<SurfaceElement>(
                    ++maxSurfaceID, *elemType, partID, faceNodes, volumeElement));
                facesExtracted++;
            }
            else
            {
                if (referencingVolumes.size() > 2)
                {
                    Logger::lout(Logger::ERROR)
                        << "Found faces being referenced by 3 or more volume elements, check your input!" << std::endl;
                }
                facesDiscarded++;
            }
        }
    }

    Logger::lout(Logger::DEBUG) << "Extracted " << facesExtracted << " surface 2D elements and discarded "
                                << facesDiscarded << " internal faces from 3D elements" << std::endl;

    return true;
}

} // namespace c2m
