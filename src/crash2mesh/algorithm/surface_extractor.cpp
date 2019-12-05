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

bool SurfaceExtractor::extract(std::vector<Part::Ptr>& parts, bool deleteVolumes)
{
    static elemid_t maxSurfaceID = 0;

    int facesExtracted = 0;
    int facesDiscarded = 0;
    for (Part::Ptr& partptr : parts)
    {
        if (partptr->elements3D.empty())
        {
            continue;
        }

        map<vector<Node::Ptr>, vector<Element3D::Ptr>, FaceCompare> faceToReferencingVolumes;
        for (const Element3D::Ptr& volume : partptr->elements3D)
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
                partptr->surfaceElements.emplace(std::make_shared<SurfaceElement>(
                    ++maxSurfaceID, *elemType, partptr->ID, faceNodes, volumeElement));
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
        if (deleteVolumes)
        {
            partptr->elements3D.clear();
        }
    }

    Logger::lout(Logger::INFO) << "Extracted " << facesExtracted << " surface 2D elements and discarded "
                                << facesDiscarded << " internal faces from 3D elements" << std::endl;

    return true;
}

} // namespace c2m
