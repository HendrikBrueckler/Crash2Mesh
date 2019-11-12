#include <crash2mesh/fileio/erf_hdf5_path.hpp>

#include <stdexcept>

namespace c2m
{

std::string ERFHDF5Path::toString(Group group)
{
    switch (group)
    {
    case Group::CSMEXPL:
        return "CSMEXPL";
    case Group::CONSTANT:
        return "constant";
    case Group::SINGLESTATE:
        return "singlestate";
    case Group::CONNECTIVITIES:
        return "connectivities";
    case Group::ENTITYRESULTS:
        return "entityresults";
    case Group::BAR:
        return "BAR";
    case Group::BEAM:
        return "BEAM";
    case Group::HEXA8:
        return "HEXA8";
    case Group::PENTA6:
        return "PENTA6";
    case Group::PLINK:
        return "PLINK";
    case Group::SHELL:
        return "SHELL";
    case Group::NODE:
        return "NODE";
    case Group::SOLID:
        return "SOLID";
    case Group::COORDINATE:
        return "COORDINATE";
    case Group::DISPLACEMENT:
        return "Translational_Displacement";
    case Group::ZONE1_SET0:
        return "ZONE1_set0";
    }

    throw std::logic_error("Missing string mapping for ERFHDF5Path::Group");
}

std::string ERFHDF5Path::toString(Dataset dataset)
{
    switch (dataset)
    {
    case Dataset::NODE_IDS_BY_ELEMENT:
        return "ic";
    case Dataset::ELEMENT_ID_BY_ELEMENT:
        return "idele";
    case Dataset::PART_ID_BY_ELEMENT:
        return "pid";
    case Dataset::ENTITY_ID_BY_ENTITY:
        return "entid";
    case Dataset::RESULT_BY_ENTITY:
        return "res";
    }

    throw std::logic_error("Missing string mapping for ERFHDF5Path::Dataset");
}

ERFHDF5Path& ERFHDF5Path::descend(Group group)
{
    m_path.emplace_back(toString(group));

    return *this;
}

ERFHDF5Path& ERFHDF5Path::ascend()
{
    if (!m_dataset.empty())
        m_dataset = "";

    if (!m_path.empty())
        m_path.pop_back();

    return *this;
}

void ERFHDF5Path::dataset(Dataset dataset)
{
    m_dataset = "erfblock/" + toString(dataset);
}

std::string ERFHDF5Path::path() const
{
    if (m_path.size() == 0)
        return "";

    std::string pathString;
    for (std::string dir : m_path)
        pathString += dir + "/";

    return pathString;
}

std::string ERFHDF5Path::pathToData() const
{
    return path() + m_dataset;
}

} // namespace c2m
