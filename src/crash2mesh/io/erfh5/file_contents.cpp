#include <crash2mesh/io/erfh5/file_contents.hpp>
#include <regex>

namespace c2m
{
namespace erfh5
{
using std::vector;
static const std::string STATE_TEMPLATE_STRING = "%<STATE_TEMPLATE>%";

Group::Group(const Group* _parent, const std::string& _name) : parent(_parent), name(_name)
{
    if (parent == nullptr)
        m_path = name;
    else
        m_path = parent->path() + "/" + name;
}
std::string Group::path() const
{
    return m_path;
}

const Group Group::CSMEXPL(nullptr, "CSMEXPL");
const Group Group::CONSTANT(&Group::CSMEXPL, "constant");
const Group Group::CONNECTIVITIES(&Group::CONSTANT, "connectivities");
const Group Group::ENTITYRESULTS_CONSTANT(&Group::CONSTANT, "entityresults");
const Group Group::SINGLESTATE(&Group::CSMEXPL, "singlestate");
const Group Group::STATE_TEMPLATE(&Group::SINGLESTATE, STATE_TEMPLATE_STRING);
const Group Group::ENTITYRESULTS_PER_STATE(&Group::STATE_TEMPLATE, "entityresults");

ResultType::ResultType(const std::string& _name) : name(_name)
{
}

const ResultType ResultType::DOMAIN("Domain");
const ResultType ResultType::COORDINATE("COORDINATE");
const ResultType ResultType::TRANSLATIONAL_DISPLACEMENT("Translational_Displacement");
const ResultType ResultType::PLASTIC_STRAIN("Membrane_Plastic_Strain");
const ResultType ResultType::MIN_PLASTIC_STRAIN("Min_Plastic_Strain");
const ResultType ResultType::MAX_PLASTIC_STRAIN("Max_Plastic_Strain");
const ResultType ResultType::THICKNESS("THICKNESS");
const ResultType ResultType::EQUIVALENT_PLASTIC_STRAIN("EPLE");

DataType::DataType(const std::string& _name) : name(_name)
{
}

const DataType DataType::CONNECTED_NODE_IDS("ic");
const DataType DataType::ELEMENT_IDS("idele");
const DataType DataType::PART_IDS("pid");
const DataType DataType::ENTITY_IDS("entid");
const DataType DataType::RESULTS("res");
const DataType DataType::STATE_INDEX("indexident");
const DataType DataType::STATE_VALUE("indexval");

FEFamily::FEFamily(const std::string _name, int _dim, int _corners, const std::vector<std::vector<int>> _faces)
    : name(_name), dimension(_dim), numCorners(_corners), facesCCW(_faces)
{
}

const FEFamily FEFamily::POINT("point", 0, 1, {});
const FEFamily FEFamily::LINE2("line2", 1, 2, {});
const FEFamily FEFamily::LINE3("line3", 1, 3, {});
const FEFamily FEFamily::TRIA3("tria3", 2, 3, {{0, 1, 2}});
const FEFamily FEFamily::TRIA6("tria6", 2, 3, {{0, 1, 2}});
// VERY DANGEROUS, QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==ID_NULL
const FEFamily FEFamily::QUAD4("quad4", 2, 4, {{0, 1, 2, 3}});
const FEFamily FEFamily::QUAD8("quad6", 2, 4, {{0, 1, 2, 3}});
const FEFamily FEFamily::TETRA4("tetra4", 3, 4, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}});
const FEFamily FEFamily::TETRA10("tetra10", 3, 4, {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}});
const FEFamily FEFamily::HEXA8("hexa8",
                               3,
                               8,
                               {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}});
const FEFamily FEFamily::HEXA20("hexa20",
                                3,
                                8,
                                {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}});
const FEFamily FEFamily::PENTA6("penta6", 3, 6, {{0, 2, 1}, {3, 4, 5}, {0, 3, 5, 2}, {2, 5, 4, 1}, {1, 4, 3, 0}});
const FEFamily FEFamily::PENTA15("penta15", 3, 6, {{0, 2, 1}, {3, 4, 5}, {0, 3, 5, 2}, {2, 5, 4, 1}, {1, 4, 3, 0}});
const FEFamily FEFamily::PYRAMID5("pyramid5", 3, 5, {{0, 3, 2, 1}, {0, 1, 4}, {1, 2, 4}, {3, 0, 4}});
const FEFamily FEFamily::PYRAMID13("pyramid13", 3, 5, {{0, 3, 2, 1}, {0, 1, 4}, {1, 2, 4}, {3, 0, 4}});

FEGenericType::FEGenericType(const std::string& _name) : name(_name)
{
}

const FEGenericType FEGenericType::NODE("NODE");
const FEGenericType FEGenericType::BEAM("BEAM");
const FEGenericType FEGenericType::BAR("BAR");
const FEGenericType FEGenericType::SPRING("SPRING");
const FEGenericType FEGenericType::JOINT("JOINT");
const FEGenericType FEGenericType::PLINK("PLINK");
const FEGenericType FEGenericType::DRAWBEAD("DRAWBEAD");
const FEGenericType FEGenericType::MUSCLE("MUSCLE");
const FEGenericType FEGenericType::JET("JET");
const FEGenericType FEGenericType::GAP("GAP");
const FEGenericType FEGenericType::TIED("TIED");
const FEGenericType FEGenericType::MEMBR("MEMBR");
const FEGenericType FEGenericType::SHELL("SHELL");
const FEGenericType FEGenericType::SOLID("SOLID");
const FEGenericType FEGenericType::PART("PART");

FEType::FEType(const std::string& _name, const FEFamily& _family, const FEGenericType& _genericType)
    : name(_name), family(_family), genericType(_genericType)
{
}

std::string FEType::pathToConnectivity() const
{
    return Group::CONNECTIVITIES.path() + "/" + name;
}

std::string FEType::pathToConnectivity(const DataType& dataType) const
{
    return pathToConnectivity() + "/erfblock/" + dataType.name;
}

std::string FEType::pathToResults(const ResultType& resultType) const
{
    return Group::ENTITYRESULTS_CONSTANT.path() + "/" + genericType.name + "/" + resultType.name;
}

std::string FEType::pathToResults(const ResultType& resultType, const DataType& dataType) const
{
    return pathToResults(resultType) + "/ZONE1_set0/erfblock/" + dataType.name;
}

std::string
FEType::pathToPerStateResults(const std::string& state, const ResultType& resultType) const
{
    std::string templatedPath = Group::ENTITYRESULTS_PER_STATE.path() + "/" + genericType.name + "/" + resultType.name;
    return std::regex_replace(templatedPath, std::regex(STATE_TEMPLATE_STRING), state);
}

std::string
FEType::pathToPerStateResults(const std::string& state, const ResultType& resultType, const DataType& dataType) const
{
    std::string templatedPath = Group::ENTITYRESULTS_PER_STATE.path() + "/" + genericType.name + "/" + resultType.name
                                + "/ZONE1_set1/erfblock/" + dataType.name;
    return pathToPerStateResults(state, resultType) + "/ZONE1_set1/erfblock/" + dataType.name;;
}

// 0D
const FEType FEType::NODE("NODE", FEFamily::POINT, FEGenericType::NODE);
// 1D
const FEType FEType::BEAM("BEAM", FEFamily::LINE2, FEGenericType::BEAM);
const FEType FEType::BAR("BAR", FEFamily::LINE2, FEGenericType::BAR);
const FEType FEType::SPRING6DOF("SPRING6DOF", FEFamily::LINE2, FEGenericType::SPRING);
const FEType FEType::SPHERICALJOINT("SPHERICALJOINT", FEFamily::LINE2, FEGenericType::JOINT);
const FEType FEType::FLEXTORSJOINT("FLEXTORSJOINT", FEFamily::LINE2, FEGenericType::JOINT);
const FEType FEType::KJOINT("KJOINT", FEFamily::LINE2, FEGenericType::JOINT);
const FEType FEType::PLINK("PLINK", FEFamily::LINE2, FEGenericType::PLINK);
const FEType FEType::MBSJOINT("MBSJOINT", FEFamily::LINE2, FEGenericType::JOINT);
const FEType FEType::MBSSPRING("MBSSPRING", FEFamily::LINE2, FEGenericType::SPRING);
const FEType FEType::SPRINGBEAM("SPRINGBEAM", FEFamily::LINE2, FEGenericType::SPRING);
const FEType FEType::DRAWBEAD("DRAWBEAD", FEFamily::LINE2, FEGenericType::DRAWBEAD);
const FEType FEType::MTOJN("MTOJN", FEFamily::LINE2, FEGenericType::JOINT);
const FEType FEType::MUSCLE("MUSCLE", FEFamily::LINE2, FEGenericType::MUSCLE);
const FEType FEType::JET("JET", FEFamily::LINE2, FEGenericType::JET);
const FEType FEType::MPCPLINK("MPCPLINK", FEFamily::LINE2, FEGenericType::PLINK);
const FEType FEType::GAP("GAP", FEFamily::LINE2, FEGenericType::GAP);
const FEType FEType::TIED("TIED", FEFamily::LINE2, FEGenericType::TIED);
// 2D
// QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==0
const FEType FEType::MEMBR("MEMBR", FEFamily::QUAD4, FEGenericType::SHELL);
const FEType FEType::THICKSHELL("THICKSHELL", FEFamily::QUAD4, FEGenericType::SHELL);
const FEType FEType::SHELL("SHELL", FEFamily::QUAD4, FEGenericType::SHELL);
const FEType FEType::SHEL6("SHEL6", FEFamily::TRIA6, FEGenericType::SHELL);
const FEType FEType::SHEL8("SHEL8", FEFamily::QUAD8, FEGenericType::SHELL);
// SHEL3 isnt in spec, replaces QUAD4-Shells that are actually triangles
const FEType FEType::SHEL3("SHEL3", FEFamily::TRIA3, FEGenericType::SHELL);
// SHEL3 isnt in spec, replaces QUAD4-Shells that are actually triangles
const FEType FEType::SURFACE3("SURFACE3", FEFamily::TRIA3, FEGenericType::SOLID); // GenericType shouldnt really matter
// SHEL3 isnt in spec, replaces QUAD4-Shells that are actually triangles
const FEType FEType::SURFACE4("SURFACE4", FEFamily::QUAD4, FEGenericType::SOLID); // GenericType shouldnt really matter
// 3D
const FEType FEType::HEXA8("HEXA8", FEFamily::HEXA8, FEGenericType::SOLID);
const FEType FEType::BRICKSHELL("BRICKSHELL", FEFamily::HEXA8, FEGenericType::SOLID);
const FEType FEType::TETRA10("TETRA10", FEFamily::TETRA10, FEGenericType::SOLID);
const FEType FEType::TETRA4("TETRA4", FEFamily::TETRA4, FEGenericType::SOLID);
const FEType FEType::PENTA6("PENTA6", FEFamily::PENTA6, FEGenericType::SOLID);
const FEType FEType::PENTA15("PENTA15", FEFamily::PENTA15, FEGenericType::SOLID);
const FEType FEType::HEXA20("HEXA20", FEFamily::HEXA20, FEGenericType::SOLID);
} // namespace erfh5
} // namespace c2m
