#ifndef C2M_ERFH5_FILECONTENTS_HPP
#define C2M_ERFH5_FILECONTENTS_HPP

#include <string>
#include <vector>

namespace c2m
{
namespace erfh5
{

class Group
{
  public:
    const static Group CSMEXPL;
    const static Group CONSTANT;
    const static Group CONNECTIVITIES;
    const static Group ENTITYRESULTS_CONSTANT;
    const static Group SINGLESTATE;
    const static Group STATE_TEMPLATE;
    const static Group ENTITYRESULTS_PER_STATE;

    const Group* const parent;
    const std::string name;
    std::string path() const;

  private:
    std::string m_path;
    Group(const Group* _parent, const std::string& _name);
};

class ResultType
{
  public:
    std::string name;

    const static ResultType DOMAIN;
    const static ResultType COORDINATE;
    const static ResultType TRANSLATIONAL_DISPLACEMENT;
    const static ResultType PLASTIC_STRAIN;
    const static ResultType MIN_PLASTIC_STRAIN;
    const static ResultType MAX_PLASTIC_STRAIN;
    const static ResultType THICKNESS;
    const static ResultType EQUIVALENT_PLASTIC_STRAIN;

  private:
    ResultType(const std::string& _name);
};

class DataType
{
  public:
    std::string name;

    const static DataType CONNECTED_NODE_IDS;
    const static DataType ELEMENT_IDS;
    const static DataType PART_IDS;
    const static DataType ENTITY_IDS;
    const static DataType RESULTS;
    const static DataType STATE_INDEX;
    const static DataType STATE_VALUE;

  private:
    DataType(const std::string& _name);
};

class FEFamily
{
  public:
    const std::string name;
    const int dimension;
    const int numCorners;

    const static FEFamily POINT;
    const static FEFamily LINE2;
    const static FEFamily LINE3;
    const static FEFamily TRIA3;
    const static FEFamily TRIA6;
    const static FEFamily QUAD4;
    const static FEFamily QUAD8;
    const static FEFamily TETRA4;
    const static FEFamily TETRA10;
    const static FEFamily HEXA8;
    const static FEFamily HEXA20;
    const static FEFamily PENTA6;
    const static FEFamily PENTA15;
    const static FEFamily PYRAMID5;
    const static FEFamily PYRAMID13;

  private:
    FEFamily(const std::string _name, int _dim, int _corners);
};

class FEGenericType
{
  public:
    const std::string name;

    const static FEGenericType NODE;
    const static FEGenericType BEAM;
    const static FEGenericType BAR;
    const static FEGenericType SPRING;
    const static FEGenericType JOINT;
    const static FEGenericType PLINK;
    const static FEGenericType DRAWBEAD;
    const static FEGenericType MUSCLE;
    const static FEGenericType JET;
    const static FEGenericType GAP;
    const static FEGenericType TIED;
    const static FEGenericType MEMBR;
    const static FEGenericType SHELL;
    const static FEGenericType SOLID;
    const static FEGenericType PART;

  private:
    FEGenericType(const std::string& _name);
};

class FEType
{
  public:
    const std::string name;

    const FEFamily& family;
    const FEGenericType& genericType;
    // 0D
    const static FEType NODE;
    // 1D
    const static FEType BEAM;
    const static FEType BAR;
    const static FEType SPRING6DOF;
    const static FEType SPHERICALJOINT;
    const static FEType FLEXTORSJOINT;
    const static FEType KJOINT;
    const static FEType PLINK;
    const static FEType MBSJOINT;
    const static FEType MBSSPRING;
    const static FEType SPRINGBEAM;
    const static FEType DRAWBEAD;
    const static FEType MTOJN;
    const static FEType MUSCLE;
    const static FEType JET;
    const static FEType MPCPLINK;
    const static FEType GAP;
    const static FEType TIED; // NOT IN SPEC, APPEARED IN Truck_vs_Truck
    const static std::vector<const FEType*> all1D;
    // 2D
    const static FEType MEMBR;
    const static FEType THICKSHELL;
    const static FEType SHELL;
    const static FEType SHEL6;
    const static FEType SHEL8;
    const static std::vector<const FEType*> all2D;
    // 3D
    const static FEType HEXA8;
    const static FEType BRICKSHELL;
    const static FEType TETRA10;
    const static FEType TETRA4;
    const static FEType PENTA6;
    const static FEType PENTA15;
    const static FEType HEXA20;
    const static std::vector<const FEType*> all3D;

    std::string pathToConnectivity(const DataType& dataType) const;
    std::string pathToResult(const ResultType& resultType, const DataType& dataType) const;
    std::string pathToPerStateResult(const std::string& state, const ResultType& resultType, const DataType& dataType) const;

  private:
    FEType(const std::string& _name, const FEFamily& _family, const FEGenericType& _genericType);
};

} // namespace erfh5

} // namespace c2m

#endif
