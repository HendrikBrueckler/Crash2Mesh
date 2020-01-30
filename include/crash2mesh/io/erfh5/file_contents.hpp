#ifndef C2M_ERFH5_FILE_CONTENTS_HPP
#define C2M_ERFH5_FILE_CONTENTS_HPP

#include <string>
#include <vector>

namespace c2m
{
namespace erfh5
{

/**
 * @brief Represents unspecific subfolder/group of erhdf5 files
 *
 */
class Group
{
  public:
    const static Group CSMEXPL;                 ///< main folder containing the crash sim results
    const static Group CONSTANT;                ///< subfolder containing non-timevariant variables
    const static Group CONNECTIVITIES;          ///< subsubfolder containing time-constant connectivities
    const static Group ENTITYRESULTS_CONSTANT;  ///< subsubfolder containing time-constant float-type results
    const static Group SINGLESTATE;             ///< subfolder containing time-variant variables/results
    const static Group STATE_TEMPLATE;          ///< subsubfolder template for each state of the time-series
    const static Group ENTITYRESULTS_PER_STATE; ///< subsubsubfolder template containing float-type results of one state

    const Group* const parent; ///< the parent directory
    const std::string name;    ///< the directories name
    /**
     * @brief Get the complete path from root to this directory
     *
     * @return std::string complete path
     */
    std::string path() const;

  private:
    std::string m_path; ///< the complete path from root to this directory
    /**
     * @brief Construct a new Group. Meant only for static members.
     *
     * @param _parent parent group
     * @param _name directory name
     */
    Group(const Group* _parent, const std::string& _name);
};

/**
 * @brief Represents a type of float-result
 *
 */
class ResultType
{
  public:
    std::string name; ///< The folder name associated with this result-type

    const static ResultType SPACE_DOMAIN;                     ///< Subdomain of space a finite element belongs to
    const static ResultType COORDINATE;                 ///< Coordinate of a finite element
    const static ResultType TRANSLATIONAL_DISPLACEMENT; ///< Displacement of a finite element
    const static ResultType PLASTIC_STRAIN;             ///< plastic strain of a finite element
    const static ResultType MIN_PLASTIC_STRAIN;         ///< minimum plastic strain of a finite element
    const static ResultType MAX_PLASTIC_STRAIN;         ///< maximum plastic strain of a finite element
    const static ResultType THICKNESS;                  ///< thickness of a finite element
    const static ResultType EQUIVALENT_PLASTIC_STRAIN;  ///< equivalent plastic strain of a finite element

  private:
    /**
     * @brief Construct a new ResultType. Meant only for static members.
     *
     * @param _name directory name associated with the result type
     */
    ResultType(const std::string& _name);
};

/**
 * @brief Represents the type and identifier of a dataset
 *
 */
class DataType
{
  public:
    std::string name; ///< The identifier name associated with this dataset

    const static DataType CONNECTED_NODE_IDS; ///< Indices /identifiers of the nodes a finite element connects
    const static DataType ELEMENT_IDS;        ///< Finite element identifiers
    const static DataType PART_IDS;           ///< Part identifiers
    const static DataType ENTITY_IDS;         ///< Generic entity identifiers
    const static DataType RESULTS;            ///< Float type results
    const static DataType STATE_INDEX;        ///< Index of time series states
    const static DataType STATE_VALUE;        ///< Time value associated with a time series state

  private:
    /**
     * @brief Construct a new DataType. Meant only for static members.
     *
     * @param _name directory name associated with the data type
     */
    DataType(const std::string& _name);
};

/**
 * @brief Represents a family of finite elements
 *        (taken from https://myesi.esi-group.com/ERF-HDF5/doc/ERF_RESULT_Specs_VPS2014.pdf)
 *
 */
class FEFamily
{
  public:
    const std::string name; ///< The family's name
    const int dimension;    ///< The dimension of elements in this family
    const int numCorners;   ///< The number of corners (not including higher order control points)
    const std::vector<std::vector<int>> facesCCW;

    const static FEFamily POINT;
    const static FEFamily LINE2;
    const static FEFamily LINE3;
    const static FEFamily TRIA3;
    const static FEFamily TRIA6;
    const static FEFamily QUAD4; // VERY DANGEROUS, QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==0
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
    /**
     * @brief Construct a new FEFamily. Meant only for static members.
     *
     * @param _name directory name associated with the data type
     * @param _dim dimension of this family
     * @param _corners number of relevant corners
     * @param _faces indices of face nodes of this element (if any)
     */
    FEFamily(const std::string _name, int _dim, int _corners, const std::vector<std::vector<int>> _faces);
};

/**
 * @brief Represents a generic type of finite elements
 *        (taken from https://myesi.esi-group.com/ERF-HDF5/doc/ERF_RESULT_Specs_VPS2014.pdf)
 *
 */
class FEGenericType
{
  public:
    const std::string name; ///< Directory name associated with this generic type

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
    /**
     * @brief Construct a new FEGenericType. Meant only for static members.
     *
     * @param _name Directory name associated with the generic type
     */
    FEGenericType(const std::string& _name);
};

/**
 * @brief Represents a concrete type of finite elements
 *        (taken from https://myesi.esi-group.com/ERF-HDF5/doc/ERF_RESULT_Specs_VPS2014.pdf)
 *
 */
class FEType
{
  public:
    const std::string name; ///< Directory name associated with this finite element type

    const FEFamily& family;           ///< finite element family this finite element type belongs to
    const FEGenericType& genericType; ///< generic type this concrete type belongs to
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
    const static FEType TIED; ///< NOT IN SPEC, APPEARED IN Truck_vs_Truck
    // 2D
    const static FEType MEMBR;      ///< VERY DANGEROUS, QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==0
    const static FEType THICKSHELL; ///< VERY DANGEROUS, QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==0
    const static FEType SHELL;      ///< VERY DANGEROUS, QUAD4s CAN BE TRIA3s IF NODE4==NODE3 OR NODE4==0
    const static FEType SHEL6;
    const static FEType SHEL8;
    const static FEType SHEL3;    ///< Not in spec, replaces QUAD4-Shells which are actually triangles
    const static FEType SURFACE3; ///< Not in spec, represents extracted triangle surfaces of 3D elements
    const static FEType SURFACE4; ///< Not in spec, represents extracted quad surfaces of 3D elements
    // 3D
    const static FEType HEXA8;
    const static FEType BRICKSHELL;
    const static FEType TETRA10;
    const static FEType TETRA4;
    const static FEType PENTA6;
    const static FEType PENTA15;
    const static FEType HEXA20;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the connectivities for this finite element type
     *
     * @return std::string path to the connectivities for this finite element type
     */
    std::string pathToConnectivity() const;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the dataset of connectivities for this finite element
     * type
     *
     * @param dataType The dataset to query
     * @return std::string path to dataset
     */
    std::string pathToConnectivity(const DataType& dataType) const;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the constant (non-state-dependent) float-type results
     * for this finite element type.
     *
     * @param resultType The type of results to query
     * @return std::string path to results
     */
    std::string pathToResults(const ResultType& resultType) const;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the dataset of constant (non-state-dependent)
     * float-type results for this finite element type.
     *
     * @param resultType The type of results to query
     * @param dataType The dataset to query
     * @return std::string path to dataset
     */
    std::string pathToResults(const ResultType& resultType, const DataType& dataType) const;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the state-dependant (variant) float-type results
     * for this finite element type.
     *
     * @param state The name of the state to query
     * @param resultType The type of results to query
     * @return std::string path to results
     */
    std::string pathToPerStateResults(const std::string& state, const ResultType& resultType) const;

    /**
     * @brief Returns the path within an ERF-HDF5 file containing the dataset of state-dependant (variant) float-type
     * results for this finite element type.
     *
     * @param state The name of the state to query
     * @param resultType The type of results to query
     * @param dataType The dataset to query
     * @return std::string path to results
     */
    std::string
    pathToPerStateResults(const std::string& state, const ResultType& resultType, const DataType& dataType) const;

  private:
    /**
     * @brief Construct a new FEType. Meant only for static members.
     *
     * @param _name folder name associated with the finite element type
     * @param _family family the finite element type belongs to
     * @param _genericType generic type the finite element type belongs to
     */
    FEType(const std::string& _name, const FEFamily& _family, const FEGenericType& _genericType);
};

} // namespace erfh5

} // namespace c2m

#endif
