#ifndef C2M_ERF_HDF5_PATH_HPP
#define C2M_ERF_HDF5_PATH_HPP

#include <list>
#include <string>

namespace c2m
{
/**
 * @brief Simple string-based path to represent ERF-HDF5 folder structure
 *
 */
class ERFHDF5Path
{
  public:
    /**
     * @brief ERF-HDF5 Directories (called "Group" in hdf5 file format)
     *
     */
    enum class Group : unsigned
    {
        // Base path
        CSMEXPL,
        // Subgroups
        CONSTANT,
        SINGLESTATE,
        CONNECTIVITIES,
        ENTITYRESULTS,
        // Elements (ASSUMPTION: this is complete, no other elements)
        BAR,
        BEAM,
        HEXA8,
        PENTA6,
        PLINK,
        SHELL,
        NODE,
        SOLID,
        // Float properties
        COORDINATE,
        DISPLACEMENT,
        // Zones (ASSUMPTION: only one zone with only one set exists)
        ZONE1_SET0
    };

    /**
     * @brief Get the name of a ERF-HDF5 Group/Directory as a string
     *
     * @param group ERF-HDF5 Group
     * @return std::string name
     */
    static std::string toString(Group group);

    /**
     * @brief ERF-HDF5 Dataset identifiers
     *
     */
    enum class Dataset : unsigned
    {
        NODE_IDS_BY_ELEMENT,
        ELEMENT_ID_BY_ELEMENT,
        PART_ID_BY_ELEMENT,
        ENTITY_ID_BY_ENTITY,
        RESULT_BY_ENTITY
    };

    /**
     * @brief Get the name of a ERF-HDF5 Dataset identifier as a string
     *
     * @param dataset ERF-HDF5 Dataset
     * @return std::string name
     */
    static std::string toString(Dataset dataset);

    /**
     * @brief Constructs an empty path
     *
     */
    ERFHDF5Path() : m_path(0), m_dataset("")
    {
    }

    /**
     * @brief Construct a path using \p group as base Group
     *
     * @param group ERF-HDF5 Group
     */
    ERFHDF5Path(Group group) : m_path({toString(group)}), m_dataset("")
    {
    }

    /**
     * @brief Copies another path
     *
     * @param other other path
     */
    ERFHDF5Path(const ERFHDF5Path& other) : m_path(other.m_path), m_dataset(other.m_dataset)
    {
    }

    /**
     * @brief Copüies another path
     *
     * @param other other path
     * @return ERFHDF5Path& reference to this path
     */
    ERFHDF5Path& operator=(const ERFHDF5Path& other)
    {
        m_path = other.m_path;
        m_dataset = other.m_dataset;
        return *this;
    }

    /**
     * @brief Go into subfolder/subgroup
     *
     * @param group ERF-HDF5 Group (subfolder of current folder)
     * @return ERFHDF5Path& reference to this (for chaining)
     */
    ERFHDF5Path& descend(Group group);

    /**
     * @brief Go into parent folder/group
     *
     * @return ERFHDF5Path& reference to this (for chaining)
     */
    ERFHDF5Path& ascend();

    /**
     * @brief Append a certain dataset to the end of the path
     *
     * @param dataset ERF-HDF5 Dataset
     */
    void dataset(Dataset dataset);

    /**
     * @brief Get base path as /-separated string, without dataset ending
     *
     * @return std::string base path as /-separated string, without dataset ending
     */
    std::string path() const;

    /**
     * @brief Get base path as /-separated string, WITH dataset ending
     *
     * @return std::string base path as /-separated string, WITH dataset ending
     */
    std::string pathToData() const;

  private:
    std::list<std::string> m_path; //!< List of expanded subdirectories
    std::string m_dataset; //!< Name of dataset at bottom of path
};

} // namespace c2m

#endif
