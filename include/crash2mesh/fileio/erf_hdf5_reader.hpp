#ifndef C2M_ERF_HDF5_READER_HPP
#define C2M_ERF_HDF5_READER_HPP

#include <crash2mesh/core/types.hpp>

#include <map>

namespace c2m
{

class ERFHDF5Reader
{
  public:
    /**
     * @brief Create a reader that reads from a given ERF-HDF5 file
     *
     * @param filename name of file to read from
     */
    ERFHDF5Reader(const std::string& filename) : m_file(filename)
    {
    }

    /**
     * @brief Read identifiers and positions of vertices into \p vertexEntityID2Position
     *
     * @param vertexEntityID2Position map from vertex identifier to its position
     * @return true if successful
     * @return false if aborted due to reading error
     */
    bool readVertices(std::map<entid_t, VolVec3>& vertexEntityID2Position) const;

  private:
    H5File m_file; //!< ERF-HDF5 file handle
};

} // namespace c2m

#endif
