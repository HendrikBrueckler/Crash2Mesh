#ifndef C2M_ERFH5_READER_HPP
#define C2M_ERFH5_READER_HPP

#include <crash2mesh/core/types.hpp>
#include <crash2mesh/fileio/erfh5/filecontents.hpp>

#include <map>

namespace c2m
{
namespace erfh5
{

class Reader
{
  public:
    /**
     * @brief Create a reader that reads from a given ERF-HDF5 file
     *
     * @param filename name of file to read from
     */
    Reader(const std::string& filename) : m_file(filename)
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

    bool readFiniteElements(std::map<const FEType*, std::map<entid_t, std::vector<entid_t>>>& elementType2Elements) const;

  private:
    void logPathError(const std::string& msg, const std::string& path) const;
    void logPathInfo(const std::string& msg, const std::string& path) const;
    H5File m_file; //!< ERF-HDF5 file handle
};

} // namespace erfh5
} // namespace c2m

#endif
