#ifndef C2M_C2M_WRITER_HPP
#define C2M_C2M_WRITER_HPP

#include <crash2mesh/core/collectors.hpp>

namespace c2m
{

/**
 * @brief Class for functions to write scenes and parts as *.c2m files (see specification below)
 *
 */
class C2MWriter
{
  public:
    /**
     * @brief Write \p scene to file \p filename in *.c2m file format (see specs below)
     *
     * @param filename full path and name of the file to be produced
     * @param scene scene to store
     * @param binary whether *.c2m binary format should be used (see specs below)
     */
    static void write(const std::string& filename, const Scene::Ptr& scene, bool binary = true);
};

/*
C2M Output Format Specification VERSION 1.0

============================
General Structure:
============================
$$SECTION HEADER$$
    ...essential ascii information for parsing rest of sections...
$$ENDSECTION HEADER$$
$$SECTION VERTICES$$
    ...binary or ascii mapping of vertex id to per-frame xyz-coordinates...
$$ENDSECTION VERTICES$$
$$SECTION STRAINS$$
    ...binary or ascii mapping of strain id to per-frame plastic strain values...
$$ENDSECTION STRAINS$$
$$SECTION RODS$$
    ...binary or ascii mapping of rod/line id to its 2 vertex ids (source and target)...
$$ENDSECTION RODS$$
$$SECTION TRIANGLES$$
    ...binary or ascii mapping of triangle id to its strain id and its 3 vertex ids ...
$$ENDSECTION TRIANGLES$$
$$SECTION PARTS$$
    ...binary or ascii mapping of part id to its number of rods, its number of triangles,
            its list of rod ids and its list of triangle ids...
$$ENDSECTION PARTS$$

============================
Section specification:
============================
Info1: in verbatim content marked with <...> is variable, rest is FIXED
Info2: there is a single newline between the end of one section and the beginning of a new one
Info3: the exact amount of whitespace in ascii-format may be subject to change!

------------------------
Header section verbatim:
------------------------
$$SECTION HEADER$$
version 1.0
binary <0 if other sections in ascii, 1 if other sections in binary>
frames <number of frames>
vertices <number of vertices>
strains <number of surface plastic strains>
rods <number of 1D elements, i.e. lines which are not part of any triangle>
triangles <number of triangles>
parts <number of parts>
$$ENDSECTION HEADER$$
------------------------
Header section info:
------------------------
- header section is always the first section
- whitespace exactly as depicted, i.e. a space between identifier and value
  and a newline after each identifier-value-pair
- all numbers marked with <...> are ascii-strings

------------------------
Vertices section verbatim:
------------------------
$$SECTION VERTICES$$
<VERTEX1_ID>
    <x-position frame0> <y-position frame0> <z-position frame0>
    <x-position frame1> <y-position frame1> <z-position frame1>
    <x-position frame2> <y-position frame2> <z-position frame2>
    <...repeated for a total of "frames" times (see header)...>
<VERTEX2_ID>
    <x-position frame0> <y-position frame0> <z-position frame0>
    <x-position frame1> <y-position frame1> <z-position frame1>
    <x-position frame2> <y-position frame2> <z-position frame2>
    <...repeated for a total of "frames" times (see header)...>
<...repeated for a total of "vertices" times (see header)...>
$$ENDSECTION VERTICES$$
------------------------
Vertices section info:
------------------------
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION VERTICES$$, after each <VERTEX_ID> and after
        each set of xyz-coordinates
    - four spaces before each set of xyz-coordinates
    - a single space between x and y coordinates, y and z coordinates
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION VERTICES$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
        $$ENDSECTION VERTICES$$
- in binary format VERTEX_ID is a 4 byte unsigned integer, each x/y/z coordinate
  is a 4 byte float (i.e. NOT double precision)

------------------------
Strains section verbatim:
------------------------
$$SECTION STRAINS$$
<STRAIN1_ID>
    <plastic strain1 at frame1>
    <plastic strain1 at frame2>
    <plastic strain1 at frame3>
    <...repeated for a total of "frames" times (see header)...>
<STRAIN2_ID>
    <plastic strain2 at frame1>
    <plastic strain2 at frame2>
    <plastic strain2 at frame3>
    <...repeated for a total of "frames" times (see header)...>
<...repeated for a total of "strains" times (see header)...>
$$ENDSECTION STRAINS$$
------------------------
Strains section info:
------------------------
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION STRAINS$$, after each <STRAIN_ID> and after
        each plastic strain value
    - four spaces before each plastic strain value
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION STRAINS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
        $$ENDSECTION STRAINS$$
- in binary format STRAIN_ID is a 4 byte unsigned integer, plastic strain
  is a 4 byte float (i.e. NOT double precision)

------------------------
Rods section verbatim:
------------------------
$$SECTION RODS$$
<ROD1_ID> <ROD1_VERTEX1_ID> <ROD1_VERTEX2_ID>
<ROD2_ID> <ROD2_VERTEX1_ID> <ROD2_VERTEX2_ID>
<...repeated for a total of "rods" times (see header)>
$$ENDSECTION RODS$$
------------------------
Rods section info:
------------------------
- VERTEX_IDs reference the Vertices section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION RODS$$ and after each specified Rod
    - a single space in between ROD_ID and each of the two following VERTEX_IDs
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION RODS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION RODS$$
- in binary format all IDs are 4byte unsigned integers

------------------------
TRIANGLES section verbatim:
------------------------
$$SECTION TRIANGLES$$
<TRIANGLE1_ID> <TRIANGLE1_STRAIN_ID> <TRIANGLE1_VERTEX1_ID> <TRIANGLE1_VERTEX2_ID> <TRIANGLE1_VERTEX3_ID>
<TRIANGLE2_ID> <TRIANGLE2_STRAIN_ID> <TRIANGLE2_VERTEX1_ID> <TRIANGLE2_VERTEX2_ID> <TRIANGLE2_VERTEX3_ID>
<...repeated for a total of "triangles" times (see header)>
$$ENDSECTION TRIANGLES$$
------------------------
Triangles section info:
------------------------
- VERTEX_IDs reference the Vertices section entries by their specified indices
- STRAIN_IDs reference the Strains section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION TRIANGLES$$, and after each specified triangle
    - a single space in between each subsequent ID
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION TRIANGLES$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION TRIANGLES$$
- in binary format all IDs are 4byte unsigned integers

------------------------
Parts section verbatim:
------------------------
$$SECTION PARTS$$
<PART1_ID> <numberOfRods in part1> <numberOfTriangles in part1>
    <PART1_ROD_ID1> <PART1_ROD_ID2> <...repeated a total of numberOfRods times...>
    <PART1_TRIANGLE_ID1> <PART1_TRIANGLE_ID2> <...repeated a total of numberOfTriangles times...>
<PART2_ID> <numberOfRods in part2> <numberOfTriangles in part2>
    <PART2_ROD_ID1> <PART2_ROD_ID2> <...repeated a total of numberOfRods times...>
    <PART2_TRIANGLE_ID1> <PART2_TRIANGLE_ID2> <...repeated a total of numberOfTriangles times...>
<...repeated for a total of "parts" times (see header)>
$$ENDSECTION PARTS$$
------------------------
Parts section info:
------------------------
- ROD_ID reference the rods section entries by their specified indices
- TRIANGLE_IDs reference the triangles section entries by their specified indices
- in ascii format (see "binary" in header) the whitespace is exactly as
  depicted, meaning:
    - a newline after $$SECTION PARTS$$, after each PART_ID-number-number triplet,
      after each list of ROD_IDs and after each list of TRIANGLE_IDs
    - four spaces before each list of ROD_IDs and before each list of TRIANGLE_IDs
    - a single space in between PART_ID and its number of rods and its number of triangles,
      as well as between each subsequent entry in the lists of ROD_IDs and TRIANGLE_IDs
    - IMPORTANT: in ascii if numberOfRods or numberOfTriangles is 0, the respective line is missing,
        i.e. there is no empty line.
        example:
        41 0 0              ||<- neither rods nor triangles
        42 0 3
            1 2 3           ||<- only triangles
        43 2 0
            20 22           ||<- only rods
        44 4 2
            4 5 6 7         ||<- rods
            23 24           ||<- ...and triangles
        ...and so on...
- in binary format there is NO WHITESPACE except:
    - a newline DIRECTLY after $$SECTION PARTS$$
    - a newline DIRECTLY after the binary contents, i.e. DIRECTLY before
      $$ENDSECTION PARTS$$
- in binary format the number of rods, number of triangles and all IDs are 4byte unsigned integers

*/
} // namespace c2m

#endif
