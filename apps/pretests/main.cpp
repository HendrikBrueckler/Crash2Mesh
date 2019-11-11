#include "highfive/H5Easy.hpp"
#include <vector>

using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2)
        return -1;

    HighFive::File file(argv[1]);
    vector<vector<double>> v = H5Easy::load<vector<vector<double>>>(file, string("/CSMEXPL/constant/entityresults/NODE/COORDINATE/ZONE1_set0/erfblock/res"));

    cout << "Length " << v.size() << " " << (v.empty() ? 0 : v[0].size()) << endl;
    return 0;
}