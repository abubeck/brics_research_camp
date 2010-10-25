#include "fiducial.hpp"

using namespace std;

std::ostream& operator << ( std::ostream& os, miarn_fiducial_item& c ) {
    os << "type " << c.type << "\n";
    os << "id " << c.id << "\n";
    for (int i = 0; i < 3 ; i++)
        os << "pos[" << i << "]= " << c.pos[i] << "\n";
    for (int i = 0; i < 3 ; i++)
        os << "rot[" << i << "]= " << c.rot[i] << "\n";
    for (int i = 0; i < 3 ; i++)
        os << "upos[" << i << "]= " << c.upos[i] << "\n";
    for (int i = 0; i < 3 ; i++)
        os << "urot[" << i << "]= " << c.urot[i] << "\n";
    os << "begin_index: " << c.begin_index << " end_index: " << c.end_index << "\n";
    os << "begin_x: " << c.begin_x << " end_x: " << c.end_x << "\n";
    os << "begin_y: " << c.begin_y << " end_y: " << c.end_y << "\n";
    for (int i = 0; i < 4 ; i++)
        os << "extra[" << i << "]= " << c.extra[i] << "\n";
    return os;
}

std::ostream& operator << ( std::ostream& os, miarn_fiducial_data& c ) {
    os << "miarn_fiducial_data_t.size: " << c.size()<< "\n";
    for (uint i = 0; i < c.size(); i++)
        os << "miarn_fiducial_data[" << i << "]: "<< c[i] ;
    return os;
}
