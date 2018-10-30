//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param.h>

using namespace std;
namespace dynamica {
    ostream &operator<<(ostream &os, const Param &param) {
        os << param.getName() << ":" << param.getValue().toString();
        return os;
    }
}