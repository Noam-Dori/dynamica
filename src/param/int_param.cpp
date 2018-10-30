//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/int_param.h>

using namespace std;

namespace dynamica {
    string IntParam::getName() const {
        return name_;
    }

    void IntParam::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "int";
        group.parameters.push_back(desc);
    }

    void IntParam::prepConfig(Config &conf) {
        IntParameter param;
        param.name = name_;
        param.value = val_;
        conf.ints.push_back(param);
    }

    void IntParam::prepConfigDescription(ConfigDescription &conf_desc) {
        IntParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.ints.push_back(param);
        param.value = max_;
        conf_desc.max.ints.push_back(param);
        param.value = min_;
        conf_desc.min.ints.push_back(param);
    }

    int IntParam::getLevel() const {
        return level_;
    }

    bool IntParam::sameType(Value val) {
        return val.getType() == "int";
    }

    bool IntParam::sameValue(Value val) {
        return val.toInt() == val_;
    }

    void IntParam::setValue(Value val) {
        val_ = val.toInt();
    }

    Value IntParam::getValue() const {
        return Value(val_);
    }
}
