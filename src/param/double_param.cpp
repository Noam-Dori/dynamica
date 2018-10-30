//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/double_param.h>

using namespace std;

namespace dynamica {
    string DoubleParam::getName() const {
        return name_;
    }

    void DoubleParam::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "double";
        group.parameters.push_back(desc);
    }

    void DoubleParam::prepConfig(Config &conf) {
        DoubleParameter param;
        param.name = name_;
        param.value = val_;
        conf.doubles.push_back(param);
    }

    void DoubleParam::prepConfigDescription(ConfigDescription &conf_desc) {
        DoubleParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.doubles.push_back(param);
        param.value = max_;
        conf_desc.max.doubles.push_back(param);
        param.value = min_;
        conf_desc.min.doubles.push_back(param);
    }

    int DoubleParam::getLevel() const {
        return level_;
    }

    bool DoubleParam::sameType(Value val) {
        return val.getType() == "double";
    }

    bool DoubleParam::sameValue(Value val) {
        return val.toDouble() == val_;
    }

    void DoubleParam::setValue(Value val) {
        val_ = val.toDouble();
    }

    Value DoubleParam::getValue() const {
        return Value(val_);
    }
}
