//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/bool_param.h>

using namespace std;

namespace dynamica {
    string BoolParam::getName() const {
        return name_;
    }

    void BoolParam::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "bool";
        group.parameters.push_back(desc);
    }

    void BoolParam::prepConfig(Config &conf) {
        BoolParameter param;
        param.name = name_;
        param.value = (unsigned char)val_;
        conf.bools.push_back(param);
    }

    void BoolParam::prepConfigDescription(ConfigDescription &conf_desc) {
        BoolParameter param;
        param.name = name_;
        param.value = (unsigned char)def_;
        conf_desc.dflt.bools.push_back(param);
        param.value = (unsigned char)true;
        conf_desc.max.bools.push_back(param);
        param.value = (unsigned char)false;
        conf_desc.min.bools.push_back(param);
    }

    int BoolParam::getLevel() const {
        return level_;
    }

    bool BoolParam::sameType(Value val) {
        return val.getType() == "bool";
    }

    bool BoolParam::sameValue(Value val) {
        return val.toBool() == val_;
    }

    void BoolParam::setValue(Value val) {
        val_ = val.toBool();
    }

    Value BoolParam::getValue() const {
        return Value(val_);
    }
}
