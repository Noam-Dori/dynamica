//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/string_param.h>

using namespace std;

namespace dynamica {
    string StringParam::getName() const {
        return name_;
    }

    void StringParam::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "string";
        group.parameters.push_back(desc);
    }

    void StringParam::prepConfig(Config &conf) {
        StrParameter param;
        param.name = name_;
        param.value = val_;
        conf.strs.push_back(param);
    }

    void StringParam::prepConfigDescription(ConfigDescription &conf_desc) {
        StrParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.strs.push_back(param);
        param.value = "";
        conf_desc.max.strs.push_back(param);
        param.value = "";
        conf_desc.min.strs.push_back(param);
    }

    int StringParam::getLevel() const {
        return level_;
    }

    bool StringParam::sameType(Value val) {
        return val.getType() == "string";
    }

    bool StringParam::sameValue(Value val) {
        return val.toString() == val_;
    }

    void StringParam::setValue(Value val) {
        val_ = val.toString();
    }

    Value StringParam::getValue() const {
        return Value(val_);
    }
}
