//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/bool_param.h>

using namespace std;

namespace dynamica {
    string BoolParam::getName() const {
        return name_;
    }

    void BoolParam::prepInfo(ParamInfoList &info_list) {
        ParamInfo desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "bool";
        info_list.entries.push_back(desc);
    }

    void BoolParam::prepValue(ParamValueList &value_list, uint8_t attribute) {
        ParamValue param;
        param.name = name_;
        switch (attribute) {
            case VALUE:
                param.value = Value(val_).toString();
                break;
            case DEFAULT:
                param.value = Value(def_).toString();
                break;
            default:break;
        }
        value_list.entries.push_back(param);
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
