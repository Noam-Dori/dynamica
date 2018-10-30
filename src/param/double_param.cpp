//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/double_param.h>

using namespace std;

namespace dynamica {
    string DoubleParam::getName() const {
        return name_;
    }

    void DoubleParam::prepInfo(ParamInfoList &info_list) {
        ParamInfo desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "double";
        info_list.entries.push_back(desc);
    }

    void DoubleParam::prepValue(ParamValueList &value_list, uint8_t attribute) {
        ParamValue param;
        param.name = name_;
        switch (attribute) {
            case VALUE:
                param.value = Value(val_).toString();
                break;
            case DEFAULT:
                param.value = Value(def_).toString();
                break;
            case MAX:
                param.value = Value(max_).toString();
                break;
            case MIN:
                param.value = Value(min_).toString();
                break;
            default:break;
        }
        value_list.entries.push_back(param);
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
