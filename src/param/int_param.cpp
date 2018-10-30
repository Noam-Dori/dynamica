//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/int_param.h>

using namespace std;

namespace dynamica {
    string IntParam::getName() const {
        return name_;
    }

    void IntParam::prepInfo(ParamInfoList &info_list) {
        ParamInfo desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "int";
        info_list.entries.push_back(desc);
    }

    void IntParam::prepValue(ParamValueList &value_list, uint8_t attribute) {
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
