//
// Created by Noam Dori on 30/10/18.
//

#include <dynamica/param/string_param.h>

using namespace std;

namespace dynamica {
    string StringParam::getName() const {
        return name_;
    }

    void StringParam::prepInfo(ParamInfoList &info_list) {
        ParamInfo desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "string";
        info_list.entries.push_back(desc);
    }

    void StringParam::prepValue(ParamValueList &value_list, uint8_t attribute) {
        ParamValue param;
        param.name = name_;
        switch (attribute) {
            case VALUE:
                param.value = val_;
                break;
            case DEFAULT:
                param.value = def_;
                break;
            default:break;
        }
        value_list.entries.push_back(param);
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
