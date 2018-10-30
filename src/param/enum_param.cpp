//
// Created by Noam Dori on 30/10/18.
//
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "modernize-loop-convert"
#pragma ide diagnostic ignored "modernize-use-auto"

#include <dynamica/param/enum_param.h>

using namespace std;

namespace dynamica {
    EnumMap fillGaps(map<string,int> old_map) {
        EnumMap ret;
        for(map<string,int>::const_iterator it = old_map.begin(); it != old_map.end(); it++) {
            ret[it->first] = pair<int,string>(it->second,"");
        };
        return ret;
    };

    void EnumParam::prepInfo(ParamInfoList &info_list) {
        ParamInfo desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "enum";
        info_list.entries.push_back(desc);
    }

    bool EnumParam::sameType(Value val) {
        return val.getType() == "int" || val.getType() == "string";
    }

    bool EnumParam::sameValue(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString())->second.first == val_) {
            return true;
        } else {
            return val.toInt() == val_;
        }
    }

    void EnumParam::setValue(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString()) != dict_.end()) {
            val_ = lookup(val);
        } else {
            val_ = val.toInt();
        }
    }

    int EnumParam::lookup(Value val) {
        if(val.getType() == "string" && dict_.find(val.toString()) != dict_.end()) {
            return dict_.find(val.toString())->second.first;
        } else {
            return val.toInt();
        }
    }

    EnumParam::EnumParam(const string &name, unsigned int level, const string &description,
                     int def, const map<string, int> &dictionary) :
            IntParam(name,level,description,def),
            dict_(fillGaps(dictionary)) {
        max_ = def;
        min_ = def;
        for(map<string,int>::const_iterator it = dictionary.begin(); it != dictionary.end(); it++) {
            if(it->second > max_) {max_ = it->second;}
            if(it->second < min_) {min_ = it->second;}
        };
    }

    EnumParam::EnumParam(const string &name, unsigned int level, const string &description,
                     const string &def, const map<string, int> &dictionary) :
            IntParam(name,level,description,dictionary.find(def)->second),
            dict_(fillGaps(dictionary)) {
        max_ = def_;
        min_ = def_;
        for(map<string,int>::const_iterator it = dictionary.begin(); it != dictionary.end(); it++) {
            if(it->second > max_) {max_ = it->second;}
            if(it->second < min_) {min_ = it->second;}
        };
    }

    EnumParam::EnumParam(const string &name, unsigned int level, const string &description, int def,
                     const pair<EnumMap, string> &dictionary) :
            IntParam(name,level,description,def),
            dict_(dictionary.first) {
        max_ = def;
        min_ = def;
        for(EnumMap::const_iterator it = dict_.begin(); it != dict_.end(); it++) {
            if(it->second.first > max_) {max_ = it->second.first;}
            if(it->second.first < min_) {min_ = it->second.first;}
        };
        enum_description_ = dictionary.second;
    }

    EnumParam::EnumParam(const string &name, unsigned int level, const string &description, const string &def,
                     const pair<EnumMap,string> &dictionary) :
            IntParam(name,level,description,dictionary.first.find(def)->second.first),
            dict_(dictionary.first) {
        max_ = def_;
        min_ = def_;
        for(EnumMap::const_iterator it = dict_.begin(); it != dict_.end(); it++) {
            if(it->second.first > max_) {max_ = it->second.first;}
            if(it->second.first < min_) {min_ = it->second.first;}
        };
        enum_description_ = dictionary.second;
    }

    string EnumParam::makeEditMethod() {
        stringstream ret;
        ret << "{";
        {
            ret << "'enum_description': '" << enum_description_ << "', ";
            ret << "'enum': [";
            {
                EnumMap::const_iterator it = dict_.begin();
                ret << makeConst(it->first, it->second.first, it->second.second);
                for(it++; it != dict_.end(); it++) {
                    ret << ", " << makeConst(it->first, it->second.first, it->second.second);
                };
            }
            ret << "]";
        }
        ret << "}";
        return ret.str();
    }

    string EnumParam::makeConst(string name, int value, string desc) {
        stringstream ret;
        ret << "{";
        {
            ret << "'srcline': 0, "; // the sole reason this is here is because dynamic placed it in its enum JSON.
            ret << "'description': '" << desc << "', ";
            ret << "'srcfile': '/does/this/really/matter.cfg', "; // the answer is no. This is useless.
            ret << "'cconsttype': 'const int', ";
            ret << "'value': '" << value << "', ";
            ret << "'ctype': 'int', ";
            ret << "'type': 'int', ";
            ret << "'name': '" << name << "'";
        }
        ret << "}";
        return ret.str();
    }
}
#pragma clang diagnostic pop