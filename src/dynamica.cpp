//
// Created by Noam Dori on 30/10/18.
//

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCDFAInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "modernize-loop-convert"
#pragma ide diagnostic ignored "modernize-use-auto"

#include <dynamica/dynamica.h>
#include <boost/foreach.hpp>

using namespace boost;
using namespace std;
using namespace ros;
using namespace ros::this_node;

namespace dynamica {

    Dynamica::Dynamica(ros::NodeHandle &nh) {
        nh_ = nh;
    };

    void Dynamica::add(const ParamPtr &param) {
        params_[param->getName()] = param;
    };

    void Dynamica::add(Param *param) {
        add(ParamPtr(param));
    };

    void Dynamica::remove(const ParamPtr &param) {
        remove(param->getName());
    };

    void Dynamica::remove(Param *param) {
        remove(param->getName());
    };

    void Dynamica::remove(const string &param_name) {
        params_.erase(param_name);
    };

    void Dynamica::start() {
        ParamInfoList conf_desc = makeInfo(); // registers defaults and max/min descriptions.
        ParamValueList conf = makeValues(); // the actual config file in C++ form.

        function<bool(ChangeCommand::Request& req, ChangeCommand::Response& rsp)> callback = bind(&internalCallback,this,_1,_2);
        // publishes Config and ConfigDescription.
        set_service_ = nh_.advertiseService(getName() + "/set", callback); // this allows changes to the parameters

        // this makes the parameter descriptions
        info_pub_ = nh_.advertise<ParamInfoList>(getName() + "/info", 1, true);
        info_pub_.publish(conf_desc);

        // this creates the type/level of everything
        value_pub_ = nh_.advertise<ParamValueList>(getName() + "/values", 1, true);
        value_pub_.publish(conf);
    }

    ParamValueList Dynamica::makeValues() {
        ParamValueList values;
        for(ParamMap::const_iterator it = params_.begin(); it != params_.end(); it++) {
            it->second->prepValue(values,VALUE);
        }
        return values;
    }

    ParamInfoList Dynamica::makeInfo() {
        ParamInfoList info;
        for(ParamMap::const_iterator it = params_.begin(); it != params_.end(); it++) {
            it->second->prepInfo(info);
        }
        return info;
    };

    void Dynamica::start(ReconfigureFunction callback) {
        start();
        setCallback(callback);
    };

    void Dynamica::start(void(*callback)(const ParamMap&,int)) {
        ReconfigureFunction f(callback);
        start(f);
    };

    // this is also available in the header file (linking template functions is problematic.
    // template <class T> void Dynamica::start(void(T::*callback)(const ParamMap&,int), T *obj) {
    //        ReconfigureFunction f = bind(callback,obj,_1);
    //        start();
    //    }

    void Dynamica::setCallback(ReconfigureFunction callback) {
        callback_ = make_shared<function<void(const ParamMap&,int)> >(callback);
    };

    void defaultCallback(const ParamMap&,int) {};

    void Dynamica::clearCallback() {
        callback_ = make_shared<ReconfigureFunction>(&defaultCallback);
    };

    // Private function: internal callback used by the service to call our lovely callback.
    bool Dynamica::internalCallback(Dynamica *obj, ChangeCommand::Request& req, ChangeCommand::Response& rsp) {
        ROS_DEBUG_STREAM("Called config callback of dynamica");

        int level = obj->update(req);

        if (obj->callback_) {
            try {
                (*obj->callback_)(obj->params_,level);
            } catch (std::exception &e) {
                ROS_WARN("Reconfigure callback failed with exception %s: ", e.what());
            } catch (...) {
                ROS_WARN("Reconfigure callback failed with unprintable exception.");
            }
        }

        obj->value_pub_.publish(obj->makeValues()); // updates the config

        return true;
    }

    int Dynamica::update(const ChangeCommand::Request &req) {
        int level = 0;
        BOOST_FOREACH(const ParamValue i,req.new_params.entries) {
            int new_level = reassignValue(i.name, i.value);
            if(new_level == -1) {
                ROS_ERROR_STREAM("Variable [" << i.name << "] is not registered");
            } else {
                level |= new_level;
            }
        }
        return level;
    }


    int Dynamica::reassignValue(const string &name, Value value) {
        if(params_.find(name) != params_.end()) {
            ParamPtr old = params_[name]; // this is the old map which might be updated.
            if(old->sameValue(value)) { return 0;} else {
                old->setValue(value);
                return old->getLevel();
            }
        } else {
            return -1;
        }
    }

    bool Dynamica::reassignAttribute(const string &name, const Value &value, Attribute property) {
        if(property == VALUE) {
            return false;
        }
        if(params_.find(name) != params_.end()) { // if the param with the given name exists,
            // and either you are modifying the level,
            // or its same same type as the member param.
            if(DDDPtr old = dynamic_pointer_cast<DDDParam>(map[name])) {
                switch (property) {
                    default: {
                        ROS_WARN_STREAM("Asked to edit unknown property of parameter [" << name << "]. Ignored.");
                        return true;
                    }
                    case DEFAULT: {
                        old->setDefault(value);
                        return true;
                    }
                    case LEVEL: {
                        old->setLevel((unsigned int) value.toInt());
                        return true;
                    }
                    case MAX: {
                        if (old->isOrdered()) {
                            dynamic_pointer_cast<DDDOrdered>(old)->setMax(value);
                        } else {
                            ROS_WARN_STREAM("Asked to edit range property of parameter [" << name
                                                                                          << "], which does not have range properties. Ignored.");
                        }
                        return true;
                    }
                    case MIN: {
                        if (old->isOrdered()) {
                            dynamic_pointer_cast<DDDOrdered>(old)->setMin(value);
                        } else {
                            ROS_WARN_STREAM("Asked to edit range property of parameter [" << name
                                                                                          << "], which does not have range properties. Ignored.");
                        }
                        return true;
                    }
                }
            }
        } else {
            return false;
        }
    }

    Value Dynamica::get(const char *name) {
        return dynamica::get(params_,name);
    }

    ParamPtr Dynamica::at(const char *name) {
        return dynamica::at(params_,name);
    }

    ostream &operator<<(ostream &os, const Dynamica &dd) {
        os << "{" << *dd.params_.begin()->second;
        for(ParamMap::const_iterator it = ++dd.params_.begin(); it != dd.params_.end(); it++) {
            os << "," << *it->second;
        }
        os << "}";
        return os;
    }

    ParamPtr at(const ParamMap& map, const char *name) {
        ParamMap::const_iterator it = map.find(name);
        if(it == map.end()) {
            return ParamPtr(); // null pointer
        } else { return it->second;}
    }

    Value get(const ParamMap& map, const char *name) {
        ParamMap::const_iterator it = map.find(name);
        if(it == map.end()) {
            return Value("\000");
        } else { return it->second->getValue();}
    }
}
#pragma clang diagnostic pop