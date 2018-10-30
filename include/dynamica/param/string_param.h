//
// Created by Noam Dori on 30/10/18.
//

#ifndef DYNAMICA_STRING_PARAM_H
#define DYNAMICA_STRING_PARAM_H

#include "../param.h"
namespace dynamica {
    /**
     * @brief a string implementation of the parameter.
     * This is used to handle strings of characters of variable length.
     * Like string, each param value can hold up to 2^32-1 characters.
     */
    class StringParam : virtual public Param {
    public:
        std::string getName() const;

        void prepGroup(Group &group);

        void prepConfig(Config &conf);

        void prepConfigDescription(ConfigDescription &conf_desc);

        int getLevel() const;

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        Value getValue() const;

        /**
         * creates a new string param
         * @param name the name of the parameter
         * @param level the change level
         * @param description details about the parameter
         * @param def the default value
         */
        StringParam(const std::string &name, unsigned int level, const std::string &description, const std::string &def) {
            name_ = name;
            level_ = level;
            desc_ = description;
            def_ = def;
            val_ = def;
        }
    protected:
        /**
         * @brief the level of the parameter:
         * the degree in which things need to be shut down if this param changes
         */
        unsigned int level_;
        /**
         * @brief the default value (def_),
         * and the current value (val_)
         */
        std::string def_,val_;
        /**
         * @brief the name of the parameter (name_),
         * and its description (desc_)
         */
        std::string name_, desc_;
    };
}


#endif //DYNAMICA_STRING_PARAM_H
