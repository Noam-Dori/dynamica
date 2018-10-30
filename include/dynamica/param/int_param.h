//
// Created by Noam Dori on 30/10/18.
//

#ifndef DYNAMICA_INT_PARAM_H
#define DYNAMICA_INT_PARAM_H

#include "../param.h"

namespace dynamica {
    /**
     * @brief an integer implementation of the parameter.
     * This is used to 32 bit signed integral numbers.
     * This can also handle shorts, bytes, and other integrals provided they are not too big
     * (by then looping will occur)
     */
    class IntParam : virtual public Param {
    public:
        std::string getName() const;

        void prepInfo(ParamInfoList &info_list);

        void prepValue(ParamValueList &value_list, uint8_t attribute);

        int getLevel() const;

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        Value getValue() const;

        /**
         * creates a new int param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to INT32_MAX
         * @param min the minimum allowed value. Defaults to INT32_MIN
         */
        inline IntParam(const std::string &name, unsigned int level, const std::string &description,
                int def, int min = INT32_MIN, int max = INT32_MAX) {
            name_ = name;
            level_ = level;
            desc_ = description;
            def_ = def;
            val_ = def;
            max_ = max;
            min_ = min;
        }

    protected:
        /**
         * @brief the level of the parameter:
         * the degree in which things need to be shut down if this param changes
         */
        unsigned int level_;
        /**
         * @brief the default value (def_),
         * the current value (val_),
         * the minimum allowed value (min_),
         * and the maximum allowed value (max_)
         */
        int def_,max_,min_,val_;
        /**
         * @brief the name of the parameter (name_),
         * and its description (desc_)
         */
        std::string name_, desc_;
    };
}

#endif //DYNAMICA_INT_PARAM_H
