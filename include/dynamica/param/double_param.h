//
// Created by Noam Dori on 30/10/18.
//

#ifndef DYNAMICA_DOUBLE_PARAM_H
#define DYNAMICA_DOUBLE_PARAM_H

#include "../param.h"


namespace dynamica {
    typedef std::numeric_limits<double> d_limit;
    /**
     * @brief a double implementation of the parameter.
     * This is used to handle double-precision floating point numbers,
     * though it can handle single precision as well.
     */
    class DoubleParam : virtual public Param {
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
         * creates a new double param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to DBL_MAX
         * @param min the minimum allowed value. Defaults to -DBL_MAX
         */
        DoubleParam(const std::string &name, unsigned int level, const std::string &description, double def,
                 double min = -d_limit::infinity(), double max = d_limit::infinity())
                : max_(), min_() {
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
        double def_,max_,min_,val_;
        /**
         * @brief the name of the parameter (name_),
         * and its description (desc_)
         */
        std::string name_, desc_;
    };
}


#endif //DYNAMICA_DOUBLE_PARAM_H
