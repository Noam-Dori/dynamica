//
// Created by Noam Dori on 30/10/18.
//

#ifndef DYNAMICA_ENUM_PARAM_H
#define DYNAMICA_ENUM_PARAM_H

#include "int_param.h"
#include <boost/foreach.hpp>

namespace dynamica {
    typedef std::map<std::string,std::pair<int,std::string> > EnumMap;
    /**
     * @brief an integer enum implementation of the parameter.
     *        This is an extension to the int parameter,
     *        which allows creating std::string aliases for certain (if not all) numbers available.
     *
     */
    class EnumParam : virtual public IntParam {
    public:

        void prepInfo(ParamInfoList &info_list);

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        EnumParam(const std::string &name, unsigned int level, const std::string &description,
                int def, const std::map<std::string,int> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        EnumParam(const std::string &name, unsigned int level, const std::string &description,
                const std::string& def, const std::map<std::string,int> &dictionary);

        #pragma clang diagnostic push
        #pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        EnumParam(const std::string &name, unsigned int level, const std::string &description,
               int def, const std::pair<EnumMap,std::string> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        EnumParam(const std::string &name, unsigned int level, const std::string &description,
               const std::string& def, const std::pair<EnumMap,std::string>  &dictionary);
        #pragma clang diagnostic pop

    protected:
        /** 
         * @brief A dictionary from the std::string aliases to their integer counterparts.
         * This method of storage allows integers to have multiple aliases.
         */
        const EnumMap dict_;
        /**
         * @brief this holds the physical enum's description. why is this here? because 1D-reconfigure.
         */
        std::string enum_description_;
    private:
        /**
         * converts the value given to an integer according to the embedded dictionary.
         * @param val the value to look up within the dictionary
         * @return if the value is a std::string which exists in the dictionary, returns the int definition of the term given.
         *         otherwise, returns the Value object defined conversion of the type to an integer.
         */
        int lookup(Value val);

        /**
         * generates the 'edit_method' sting for prepGroup().
         * @return a std::string that should not be touched.
         */
        std::string makeEditMethod();

        /**
         * generates a 'const' sting for prepGroup().
         * @param name the name of the constant
         * @param value the value of the constant
         * @param desc the description given to the constant.
         * @return a std::string that should not be touched.
         */
        std::string makeConst(std::string name, int value, std::string desc);
    };
}

#endif //DYNAMICA_ENUM_PARAM_H