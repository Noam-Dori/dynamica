//
// Created by Noam Dori on 30/10/18.
//
#include <ros/ros.h>

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "param.h"

#ifndef DYNAMICA_DYNAMICA_H
#define DYNAMICA_DYNAMICA_H
namespace dynamica {
    // this is a map from the Param name to the object. Acts like a set with a search function.
    typedef std::map<std::string,ParamPtr> ParamMap;
    // the function you will use a lot
    typedef boost::function<void(const ParamMap&,int)> ReconfigureFunction;

    /**
     * @brief The Dynamica class is the main class responsible for keeping track of parameters basic properties,
     *        values, descriptions, etc.
     *
     *        It is also responsible of handling callbacks, config change requests, description setup and config setup,
     *        and the ROS publishers and services.
     *
     *        To operate a Dynamica instance, you must go through the following procedure:
     *
     *        1. Construct a Dynamica instance with proper handling.
     *        2. Add parameters to the instance as needed with any of the "add" methods.
     *        3. Start the ROS services with any of the "start" methods.
     *        4. If you need to change the callback after startup you may do so using "setCallback".
     *        5. When you need to get any of the stored parameters, call either "get" or "at" on this instance,
     *           rather than through the callback.
     */
    class Dynamica {
    public:
        /**
         * @brief creates the most basic instance of a 2d-conf object.
         * @param nh the node handler of the node this is placed at.
         */
         explicit Dynamica(ros::NodeHandle &nh);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to add to the list.
         */
         virtual void add(ParamPtr param);

        /**
         * @brief a convenience method for adding a parameter to the list, allowing it to be generated.
         * @warning When adding in this manner, you must be careful. After using this method to add the parameter,
         *          running any of the "remove" methods on this object WILL cause the entire param object to be deleted!
         *          To make sure that you can add the object back after removing it, please use the other "add" method.
         * @param param the pointer to the 2d-param to add to the list.
         */
         virtual void add(Param *param);

        /**
         * removes the specified parameter from the list.
         * @param param the parameter to remove.
         */
         virtual void remove(ParamPtr param);

        /**
         * removes the specified parameter from the list.
         * @param param the parameter to remove.
         */
         virtual void remove(Param *param);

        /**
         * removes the specified parameter from the list.
         * @param param_name the name of the parameter to remove.
         */
         virtual void remove(std::string param_name);

        /**
         * @brief sets the callback to this.
         * @param callback a boost function with the method to use when values are updated.
         */
         void setCallback(ReconfigureFunction callback);

        /**
         * @brief sets the callback to be empty.
         */
         void clearCallback();

        /**
         * @brief starts the server and config, without having any callback.
         */
         virtual void start();

        /**
         * @brief starts the server, using the given callback in function form.
         * @param callback a boost function with the method to use when values are updated.
         */
         void start(ReconfigureFunction callback);

        /**
         * @brief starts the server, using the given callback in method form.
         * @param callback a void pointer accepting a callback type with the method to use when values are updated.
         */
         void start(void(*callback)(const ParamMap&, int));

        /**
         * @brief starts the server, using the given callback in class-wise static method form.
         * @param callback a class void pointer accepting a callback type with the method to use when values are updated.
         * @param obj the object used for reference in the class void
         * @tparam T the class of the object.
         */
         template<class T>
         void start(void(T::*callback)(const ParamMap&, int), T *obj) {
            ReconfigureFunction f = boost::bind(callback,obj,_1,_2);
            start();
         }

        /**
        * @brief a tool people who use this API can use to find the value given within the param map.
        * @param name the string to look for
        * @return the value of param with the given name if it exists, a string value containing "\000" otherwise
        */
        Value get(const char* name);

        /**
        * @brief a tool people who use this API can use to find the param given within the param map.
        * @param name the string to look for
        * @return the param with the given name if it exists, nullptr otherwise
        */
        ParamPtr at(const char* name);

        /**
         * @brief the operator taking care of streaming the param values
         * @param os the stream to place the param into
         * @param dd the dd-reconfigure you want to place into the stream
         * @return os, but with dd-reconfigure added.
         */
        friend std::ostream& operator<<(std::ostream& os, const Dynamica &dd);
    protected:

        /**
         * @brief makes the config descriptions for publishing
         * @return a ROS message of type ConfigDescription
         */
         ConfigDescription makeDescription();

        /**
         * @brief makes the config update for publishing
         * @return a ROS message of type Config
         */
         Config makeConfig();

        /**
         * @brief calls the internal callback for the low-level service, not exposed to us.
         * @param obj the object we are using for its callback.
         * @param req ----(ROS)
         * @param rsp ----(ROS)
         * @return -------(ROS)
         * @note this is here so that deriving methods can call the internal callback.
         */
        static bool internalCallback(Dynamica *obj, Reconfigure::Request &req, Reconfigure::Response &rsp);

         /**
          * @brief the ROS node handler to use to make everything ROS related.
          */
         ros::NodeHandle nh_;
         /**
          * @brief a map of all contained parameters.
          */
         ParamMap params_;
         /**
          * @brief the publisher responsible for updating the descriptions of the parameter for commandline (desc_pub_),
          * and the publisher responsible for updating the configuration values for commandline and client (update_pub_).
          * desc_pub_ publishes to "parameter_descriptions", and update_pub_ publishes to "/parameter_updates".
          */
         ros::Publisher desc_pub_, update_pub_;

    private:

        /**
         * @brief reassigns a value to the internal map assuming it is registered.
         * @param map the map that is being edited
         * @param name the name of the parameter to test
         * @param value the value of the new parameter
         * @tparam T the type of value
         * @return -1 if the value could not be reassigned,
         *         0 if the value was not changed,
         *         otherwise the level of the parameter changed.
         */
         template <class T>
         static int reassign(ParamMap& map, const string &name, T value);

        /**
         * @brief gets the updates and assigns them to DDMap
         * @param req the ROS request holding info about the new map
         * @param config the map to update
         * @return the level of change (integer)
         */
         int getUpdates(const Reconfigure::Request &req, ParamMap &config);

         /**
          * @brief the use defined callback to call when parameters are updated.
          */
          boost::shared_ptr<ReconfigureFunction> callback_;
          #pragma clang diagnostic push // CLion suppressor
          #pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
         /**
          * @brief the service server used to trigger parameter updates.
          *        It also contains the new parameters sent from client or commandline.
          */
          ros::ServiceServer set_service_;
          #pragma clang diagnostic pop
    };

    /**
     * @brief a tool people who use this API can use to find the param given within the param map.
     * @param name the string to look for
     * @param map the map to search
     * @return the param with the given name if it exists, nullptr otherwise
     */
     ParamPtr at(const ParamMap& map, const char* name); // I could do this with an operator, but its bad design.

    /**
     * @brief a tool people who use this API can use to find the value given within the param map.
     * @param name the string to look for
     * @param map the map to search
     * @return the value of param with the given name if it exists, a string value containing "\000" otherwise
     */
     Value get(const ParamMap& map, const char* name); // I could do this with an operator, but its bad design.
}
#endif //DYNAMICA_DYNAMICA_H