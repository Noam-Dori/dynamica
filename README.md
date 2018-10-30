Dynamica
==================================================

Dynamica is the successor to the ROS standard package [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure).
It includes the features included in its extensions 
[ddyanmic_reconfigure](https://github.com/awesomebytes/ddaynmic_reconfigure) 
and [3d_reconfigure](https://github.com/wew84/3d_reconfigure)

## Dependencies
Dynamica depends only on the default packages.

## Features
* all features from dyanmic_reconfigure, including their GUI
* persistance of parameters - any change you did to the parameters will be saved and reloaded
* extendable parameters via code - allows to give the parameters extra features
* the ability to 'lock' parameters - disable them from reconfiguration until unlocked.
* the ability to add and remove parameters to the reconfigure server during runtime.
* extnesion of the param types to a JSON-like configuration (array and object params)
* server & client models for **C++**, **python** (maybe **Java**, **Javascript**?)

## Configuration
Other than the installation of the package to your workspace, no other configuration is needed.
The package used is called ``dynamica``,
and this both the namespace and the include directory used to implement the program.

# FROM HERE STUFF IS NOT RIGHT AND WILL BE CHANGED IN DUE TIME

## Implementation
let us look into the following code, which implements Dynamica:
````cpp
#include <ros/ros.h>

#include <dynamica/dynamica.h>
#include <dynamica/param/dd_all_params.h>

using namespace dynamica;

void callback(const ParamMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(), get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;

    // DDynamic setup stage
    DDynamicReconfigure dd(nh);
    dd.add(new IntParam("int_param", 0, "An Integer parameter", 50, 0, 100));
    dd.add(new DoubleParam("double_param", 0, "A double parameter", .5, 0, 1));
    dd.add(new StringParam("str_param", 0, "A string parameter", "Hello World"));
    dd.add(new BoolParam("bool_param", 0, "A Boolean parameter", true));
    std::map<std::string, int> dict; // An enum to set size
        dict["Small"] = 0;      // A small constant
        dict["Medium"] = 1;     // A medium constant
        dict["Large"] = 2;      // A large constant
        dict["ExtraLarge"] = 3; // An extra large constant
    dd.add(new EnumParam("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
    dd.start(callback);

    // Actual Server Node code
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
````
This segment of code is used for declaring the configuration file and for setting up the server in place of the node which uses the parameters.

### Breakdown

Let's break down the code line by line:
```cpp
#include <ros/ros.h>

#include <dynamica/dynamica.h>
#include <dynamica/param/dd_all_params.h>

using namespace dynamica;
```
In here, we import all needed files:
* ``<ros/ros.h>`` provides basic ROS management.
* ``<dynamica/dynamica.h>`` provides the Dynamica API
* ``<dynamica/param/dd_all_params.h>`` allows you to use all default parameter types.

The non include line allows us to use classes and functions provided in the ``dynamica`` namespace
without mentioning what package they are from.

In contrast to dynamic_reconfigure, these do not change.

```cpp
void callback(const ParamMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(), get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}
```

This is the callback used when Dynamica receives a parameter change request.
It takes two parameters: the first is a map of the new configuration mapping from the name of the parameter to the actual parameter object,
and the second is the level, which is the highest level of severity caused by the parameter change.
This is calculated by applying the OR operator on all levels of the parameters that changed.

In this callback the level is not used, but we do print out the new configuration.

```cpp
int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
```

All this section do is initialise our ROS node and its handler.
This is default stuff you do anyways.

```cpp
    // DDynamic setup stage
    DDynamicReconfigure dd(nh);
    dd.add(new IntParam("int_param", 0, "An Integer parameter", 50, 0, 100));
    dd.add(new DoubleParam("double_param", 0, "A double parameter", .5, 0, 1));
    dd.add(new StringParam("str_param", 0, "A string parameter", "Hello World"));
    dd.add(new BoolParam("bool_param", 0, "A Boolean parameter", true));
```

This is we start using Dynamica. First, we initialise our Dynamica object.
Then, we start adding parameters to it. In Dynamica, adding parameters is not just a simple function,
but you have to add a parameter object (an instance of the abstract ``Param`` class).
Let's look into the param objects above to see some common factors:
* The type of the parameter is declared first by specifying ``new DDType()``.
  For example, adding a new int parameter is done by doing ``dd.add(new IntParam(...))``

* Within the param constructor, the first argument is the name of the parameter.
  For example, in our int parameter, the name is set to ``"int_param"``.

* The second argument is the level of the parameter, that is,
  what needs to be reset or redone in the software/hardware in order to reapply this parameter?
  Usually, the higher the level, the more drastic measures you need to take to re-implement the parameter.

* The third parameter is the description of the parameter. This is great for documentation and for commandline tools.

* The fourth parameter is the default value. Depending on the type of parameter, each may treat this argument differently.

* ``IntParam`` and ``DoubleParam`` have a fifth and sixth optional parameters: minimum and maximum allowed values.
  While the server side does not care about these values, the client may want to know these.

* It is important to note that the first 4 arguments are standardised for all param types,
  but from there onwards each param type may choose what to place there.

```cpp
    std::map<std::string, int> dict; // An enum to set size
        dict["Small"] = 0;      // A small constant
        dict["Medium"] = 1;     // A medium constant
        dict["Large"] = 2;      // A large constant
        dict["ExtraLarge"] = 3; // An extra large constant
    dd.add(new EnumParam("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
```

Here we add an int-enum parameter to our Dynamica. ``EnumParam`` is an int like parameter that also contains a dictionary
to remap predefined strings to usable integers. This param type has a required 5th argument (in contract to ``IntParam`` having 5th and 6th optional)
which is a ``std::map<std::string,int>`` object mapping string values to integers.

In the code above we can see how to create a dictionary of our liking:

* we first initiate a map and name it with ``std::map<std::string,int> dict``.
* we then populate it with the format ``dict[<key>] = <value>`` where ``<key>`` is the string alias for the value,
  and ``<value>`` is the value you want to give an alias to.

This dictionary is then added into the enum as the 5th argument.

```cpp
    dd.start(callback);

    // Actual Server Node code
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
```

This section of code actually allows Dynamica to start working. Let's look into the two sections:
* ``dd.start(callback)`` sets the callback of Dynamica to be the method ``callback`` and jump starts Dynamica.
* ``ros::spin()`` allows Dynamica to listen to parameter-change requests.
  Although the node now requires a spin, this does not mean you cannot add your own service-servers and subscribers to this node.
  ``ros::spin()`` can take care of multiple subscribers/service-servers in the same spinners (although in the same thread).
  If you want Dynamica and your actual node to work on separate threads, consider using ``ros::MultiThreadedSpinner``.
  Dynamica only uses 1 service-server and no subscribers, so 1 thread for it is more than enough.

### How does this compare with Dynamic-Reconfigure?
Let's go over the main differences between Dynamica's implementation with dynamic_reconfigure's implementation:

#### Parameter Generation

**dynamic_reconfigure:**
```python
from dynamic_reconfigure.parameter_generator_catkin import *

PACKAGE = "dynamic_tutorials"

gen = ParameterGenerator()

gen.add("int_param",    int_t,    0, "An Integer parameter", 50,  0, 100)
gen.add("double_param", double_t, 0, "A double parameter",    .5, 0,   1)
gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)

size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
                       gen.const("Medium",     int_t, 1, "A medium constant"),
                       gen.const("Large",      int_t, 2, "A large constant"),
                       gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
                     "An enum to set size")

gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

exit(gen.generate(PACKAGE, "dynamic_tutorials", "Tutorials"))
```
**Dynamica:**
```cpp
DDynamicReconfigure dd(nh);

dd.add(new IntParam("int_param", 0, "An Integer parameter", 50, 0, 100));
dd.add(new DoubleParam("double_param", 0, "A double parameter", .5, 0, 1));
dd.add(new StringParam("str_param", 0, "A string parameter", "Hello World"));
dd.add(new BoolParam("bool_param", 0, "A Boolean parameter", true));

std::map<std::string, int> dict; // An enum to set size
    dict["Small"] = 0;      // A small constant
    dict["Medium"] = 1;     // A medium constant
    dict["Large"] = 2;      // A large constant
    dict["ExtraLarge"] = 3; // An extra large constant

dd.add(new EnumParam("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));

dd.start(callback);
```
While these two code snippets accomplish the exact same things, they do so in different manners:
* dynamic_reconfigure specifies the type of the parameter using a string (for example ``int_t = "int"``), while Dynamica uses classes to accomplish that (``new IntParam`` in place of ``int_t``).
  
  Why classes instead of strings? In contrast to strings, classes can be extended and modified so they get a special treatment.
  Take enums for example. In order to work with enums, dynamic_reconfigure had to add a whole new parameter input to handle the dictionary of the enums,
  while Dynamica simply extended the ``IntParam`` class (to ``EnumParam``) to handle dictionaries.
  
  This will be discussed more thoroughly on "Architecture".
  
* Enums are dramatically different. 
    * Dynamica uses well defined standard C++ objects for its dictionaries,
      while dynamic_reconfigure defines its own constants and enums. This allows you to use well known and reliable API instead of a loosely defined one.
  
    * while ``EnumParam`` is an extension of ``IntParam``, you do not need to mention that. The API takes care of that for you!
      An added bonus of this is that the enums automatically inference their boundaries, you don't need to mention ``int_t, max, min``.
      
    * Dynamica's supported physical enums have been stripped of descriptions and the constants were as well.
      This is because the descriptions were not used anywhere. You can still make enums with const and enum descriptions, but they will not be used anywhere.
      Adding line comments to the parameters is a good alternative.

* Dynamica requires a node handler. This is due to how dynamic_reconfigure handles parameters in its ROS architecture for C++.

* all parameters provided in dynamic_reconfigure's last line are not needed in Dynamica, which requires nothing.

#### Callback & Server

**dynamic_reconfigure:**
```cpp
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");

  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
```
**Dynamica:**
```cpp
void callback(const ParamMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(), get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;

    DDynamicReconfigure dd(nh);
    
    dd.start(callback);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
```

* The callback of the dynamic_reconfigure requires a custom data-type per configuration. This is problematic, especially if you want dynamic parameters like vectors.
  Dynamica uses ``ParamMap`` as its parameter container, which is generic and can work over multiple config types (and therefore can handle vectors)
  
  This also changes the way to access these parameters.
  ``ParamMap`` is actually a map from string to a pointer to the generic parameter used with the parameter generator.
  This allows you to use all functions ``std::map`` provides, and regardless, Dynamica has additional API that could be used on ``ParamMap`` objects,
  such as ``dynamica::get`` and ``dynamica::at``.
  
  The generic interface no longer gives you a specific primitive value, but rather an instance of ``dynamica::Value``,
  which must be converted into a primitive type. While a bit cumbersome, this does allow rather implicit conversion between types.
  
* The Dynamica server does not internally initialise a node handler. This means you can implement Dynamica within the node that actually uses the parameters.

* Dynamica actually needs you to start it. While a disadvantage, it is done anyways on dynamic_reconfigure,
  and Dynamica also has ``DDynamicReconfigure::setCallback`` to change callbacks, so nothing much is lost.

* again, Dynamica does not require you to have a custom made config class, making your init code a lot shorter.

* ``start`` is a bit more fluid and allows you to place member function pointers or regular function pointers instead of ``boost::function``.
  This helps clean up the code.

### Simplified API

#### Value

The Value class is used to wrap all basic data-types (bool,int,double,string) in something generic.
The value object always stores an explicit basic data-type.
This has three main uses:

1. Values can represent all basic data-types. This means that arguments that need something relatively similar from all basic data-types can now just use the value in its argument.
   This also goes for when you need to return something that is of different data-types from different classes (one can only return integer, other can only return strings).

2. Values can be explicitly converted to all basic data-types they wrap. 
   This means that converting an int to a string is far easier.

3. Values store the type they were instantiated with. This can be tested against to get the original piece of data the value stored.

##### Constructors

``Value(int val)``,``Value(double val)``,``Value(bool val)``,``Value(string val)``,``Value(const char* val)``
are all constructors that assign the value type to the type they are given (with the exception for ``const char*`` which returns string and is there for convenience),
then store the value itself in its basic form.

##### Getter

There is only one true getter: ``getType()``, which returns the string name of the type it stores.

##### Converters

Each basic data-type has its own converter: ``toInt()``,``toDouble()``,``toBool()``,``toString()``.
When one is called, the value will attempt to return a converted form of what it stores into the required data-type.
The value does not just use an implicit cast. It tries to convert the data-type according to common needs that are not answered with other one-liners.
For example, converting a string to an int, a Value will first attempt to scan the string and see it fits a numeric format.
If it succeeds, it will convert and return that number. Otherwise, it will return the next best thing: a hash value of the string.

#### Param

The Param class is *the* abstraction of all parameter types, and is the template for creating them.
At this point, not much is known about the parameter, but the following:

* the parameter has a name
* the parameter has a severity level
* the parameter has a description
* the parameter contains some value, though its type and contents are unknown.

Other than storing data, the parameter also has specialised methods to interact with DDynamicReconfigure in order to apply changes and send them.
These methods should not be touched by the user.

Since this class is abstract, the class has multiple implementations whicch are not directly exposed but are used,
so its worth checking out their descriptions.

While this class is abstract, it does have one implemented thing, and that is its stream operator (`<<`) which can be freely used.

##### Generic Constructor

While Param is abstract, all of its concrete implementations should follow this guideline:
```cpp
DD<Type>(const string &name, unsigned int level, const string &description, <some-type> def, <extra-args>)
```
Where:
* ``<Type>`` is the type name you are implementing
* ``name`` is the reference name
* ``level`` is the severity level
* ``description`` is the object's description
* ``def`` is the default value and the first value stored right after construction.

You may then include extra arguments as you wish, required or optional.

##### Getters

parameters have three well known getters:
* ``getName()`` gets the name of the parameter.
* ``getLevel()`` gets the severity level of the parameter.
* ``getValue()`` gets the value the parameter stores.

Other getters, such as "getDesc()", may be added in the future.

the parameters also have a stream (``<<``) operator which can be used to convert said parameters into neat strings.

##### Setter

2D-params are only required to be dynamic with their values,
so the only setter they are required to have is ``setValue(Value val)``,
which changes the value the parameter stores.

##### Testers

DDParams are also required to have some out-of-the-box testing features:
* ``sameType(Value val)`` checks whether or not 
  the raw value stored in the value is compatible with the given parameter.
  Compatible is a very broad word in this scenario.
  It means that the value can be placed in the parameter regardless of other limitations.

* ``sameValue(Value val)`` checks whether or not the value stored in the value object,
  when converted to the type of the internal value, are equal. This acts regardless of type.

#### DDynamicReconfigure

The DDynamicReconfigure class is the main class responsible for keeping track of parameters basic properties,
values, descriptions, etc.

It is also responsible of handling callbacks, config change requests, description setup and config setup, and the ROS publishers and services.

To operate a DDynamic instance, you must go through the following procedure:

1. Construct a DDynamicReconfigure instance with proper handling.
2. Add parameters to the instance as needed with any of the ``add`` methods.
3. Start the ROS services with any of the ``start`` methods.
4. If you need to change the callback after startup you may do so using ``setCallback``.
5. When you need to get any of the stored parameters, call either ``get`` or ``at`` on this instance,
   rather than through the callback.

##### Constructor

DDynamicReconfigure has one sole constructor: ``DDynamicReconfigure(NodeHandle &nh)`` which constructs the instance and
sets the handler to the one you are using.

##### Parameter Handling

All parameter handling is done through registration using an ``add`` function:

* ``add(ParamPtr param)`` is the main function which uses boost's shared pointers to represent the data in a virtual manner (and allows polymorphism)
* ``add(Param *param)`` is a convenience function which converts ``param`` into a shared pointer and uses the other add function.

Both of these functions will add a generic ``Param`` object into the given instance and will index it for later searches.
Perhaps in the future a "remove(string name)" function will be added.

##### Callback Handling & Startup

Below are the two default functions that are used by the rest:

* ``start()`` initializes all publishers and services and releases the needed messages for the commandline and other clients.
* ``setCallback(ReconfigureFunction callback)`` sets the triggered callback to the one specified, and triggers nothing else.

There is also ``clearCallback()`` which resets the callback to do nothing when triggered.

Following are convenience function which utilize ``start()`` and ``setCallback()``:

* ``start(ReconfigureFunction callback)`` calls start(), then setCallback(callback)
* ``start(void(*callback)(const ParamMap&, int))`` remaps the void pointer to a boost function (of type ``ReconfigureFunction``) then calls start(callback)
* ``template<class T> void start(void(T::*callback)(const ParamMap&, int), T *obj)``
  binds the **member** function into a boost function (of type ``ReconfigureFunction``) then calls start(callback)

##### Parameter Fetching

There are multiple proper ways to get the values stored within the DDynamicReconfigure instance:

* through ``at(string name)``: this will get you the pointer to the parameter with the name you specified.
  If no such parameter exists it will return you a null-pointer (be careful not to de-reference those!)

* through ``get(string name)``: this will get you the value stored in the parameter with the name you specified.
  If no such parameter exists it will return you a value storing a NULL character.

* through the stream (``<<``) operator: this will convert the Dynamica instance into a string and stream it into the
  given streamer.

both ``at`` and ``get`` have alternate static versions which apply directly on ``ParamMap`` objects.

## Architecture

### Code Design

#### Include Structure:

![](http://www.plantuml.com/plantuml/png/3OnDIuP054Rt_efQjCm9JB8WqcXJ44Nu4hIH-RZgu7p8dNiT_FSDFAk7SqwVI2AnTzMr3Tgn0KPtjHBjwKa8bBbUBAsiE07g60W2rJfw8JEaw46T14aOSmRfhPuG2ZFRXH54XjpTtneuHAcBsJgO4Y5hglTol53S83mFxpzt-FNuyA7KvLDVpAiST3isgg6vu-_VRakj-ZlMCGytpLjPrKCmHVy7)

To operate Dynamica, you will need to include 2 file types:

* The ``dynamica`` file, which gives you access to the ``DDynamicReconfigure`` class,
  the ``Param`` class, the ``DDValue`` class, and the toolbox methods.
  This will allow you to operate on the top level API without caring about what type of parameters you will get.

* the file ``dd_all_params`` or any of the ``Param`` implementations. You will need the implementations to insert physical 
  (and not abstract) parameters into your ``DDynamicReconfigure`` server.
  As a shortcut, ``dd_all_params`` gives you all basic parameter types (int,double,bool,string,enum) in one include.

As a bonus, you also get two static class-less methods: ``get`` and ``at``.

#### Class Structure:

![](http://www.plantuml.com/plantuml/png/3OnBIyD054Rt_HMwS6d6HmDH43EIZRfgmOBTX7dS96Fc4Uwzqw7_te5lzN7EwOaLSWv-T-kYyTb2Hd-pC6_qAWIgqioEbwmp0PeK6I8t9WMX2b0AeAyC9AozHXMS6H4gCxav8uW2fTlVMxY8MXV6AwAH6BFXPglFEwSLuflyF3wWXDFJYlmrdDpXyNl_yN9e_xhsz-UevKgjFbzWAlBkUQZRzH1jrVy1)

Like the API section shows, there are only 3 major classes: ``DDValue``,``Param``,``DDynamicReconfigure``.

The DDValue class is a concrete class which should not be inherited, since it wraps physical values. 
Each instance stores 5 values: one for each type is can handle, and one to store the type.
When a value is instantiated, the value is stored in its raw form according to the chosen type,
and the rest stay with default values. When the value is accessed only then is the value converted (but not saved!)

The Param interface class is an abstract class which should be implemented. 
Its basic implementations (int,double,bool,string) have already been implemented in the standard package.
These basic forms can also be further extended. For example, EnumParam **extends** IntParam because it has all of the features IntParam has.
This can be done to other Param implementations, and you can also further extend the extended classes (for example, DDInvertibleEnum).
An example is given at the Extension section if you want to look more into this.
When anny Param implementation is extended, the user has access to everything within the object so that he can do what he needs to.

The DDynamicReconfigure class is the concrete class that does the work against ROS and interfaces with the user.
Unlike DDValue, this class can be extended, and it has an internal API that can aid users who wish to extend this class.
In the Extension section below this is elaborated. Keep in mind that extending DDynamicReconfigure is not required.
While DDynamicReconfigure allows extension, it does not provide full access to everything,
since the base function of DDynamic should not be modified.

### ROS Design

![](http://www.plantuml.com/plantuml/png/3OnDIyKm44Nt_HMwS6aZg222s8BWgo8QGOHkGZwcRMoJb9b9m_lt1kxcmZcd8zR8EMpDfOzsomuoRXSByqwFGg0kxUnvoIOJe4sH8N9hKn2w0AK0vin0mhbprC5RXL2PoSyPGHGe3tVN3WvHwm8JAMBCbjkz_cTEAyIdVlY-mTFRwxiCgAIHu_JpgORVrG38hzF7ljAz6Oy_MVghsvUwfeFegluF)

Like dynamic_reconfigure, Dynamica is built on two subscribers and one service:

* ``desc_pub_`` publishes to topic "/parameter_descriptions", and is responsible for updating the descriptions of the parameter for commandline.
* ``update_pub_`` publishes to "/parameter_descriptions", and is responsible for updating the configuration values for commandline and client.
* ``set_service`` publishes and listens to requests on "/set_parameters", and is used to trigger parameter updates.
  It also contains the new parameters sent from client or commandline.

Since the DDynamicReconfigure object is held on the server side, so are these ROS entities.

## Extension

***In all of these extensions, make sure to add the proper includes!***

### Adding a new Parameter type

To add a new parameter type, you must either:
* Extend one of the existing classes
* Implement the base class, ``Param``.

In some cases, you might want your class to extend multiple classes, for example ``DDIntVector`` both implements ``DDVector`` and extends ``IntParam``.
(``DDVector`` does not exist in the standard param library).

Let us look into an example implementation of the param type "DDIntEnforcer", which will update other parameters to its value when it updates.

```cpp
#ifndef dynamica_INT_ENFORCER_PARAM_H
#define dynamica_INT_ENFORCER_PARAM_H

#include <dynamica/param/dd_int_param.h>
#include <list>

namespace my_dd_reconfig {
    // class definition
    class DDIntEnforcer : public IntParam {
    public:

        void setValue(Value val);
        
        // adds a parameter to be enforced by this param.
        DDIntEnforcer &addEnforced(ParamPtr param);
        
        // removes a parameter from being enforced by this param.
        void removeEnforced(ParamPtr param);

        /**
         * creates a new int enforcer param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to INT32_MAX
         * @param min the minimum allowed value. Defaults to INT32_MIN
         */
        inline DDIntEnforcer(const string &name, unsigned int level, const string &description,
                int def, int max = INT32_MAX, int min = INT32_MIN) :
                IntParam(name,level,description,def) {};

    protected:
        list<ParamPtr> enforced_params_;
    };
    
    DDIntEnforcer::setValue(Value val) {
        val_ = val.toInt();
        for(list<ParamPtr>::iterator it = enforced_params_.begin(); it != enforced_params_.end(); ++it) {
            if(!enforced_params_[it].sameValue(val)) {
                enforced_params_[it].setValue(val);
            }
        }
    };
    
    DDIntEnforcer &DDIntEnforcer::addEnforced(ParamPtr param) {
        enforced_params_.push_back(param);
        return *this;
    };
    
    void DDIntEnforcer::removeEnforced(ParamPtr param) {
        enforced_params_.remove(param);
    };
}

#endif //dynamica_INT_ENFORCER_PARAM_H
```

Notice how nothing within this class is private. This allows further extension of this class.
Moreover, notice that in here we are also using variables inherited from ``IntParam``, specifically ``val_``.

### Extending DDynamic's functions

Extending DDynamicReconfigure means that you need additional functionality from the parameter server which Dynamica does not provide.
If that is the case, extending a class from DDynamic gives you access to make new methods as need for the extra functionality,
and access to the following to make work with DDynamic a bit easier:
* ``nh_``: this is the node handler used to create all publishers and subscribers in the parent class.
* ``params_`` this is the current parameter map Dynamica uses to update parameters and add new ones.
* ``desc_pub_``: As explained before, this is the publisher responsible of updating the descriptions for the parameters and other metadata for the client and commandline.
* ``update_pub_``: This is the publisher responsible for updating the configuration values for the client and commandline.
* ``makeDescription()``: This is a helper method that generates a new Description message to be published by ``desc_pub_``.
  The message can be modified.
* ``makeConfiguration()``: This is a helper method that generates a new Description message to be published by ``update_pub_``.
  The message can be modified.
* ``internalCallback()``: This is a helper method that allows you to call the base param change callback built into Dynamica.

From there, it's your choice what to do with these.