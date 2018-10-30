# what is the name of the parameter? this is used to do an index search.
# this name is considered fully qualified, meaning it also includes its ancestors' names.
# format should be snake_case/another_snake/last_name
# root is an object, but all entries between the slashes can be fully alpha numerical.
# array indices would be fully numerical.
string name

# how severe will changing this file be? each bit in this is used as an arbitrary flag so use it to your advantage
uint32 level

# what does this parameter mean?
string description

# what base datatype should this parameter be parsed into?
# some options are int,double,string,array,etc...
string type