# what is the name of the parameter? this is used to do an index search.
# this name is considered fully qualified, meaning it also includes its ancestors' names.
# format should be snake_case/another_snake/last_name
# root is an object, but all entries between the slashes can be fully alpha numerical.
# array indices would be fully numerical.
string name

string OBJECT = ##obj##
string ARRAY = ##arr##
string DELETE = ##del##

# generic value for this parameter. see above for default indicators
string value