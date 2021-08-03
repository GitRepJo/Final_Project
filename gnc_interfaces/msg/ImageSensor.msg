# Store the data of a compressed image
# Can be used instead of the default message type if no header is required

# Time as specified in Unity
int32 time

# Specifies the format of data.
# Acceptable values are: jpeg, png
string format 

# new data is available
bool newdata

# datapath to image saved by unity3d
string datapath

# Compressed Image Buffer
uint8[] data

# Use this field instead the uint8[] data as byte serialization is not supported by the rosbridge (06.2021)
int16[] doublebytedata
