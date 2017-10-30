
# This message contains the object drone role, formulated by the multi-agent
# strategy and read by each drone's IA.

# Drone name
string name

# Role code
float64 role

# List of input-output topic names. Refer to the role code for convention about
# the specific use of each field
string[] params
