
# This expresses a program send from nlp_node after processing a sentence to be grounded in database
std_msgs/Header header                  # timestamp in the header is the acquisition time and frame

string[] data                  # sentence program as string

# dict: 'actions': [''], 'action_probs': [0.3, 0.2, 0.1], 'action_timestamps': [169.., 169.., 169..]
#       'objects': [''], 'object_probs': [0.3, 0.2, 0.1], 'object_timestamps': [169.., 169.., 169..]
# 'object_classes': ['cup', 'potted meat can', '<Ontology URI>']
#       'storages:' [''], 'storage_probs': [], 'storage_timestamps': []
#       'parameters': [], 'parameter_values': [], 'parameter_timestamps': []
# 

