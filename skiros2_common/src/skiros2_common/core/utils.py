try:
    basestring
except NameError:
    basestring = str

def ontology_type2name(ontology_type):
    if isinstance(ontology_type, list):
        return [ontology_type2name(t) for t in ontology_type]
    elif isinstance(ontology_type, basestring):
        return ontology_type.split(':')[-1]
    else:
        return ontology_type


def ontology_type2name_dict(ontology_type):
    if isinstance(ontology_type, basestring):
        ontology_type = [ontology_type]
    return dict(zip(ontology_type2name(ontology_type), ontology_type))
