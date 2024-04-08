from typing import Type, Optional, List, Tuple
from abc import ABCMeta
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier


ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")
OWLR = Namespace("http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl#")


def _flatten_result(res: List) -> str:
    return ' .\n'.join([', '.join([str(elm) for elm in line]) for line in res])


def _get_class_by_python_name(cls_name: str) -> str:
    res = BaseClass._onto_communicator.query(
        f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            SELECT ?sub WHERE {{
                ?sub {OWLR['python_name'].n3()} '{cls_name}' .
                ?sub {RDF.type.n3()} {OWL.Class.n3()} .
            }}"""
    )
    if isinstance(res, tuple):
        raise RuntimeError(f"Querying for class {cls_name} resulted in an error: {res}")

    print(f"Classes with python name '{cls_name}': {_flatten_result(res)}")
    if len(res) > 1:
        raise ValueError(f"Multiple classes share the same python name '{cls_name}'! Returned classes: {_flatten_result(res)}")
    elif len(res) == 0:
        raise ValueError(f"There is no class with python name '{cls_name}'!")
    return next(iter(res))[0]


class OntoCommunicator(metaclass=ABCMeta):
    """Any sort of class that provides connection to any ontology.
    This is to shield the rest of the API from the used ontology,
    architecture, protocols, etc.
    Just implement the 'query' method to receive a SPARQL query
    string and return a response.
    """

    def query(self, query_string) -> str:
        raise NotImplementedError()


class BaseClass():
    _onto_communicator: Optional[OntoCommunicator] = None

    def __init__(self):
        if self.__class__ == BaseClass:
            raise RuntimeError("Cannot instantiate the BaseClass class directly!")
        if self._onto_communicator is None:
            raise ValueError("Before using any Onto Class, use 'BaseClass.set_onto_communicator' to set onto backend!")
        self.__uri: URIRef = self._prepare_uri()

    @classmethod
    def set_onto_communicator(cls, onto_communicator: OntoCommunicator) -> None:
        if cls._onto_communicator is not None:
            raise ValueError(f"Onto communicator was already set to {cls._onto_communicator} in this script. Setting it multiple times might cause various issues. Remove this at your own discretion!")
        cls._onto_communicator = onto_communicator

    @property
    def uri(self) -> URIRef:
        return self.__uri

    def _get_base_cls_name(self) -> str:
        cls_name = str(self.__class__)
        return cls_name[cls_name.rfind(".") + 1:cls_name.rfind("'")]

    def _prepare_uri(self) -> Literal:
        """Either get URI for this class from the ontology (if it exists)
        or generate new URI (if this is a new class) and send it to the ontology!
        """
        raise NotImplementedError(f"The class {self.__class__} must define '_prepare_uri' method!")


class ExistingClass(BaseClass):

    def __init__(self):
        if self.__class__ == ExistingClass:
            raise RuntimeError("Cannot instantiate the ExistingClass class directly!")
        super().__init__()

    def _prepare_uri(self):
        base_cls_name = self._get_base_cls_name()
        print(f"Loading uri of {base_cls_name}")
        existing_uri = _get_class_by_python_name(base_cls_name)
        return existing_uri


class NewClass(BaseClass):

    def __init__(self):
        if self.__class__ == NewClass:
            raise RuntimeError("Cannot instantiate the NewClass class directly!")
        super().__init__()


class OntoSubClass():

    def __new__(cls, new_class_name: str, parent_class: Type) -> Type:
        """Returns a subclass of the given parent class.
        Careful, this does not work as normal Python class.

        ```new_class = OntoSubClass("new_class", SomeParentClass)```

        In the line above, `new_class` is not an instance but a class/type.
        To get an instance, use:

        ```obj = new_class()```

        This behavior is so that you can create a subclass from this (sub)class.
        For example:
        ```
        new_class = OntoSubClass("new_class", SomeParentClass)
        even_newer_class = OntoSubClass("even_newer_class", new_class)
        ```

        Args:
            new_class_name (str): Name of the new class. Remember, this is used to make entity in the ontology!
            parent_class (Type): Parent Python class that inherits from BaseClass.

        Returns:
            Type: A type defining the new class.
        """
        def constructor(self):
            super(self.__class__, self).__init__()

        assert BaseClass in parent_class.mro(), f"The parent class '{parent_class}' must inherit from BaseClass!"
        subclass = type(new_class_name, (NewClass, parent_class), {"__init__": constructor})
        return subclass

    def __init__(self):
        """This is just to prevent some unwanted hacks like calling __init__ directly. Abusing Python is bad!"""
        raise RuntimeError("How did you get here?")
