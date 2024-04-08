from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
from rdflib.plugins.stores.sparqlstore import SPARQLUpdateStore, Store, _node_to_sparql

import rdflib
from rdflib import BNode, URIRef, Literal
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.plugins.sparql.processor import prepareQuery
from time import time, sleep
import timeit
import numpy as np
import warnings
import datetime as dt
from uuid import uuid4
from scipy.spatial import Delaunay
from datetime import datetime


CROW = Namespace(f'http://imitrob.ciirc.cvut.cz/ontologies/crow#')

ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"


class SpeedTester():
    CROW_OBJECTS = ["cube_holes", "sphere_holes", "wheel", "wafer", "peg_screw", "screw_round", "screwdriver", "hammer", "wrench"]
    CROW_TEMPLATES = ["CUBE", "SPHERE", "WHEEL", "WAFER", "PEG", "SCREW", "FLATSCREWDRIVER", "HAMMER", "WRENCH"]
    SEPLEN = 30

    def __init__(self):
        self.crowracle = CrowtologyClient()
        self.onto = self.crowracle.onto
        onto_namespaces = self.crowracle.onto.namespaces
        if "crow" not in onto_namespaces.keys():
            onto_namespaces["crow"] = self.crowracle.CROW
        replacement_ns = {}
        for ns, uri in onto_namespaces.items():
            if ns == "base":
                continue
            uri = str(uri)
            replacement_ns[uri] = ns + ":"
            if uri.endswith("#"):
                replacement_ns[uri[:-1] + "/"] = ns + ":"
            if uri.endswith("/"):
                replacement_ns[uri[:-1] + "#"] = ns + ":"

    def run_prep_and_func(self, prep_data_function, main_funcion):
        self.SEPLEN
        print("#" * self.SEPLEN)
        print("Testing the runtime of the data preparation function (subtract this from the total runtime)...")
        prep_func = speed_test(prep_data_function)
        r_mean, r_median, r_min, r_max = prep_func()
        print("-" * self.SEPLEN)
        print("Running the main test function...")
        @speed_test
        def run_function():
            data = prep_data_function()
            if type(data) is not tuple:
                main_funcion(data)
            else:
                main_funcion(*data)
        run_function(r_mean, r_median, r_min, r_max)
        print("#" * self.SEPLEN)

    def get_me_an_object(self):
        choice = np.random.choice(len(self.CROW_OBJECTS))
        object_name = self.CROW_OBJECTS[choice]
        location = np.random.randn(3) * 5
        size = (np.random.rand(3) * 2 + 0.5) / 10
        uuid = str(uuid4())
        timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        template = self.crowracle.CROW[self.CROW_TEMPLATES[choice]]
        adder_id = np.random.randint(0, 2**32)
        tracked = True
        return object_name, location, size, uuid, timestamp, template, adder_id, tracked

    def get_no_template(self):
        object_name, location, size, uuid, timestamp, template, adder_id, tracked = self.get_me_an_object()
        return object_name, location, size, uuid, timestamp, adder_id, tracked

    def get_me_an_object_around(self, x=0, y=0, z=0, sigma=0.2):
        choice = np.random.choice(len(self.CROW_OBJECTS))
        object_name = self.CROW_OBJECTS[choice]
        location = np.r_[np.random.randn(2) * sigma, np.random.rand() * 0.03] + np.r_[x, y, z]
        size = (np.random.rand(3) * 2 + 0.5) / 10
        uuid = str(uuid4())
        timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        template = self.crowracle.CROW[self.CROW_TEMPLATES[choice]]
        adder_id = np.random.randint(0, 2**32)
        tracked = True
        return object_name, location, size, uuid, timestamp, template, adder_id, tracked

    def add_object(self):
        object_name, location, size, uuid, timestamp, template, adder_id, tracked = self.get_me_an_object()
        self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, template, adder_id, tracked)
        return self.crowracle.CROW[object_name + '_od_'+str(adder_id)]

    def add_and_gen_update(self):
        obj = self.add_object()
        update_location = np.random.randn(3) * 5
        update_size = (np.random.rand(3) * 2 + 0.5) / 10
        update_timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        update_tracked = bool(np.random.randint(0, 2))
        return obj, update_location, update_size, update_timestamp, update_tracked

def test_in_hull(p, points):
    """Test if point p is in hull defined by points, using Delaunay triangulation
    Args:
        p (list): tested point, 3d
        points (list of lists): polyhedron points, 3d
    Returns:
        result (bool): Whether or not the point is inside polyhedron
    """
    if not isinstance(points,Delaunay):
        hull = Delaunay(points)
    return hull.find_simplex(p)>=0
# %% CROWTOLOGY

def main(args=[]):
    pass

tester = SpeedTester()
onto = tester.onto
crw = tester.crowracle

# %% AREA SORTING
crw.reset_database()
areas = [
    [
        [0, 0],
        [0.4, 0],
        [0.4, 0.2],
        [0, 0.2],
    ],
    [
        [1, 1],
        [1.5, 1],
        [1.5, 2],
        [1, 2],
    ],
    [
        [-1, -2],
        [-0.3, -2.5],
        [-0.5, -1.5],
        [-1, -1],
    ],
]
for i, area in enumerate(areas):
    polygon = [(np.array(li) + np.random.randn(2) * 0.03).tolist() for li in area]
    polygon_shifted = [(np.array(li) + np.random.randn(2) * 0.03 - 5).tolist() for li in area]
    crw.add_storage_space_flat(f"area_{i}", polygon, isMainArea=True)
    crw.add_storage_space_flat(f"area_{i}_shifted", polygon_shifted, isMainArea=True)
print("Added these areas:")
print(crw.getStoragesProps())
print("Adding not main areas...")
for i, area in enumerate(areas):
    polygon = [(np.array(li) + np.random.randn(2) * 0.05 + 5).tolist() for li in area]
    crw.add_storage_space_flat(f"fake_area_{i}", polygon)

print("Adding objects to areas...")
list_of_objs_per_area = {}
for adict in crw.getStoragesProps():
    area_uri = adict["uri"]
    area_name = adict["name"]
    if "shifted" in area_name:  # do not add objects to shifted areas
        continue
    list_of_objs_per_area[area_name] = []
    centroid = crw.get_area_centroid(area_uri)
    # generate objects around center
    sigma = 0.05
    for i in range(10):
        object_name, location, size, uuid, timestamp, template, adder_id, tracked = tester.get_me_an_object_around(*centroid, sigma=sigma)
        crw.add_detected_object_no_template(object_name, location, size, uuid, timestamp, adder_id, tracked)
        print(f"Added object at {location}")
        obj = crw.CROW[object_name + '_od_'+str(adder_id)]
        list_of_objs_per_area[area_name].append(obj)

    onto_objs_by_name = crw.get_objects_from_area(area_name)
    onto_objs_by_uri = crw.get_objects_from_area(area_uri)
    print(sorted(list_of_objs_per_area[area_name]))
    print(sorted(onto_objs_by_name))
    print(sorted(onto_objs_by_uri))

print("Testing if sorting objects into areas is working...")
current_objects = crw.getTangibleObjects()
crw.pair_objects_to_areas() #crw.pair_objects_to_areas_wq()

input("Done")

# %% shift objects
shift = 5
batch_update = []
tmsg = crw.node.get_clock().now().to_msg()
timestamp = datetime.fromtimestamp(tmsg.sec+tmsg.nanosec*(1e-9)).strftime('%Y-%m-%dT%H:%M:%SZ')
for obj, x, y, z in onto.query(_query_present_tangible_location):
    batch_update.append((obj, np.r_[x.toPython() + shift, y.toPython() + shift, z.toPython() + 0.03], [0.1, 0.1, 0.1], timestamp, True))

crw.update_batch_all([], batch_update)


# %%
_query_area_objs = """
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

SELECT DISTINCT ?obj ?obj_x ?obj_y ?obj_z ?area ?area_x ?area_y ?area_z
FROM <http://imitrob.ciirc.cvut.cz/ontologies/crow>
WHERE {
  {
    		?obj crow:hasId ?obj_id .
            ?obj crow:hasAbsoluteLocation ?obj_loc .
            ?obj_loc crow:x ?obj_x .
            ?obj_loc crow:y ?obj_y .
            ?obj_loc crow:z ?obj_z .
  } UNION {
            ?area a crow:StorageSpace .
            ?area crow:hasPolyhedron ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?area_x .
            ?pt crow:y ?area_y .
            ?pt crow:z ?area_z .
            FILTER EXISTS { ?area crow:areaType 'main' }

  }
}"""

_query_main_areas = """
SELECT DISTINCT ?area ?x ?y ?z

WHERE {
    ?area a crow:StorageSpace .
    ?area crow:hasPolyhedron ?poly .
    ?poly crow:hasPoint3D ?pt .
    ?pt crow:x ?x .
    ?pt crow:y ?y .
    ?pt crow:z ?z .
    FILTER EXISTS { ?area crow:areaType 'main' }
}"""

_query_present_tangible_location = """SELECT DISTINCT ?obj ?x ?y ?z
WHERE {
		?obj crow:hasId ?id .
    ?obj crow:hasAbsoluteLocation ?loc .
    ?loc crow:x ?x .
    ?loc crow:y ?y .
    ?loc crow:z ?z .
}"""

# %% PAIR TO AREAS
r = list(onto.query("""
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

SELECT DISTINCT ?obj ?obj_x ?obj_y ?obj_z ?area ?area_x ?area_y ?area_z
FROM <http://imitrob.ciirc.cvut.cz/ontologies/crow>
WHERE {
  {
    		?obj crow:hasId ?obj_id .
            ?obj crow:hasAbsoluteLocation ?obj_loc .
            ?obj_loc crow:x ?obj_x .
            ?obj_loc crow:y ?obj_y .
            ?obj_loc crow:z ?obj_z .
  } UNION {
            ?area a crow:StorageSpace .
            ?area crow:hasPolyhedron ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?area_x .
            ?pt crow:y ?area_y .
            ?pt crow:z ?area_z .
            FILTER EXISTS { ?area crow:areaType 'main' }

  }
}"""))
a = np.r_[r]
idx_objs = a[:, 0].astype(bool)
objs = a[idx_objs, :4]
areas = a[np.logical_not(idx_objs)][:, 4:]

# build areas
dict_areas = {}
area_values = ""
for ap in areas:
    aname, *points = ap
    aname = aname.n3()
    if aname not in dict_areas:
        area_values += f"({aname})\n"
        dict_areas[aname] = []
    dict_areas[aname].append([p.toPython() for p in points])

inserts = ""
deletes = """?obj crow:insideOf ?a .
      ?a crow:contains ?obj ."""
wheres = f"""?obj crow:insideOf ?a .
      ?a crow:contains ?obj .
  VALUES (?a)
  {{
    {area_values}
  }}"""
for obj in objs:
    o, *opoints = obj
    o = o.n3()
    s = f"{o} crow:insideOf ?a .\n"
    wheres += s
    opoints = [p.toPython() for p in opoints]
    for aname, apoints in dict_areas.items():
        if test_in_hull(opoints, apoints):
            inserts += f"{o} crow:insideOf {aname} .\n"
            inserts += f"{aname} crow:contains {o} .\n"
query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
    prefix owl: <http://www.w3.org/2002/07/owl#>
    prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
    prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
    DELETE {{
        {deletes}
    }}\n"""
query += f"""WHERE {{
    {wheres}
}}"""
onto.update(query)
if len(inserts) > 0:
    query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        INSERT DATA {{
            {inserts}
        }}\n"""
    onto.update(query)


# %% PAIR TOGETHER
r = list(onto.query("""
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

SELECT DISTINCT ?obj ?obj_x ?obj_y ?obj_z ?area ?area_x ?area_y ?area_z
FROM <http://imitrob.ciirc.cvut.cz/ontologies/crow>
WHERE {
  {
    		?obj crow:hasId ?obj_id .
            ?obj crow:hasAbsoluteLocation ?obj_loc .
            ?obj_loc crow:x ?obj_x .
            ?obj_loc crow:y ?obj_y .
            ?obj_loc crow:z ?obj_z .
  } UNION {
            ?area a crow:StorageSpace .
            ?area crow:hasPolyhedron ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?area_x .
            ?pt crow:y ?area_y .
            ?pt crow:z ?area_z .
            FILTER EXISTS { ?area crow:areaType 'main' }

  }
}"""))
a = np.r_[r]
idx_objs = a[:, 0].astype(bool)
objs = a[idx_objs, :4]
areas = a[np.logical_not(idx_objs)][:, 4:]

dict_areas = {}
area_values = ""
for ap in areas:
    aname, *points = ap
    aname = aname.n3()
    if aname not in dict_areas:
        area_values += f"({aname})\n"
        dict_areas[aname] = []
    dict_areas[aname].append([p.toPython() for p in points])

allpoints = []
object_uris = []
for obj in objs:
    o, *opoints = obj
    o = o.n3()
    object_uris.append(o)
    allpoints.append([p.toPython() for p in opoints])
allpoints = np.array(allpoints)
object_uris = np.array(object_uris)

inserts = ""
for aname, points in dict_areas.items():
    hull = Delaunay(points)
    w = hull.find_simplex(allpoints)
    for o in object_uris[w >= 0]:
        inserts += f"{o} crow:insideOf {aname} .\n"
        inserts += f"{aname} crow:contains {o} .\n"

query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

DELETE {{
   ?obj crow:insideOf ?a .
  	?a crow:contains ?obj .
}}
WHERE {{
    ?obj crow:insideOf ?a .
	 ?a crow:contains ?obj .
  VALUES (?a)
  {{
    {area_values}
  }}
}};
INSERT DATA {{
    {inserts}
}}
"""
onto.update(query)

# %%  SUPER LARGE UPDATE
crw.reset_database()

batch_update = [tester.add_and_gen_update() for i in range(10)]
batch_add = [tester.get_no_template() for i in range(10)]

inserts = ""
deletes = ""
wheres = ""
union_started = False

for i, obj in enumerate(batch_update):
    object, location, size, timestamp, tracked = obj
    object = URIRef(object).n3()
    new_stamp = Literal(timestamp, datatype=XSD.dateTimeStamp).n3()
    new_x = Literal(location[0], datatype=XSD.float).n3()
    new_y = Literal(location[1], datatype=XSD.float).n3()
    new_z = Literal(location[2], datatype=XSD.float).n3()
    new_pcl_x = Literal(size[0], datatype=XSD.float).n3()
    new_pcl_y = Literal(size[1], datatype=XSD.float).n3()
    new_pcl_z = Literal(size[2], datatype=XSD.float).n3()
    tracked = Literal(tracked, datatype=XSD.boolean).n3()

    deletes += f"""
        {object} crow:hasTimestamp ?old_stamp{str(i)} .
        ?loc{str(i)} crow:x ?old_x{str(i)} .
        ?loc{str(i)} crow:y ?old_y{str(i)} .
        ?loc{str(i)} crow:z ?old_z{str(i)} .
        ?pcl{str(i)} crow:x ?old_pcl_x{str(i)} .
        ?pcl{str(i)} crow:y ?old_pcl_y{str(i)} .
        ?pcl{str(i)} crow:z ?old_pcl_z{str(i)} .
        {object} crow:isTracked ?old_tracked{str(i)} .
    """
    inserts += f"""
        {object} crow:hasTimestamp {new_stamp} .
        ?loc{str(i)} crow:x {new_x} .
        ?loc{str(i)} crow:y {new_y} .
        ?loc{str(i)} crow:z {new_z} .
        ?pcl{str(i)} crow:x {new_pcl_x} .
        ?pcl{str(i)} crow:y {new_pcl_y} .
        ?pcl{str(i)} crow:z {new_pcl_z} .
        {object} crow:isTracked {tracked} .
    """
    if union_started:
        wheres += "UNION {"
    else:
        wheres += "{"
        union_started = True
    wheres += f"""
            {object} crow:hasTimestamp ?old_stamp{str(i)} .
            {object} crow:hasAbsoluteLocation ?loc{str(i)} .
            {object} crow:hasPclDimensions ?pcl{str(i)} .
            ?loc{str(i)} crow:x ?old_x{str(i)} .
            ?loc{str(i)} crow:y ?old_y{str(i)} .
            ?loc{str(i)} crow:z ?old_z{str(i)} .
            ?pcl{str(i)} crow:x ?old_pcl_x{str(i)} .
            ?pcl{str(i)} crow:y ?old_pcl_y{str(i)} .
            ?pcl{str(i)} crow:z ?old_pcl_z{str(i)} .
            {object} crow:isTracked ?old_tracked{str(i)} .
        }}
    """

already_template = []
for i, obj in enumerate(batch_add):
    object_name, location, size, uuid, timestamp, adder_id, tracked = obj
    individual_name = object_name + '_od_'+str(adder_id)
    individual = CROW[individual_name].n3()
    PART = Namespace(f"{ONTO_IRI}/{individual_name}#")
    loc = PART.xyzAbsoluteLocation.n3()
    pcl = PART.hasPclDimensions.n3()
    inserts += f"""
        {individual} ?prop_{object_name} ?value_{object_name} .
        {individual} crow:hasAbsoluteLocation {loc} .
        {individual} crow:hasPclDimensions {pcl} .
        {loc} a crow:xyzAbsoluteLocation .
        {loc} crow:x {Literal(location[0], datatype=XSD.float).n3()} .
        {loc} crow:y {Literal(location[1], datatype=XSD.float).n3()} .
        {loc} crow:z {Literal(location[2], datatype=XSD.float).n3()} .
        {pcl} a crow:xyzPclDimensions .
        {pcl} crow:x {Literal(size[0], datatype=XSD.float).n3()} .
        {pcl} crow:y {Literal(size[1], datatype=XSD.float).n3()} .
        {pcl} crow:z {Literal(size[2], datatype=XSD.float).n3()} .
        {individual} crow:hasId {Literal('od_'+str(adder_id), datatype=XSD.string).n3()} .
        {individual} crow:hasUuid {Literal(uuid, datatype=XSD.string).n3()} .
        {individual} crow:hasTimestamp {Literal(timestamp, datatype=XSD.dateTimeStamp).n3()} .
        {individual} crow:isTracked {Literal(tracked, datatype=XSD.boolean).n3()} .
    """
    if object_name not in already_template:
        already_template.append(object_name)
        if union_started:
            wheres += "UNION {"
        else:
            wheres += "{"
            union_started = True        
        wheres += f"""
                ?template_{object_name} ?prop_{object_name} ?value_{object_name} ;
                        crow:hasDetectorName {Literal(object_name, datatype=XSD.string).n3()} .
                FILTER NOT EXISTS {{ ?template_{object_name} crow:hasUuid ?uuid_any_{object_name} }}
                MINUS {{ ?template_{object_name} crow:hasAbsoluteLocation|crow:hasPclDimensions ?value_{object_name} }}
            }}
        """
    
query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
    prefix owl: <http://www.w3.org/2002/07/owl#>
    prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
    prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
    DELETE {{
        {deletes}
    }}\n"""
query += f"""INSERT {{
        {inserts}
    }}\n"""
query += f"""WHERE {{
    {wheres}
}}"""
print(query)
# %%
onto.update(query)

# %% MOLOCH TEST
crw.reset_database()

batch_update = [tester.add_and_gen_update() for i in range(10)]
batch_add = [tester.get_no_template() for i in range(10)]

# %% EN DIS DEL PAIR
nobjs = 10

def gen_objs():
    crw.reset_database()
    enabled_objs = [tester.add_object() for i in range(nobjs)]
    disabled_objs = [tester.add_object() for i in range(nobjs)]
    tbdel_objs = [tester.add_object() for i in range(nobjs)]
    for obj in disabled_objs:
        crw.disable_object(obj)
    return enabled_objs, disabled_objs, tbdel_objs

enabled_objs, disabled_objs, tbdel_objs = gen_objs()

# %%
start = time()
query = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
"""

if len(enabled_objs) > 0:
    tobe_disabled = ""
    for obj in enabled_objs:
    # for obj in enabled_objs[:int(nobjs/2)]:
        tobe_disabled += f"({obj.n3()})\n"
    query += f"""
    DELETE {{
      ?individual crow:hasId ?id .
    }}
    INSERT {{
      ?individual crow:disabledId ?id .
    }}
    WHERE {{
        ?individual crow:hasId ?id .
        VALUES (?individual)
        {{
          {tobe_disabled}
        }}
    }}"""
     
if len(disabled_objs) > 0:
    tobe_enabled = ""
    for obj in disabled_objs:
    # for obj in disabled_objs[:int(nobjs/2)]:
        tobe_enabled += f"({obj.n3()})\n"
    query += f""";
    DELETE {{
      ?individual crow:disabledId ?id .
    }}
    INSERT {{
      ?individual crow:hasId ?id .
    }}
    WHERE {{
        ?individual crow:disabledId ?id .
        VALUES (?individual)
        {{
          {tobe_enabled}
        }}
    }}"""

if len(tbdel_objs) > 0:
    tobe_deleted = ""
    for obj in tbdel_objs:
    # for obj in disabled_objs[:int(nobjs/2)]:
        tobe_deleted += f"({obj.n3()})\n"
    query += f""";
    DELETE {{
        ?individual ?p1 ?o1 .
        ?s2 ?p2 ?individual .
    }}
    WHERE {{
      {{?individual ?p1 ?o1}} UNION {{?s2 ?p2 ?individual}}
      VALUES (?individual) {{
         {tobe_deleted}
      }}
    }}"""
        
# print(query)
onto.update(query)
print(f"Full query runtine = {time() - start}")

enabled_objs, disabled_objs = disabled_objs, enabled_objs
     
     
# %%
start = time()
query = crw.generate_en_dis_del_pair_query(disabled_objs, enabled_objs, tbdel_objs)
enabled_objs, disabled_objs = disabled_objs, enabled_objs
onto.update(query)
print(f"Full query runtine = {time() - start}")