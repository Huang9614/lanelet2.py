#!/usr/bin/env python
import os
import tempfile

import lanelet2
from lanelet2.core import (AllWayStop, AttributeMap, BasicPoint2d,
                           BoundingBox2d, Lanelet, LaneletMap,
                           LaneletWithStopLine, LineString3d, Point2d, Point3d,
                           RightOfWay, TrafficLight, getId)
import lanelet2.core
import lanelet2.geometry
from lanelet2.projection import (UtmProjector, MercatorProjector,
                                 LocalCartesianProjector, GeocentricProjector)


example_file = os.path.join(os.path.dirname(os.path.abspath(
    __file__)), "../../lanelet2_maps/res/mapping_example.osm")
if not os.path.exists(example_file):
    # location after installing
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "../../share/lanelet2_maps/res/mapping_example.osm")


def tutorial():
    # We do our best to keep the python interface in sync with the c++ implementation. As a rule of thumb: Everything
    # you can do in c++ works pretty similar in python, we just "pythonized" some things. Therefore this tutorial only
    # shows you just the most important things and the things that are different from C++. For the rest have a look
    # at the c++ tutorial.
    part1primitives()
    part2regulatory_elements()
    part3lanelet_map()
    part4reading_and_writing()
    part5traffic_rules()
    part6routing()
    part7example_map()


def part1primitives():
    # Primitives work very similar to c++, except that the data can be accessed as properties instead of functions
    p = Point3d(getId(), 0, 0, 0)
    assert p.x == 0
    p.id = getId()
    p.attributes["key"] = "value"
    assert "key" in p.attributes
    assert p.attributes["key"] == "value"

    # the 2d/3d mechanics work too
    p2d = lanelet2.geometry.to2D(p)

    # all (common) geometry calculations are available as well:
    p2 = Point3d(getId(), 1, 0, 0)
    assert lanelet2.geometry.distance(p, p2) == 1
    assert lanelet2.geometry.distance(p2d, Point2d(getId(), 1, 0, 1)) == 1

    # linestrings work conceptually similar to a list (but they only accept points, of course)
    ls = LineString3d(getId(), [p, p2])
    assert ls[0] == p
    assert ls[-1] == p2
    assert p in ls
    for pt in ls:
        assert pt.y == 0

    ls_inv = ls.invert()
    assert ls_inv[0] == p2
    ls.append(Point3d(getId(), 2, 0, 0))
    del ls[2]



def part2regulatory_elements():
    # regulatory elements profit from pythons type system

    # TrafficLight
    lanelet = get_a_lanelet()
    light = get_linestring_at_y(3)
    traffic_light_regelem = TrafficLight(getId(), AttributeMap(), [light])
    lanelet.addRegulatoryElement(traffic_light_regelem)
    assert traffic_light_regelem in lanelet.regulatoryElements
    lights = [regelem for regelem in lanelet.regulatoryElements if isinstance(
        regelem, TrafficLight)]
    assert traffic_light_regelem in lights
    assert light in lights[0].trafficLights

    # RightOfWay
    stop_linestring = get_linestring_at_y(0)
    right_of_way_lanelets = [get_a_lanelet(), get_a_lanelet(1)]
    yielding_lanelets = [get_a_lanelet(2)]
    right_of_way_regelem = RightOfWay(getId(),
                                      AttributeMap(),
                                      right_of_way_lanelets,
                                      yielding_lanelets,
                                      stop_linestring)
    map = LaneletMap()
    map.add(yielding_lanelets[0])
    map.add(right_of_way_lanelets[0])
    map.add(right_of_way_lanelets[1])
    # must add to the map explicitly
    map.add(right_of_way_regelem)
    assert right_of_way_regelem in map.regulatoryElementLayer
    rightOfWays = [regelem for regelem in map.regulatoryElementLayer
                   if isinstance(regelem, RightOfWay)]
    assert right_of_way_regelem in rightOfWays
    # must have the circular reference from the yielding lanelet
    # otherwise, the last assertion will fail
    # this should have been automatically inferred by the regulatoryElements
    # getter function
    yielding_lanelets[0].addRegulatoryElement(right_of_way_regelem)
    # This regulatory element should affect the yielding lanelet
    assert right_of_way_regelem in yielding_lanelets[0].regulatoryElements

    # AllWayStop
    lanelets_with_stop_lines = [
        LaneletWithStopLine(get_a_lanelet(), get_linestring_at_y(0)),
        LaneletWithStopLine(get_a_lanelet(1), get_linestring_at_y(1)),
        LaneletWithStopLine(get_a_lanelet(2), get_linestring_at_y(2)),
        LaneletWithStopLine(get_a_lanelet(3), get_linestring_at_y(3))
    ]
    map = LaneletMap()
    # add the lanelets to the map, access stop line
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        map.add(lanelet_with_stop_line.lanelet)
        lanelet_with_stop_line.stopLine
    all_way_stop_regelem = AllWayStop(getId(),
                                      AttributeMap(),
                                      lanelets_with_stop_lines)
    # must add to the map explicitly
    map.add(all_way_stop_regelem)
    assert all_way_stop_regelem in map.regulatoryElementLayer
    allWayStops = [regelem for regelem in map.regulatoryElementLayer
                   if isinstance(regelem, AllWayStop)]
    assert all_way_stop_regelem in allWayStops
    # must have the circular reference from each yielding lanelet
    # otherwise, the last assertion will fail
    # this should have been automatically inferred by the regulatoryElements
    # getter function
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        lanelet_with_stop_line.lanelet.addRegulatoryElement(
            all_way_stop_regelem)
    # This regulatory element should affect each yielding lanelet
    for lanelet_with_stop_line in lanelets_with_stop_lines:
        assert all_way_stop_regelem in lanelet_with_stop_line.lanelet.regulatoryElements


def part3lanelet_map():
    # lanelets map work just as you would expect:
    map = LaneletMap()
    lanelet = get_a_lanelet()
    map.add(lanelet)
    assert lanelet in map.laneletLayer
    assert map.pointLayer
    assert not map.areaLayer
    assert len(map.pointLayer.nearest(BasicPoint2d(0, 0), 1)) == 1
    searchBox = BoundingBox2d(BasicPoint2d(0, 0), BasicPoint2d(2, 2))
    assert len(map.pointLayer.search(searchBox)) > 1

    # you can also create a map from a list of primitives (replace Lanelets by the other types)
    mapBulk = lanelet2.core.createMapFromLanelets([get_a_lanelet()])
    assert len(mapBulk.laneletLayer) == 1


def part4reading_and_writing():
    # there are two ways of loading/writing a lanelet map: a robust one and an normal one. The robust one returns found
    # issues as extra return parameter
    map = LaneletMap()
    lanelet = get_a_lanelet()
    map.add(lanelet)
    path = os.path.join(tempfile.mkdtemp(), 'mapfile.osm')
    # Select a suitable projector depending on the data source
    ## UtmProjector: (0,0,0) is at the provided lat/lon on the WGS84 ellipsoid
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    ## MarcatorProjector: (0,0,0) is at the provided lat/lon on the mercator cylinder
    projector = MercatorProjector(lanelet2.io.Origin(49, 8.4))
    ## LocalCartesianProjector: (0,0,0) is at the provided origin (including elevation)
    projector = LocalCartesianProjector(lanelet2.io.Origin(49, 8.4, 123))

    # Writing the map to a file
    ## 1. Write with the given projector and use default parameters
    lanelet2.io.write(path, map, projector)

    ## 2. Write and get the possible errors
    write_errors = lanelet2.io.writeRobust(path, map, projector)
    assert not write_errors

    ## 3. Write using the default spherical mercator projector at the giver origin
    ## This was the default projection in Lanelet1. Use is not recommended.
    lanelet2.io.write(path, map, lanelet2.io.Origin(49, 8.4))

    ## 4. Write using the given projector and override the default values of the optional parameters for JOSM
    params = {
               "josm_upload": "true",          # value for the attribute "upload", default is "false"
               "josm_format_elevation": "true"  # whether to limit up to 2 decimals, default is the same as for lat/lon
             };
    lanelet2.io.write(path, map, projector, params)

    # Loading the map from a file
    loadedMap, load_errors = lanelet2.io.loadRobust(path, projector)
    assert not load_errors
    assert loadedMap.laneletLayer.exists(lanelet.id)

    '''
    ## GeocentricProjector: the origin is the centre of the Earth
    gc_projector = GeocentricProjector()
    write_errors = lanelet2.io.writeRobust(path, map, gc_projector)
    assert not write_errors
    loadedMap, load_errors = lanelet2.io.loadRobust(path, gc_projector)
    assert not load_errors
    assert loadedMap.laneletLayer.exists(lanelet.id)
    '''

def part5traffic_rules():
    # this is just as you would expect
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    lanelet = get_a_lanelet()
    lanelet.attributes["vehicle"] = "yes"
    assert traffic_rules.canPass(lanelet)
    assert traffic_rules.speedLimit(lanelet).speedLimit > 1


def part6routing():
    # and this as well
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    map = lanelet2.io.load(example_file, projector)
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(map, traffic_rules)
    lanelet = map.laneletLayer[4984315]
    toLanelet = map.laneletLayer[2925017]
    assert graph.following(lanelet)
    assert len(graph.reachableSet(lanelet, 100, 0)) > 10
    assert len(graph.possiblePaths(lanelet, 100, 0, False)) == 1

    # here we query a route through the lanelets and get all the vehicle lanelets that conflict with the shortest path
    # in that route
    route = graph.getRoute(lanelet, toLanelet)
    path = route.shortestPath()
    confLlts = [llt for llt in route.allConflictingInMap() if llt not in path]
    assert len(confLlts) > 0

    # for more complex queries, you can use the forEachSuccessor function and pass it a function object
    assert hasPathFromTo(graph, lanelet, toLanelet)


def hasPathFromTo(graph, start, target):
    class TargetFound(BaseException):
        pass

    def raiseIfDestination(visitInformation):
        # this function is called for every successor of lanelet with a LaneletVisitInformation object.
        # if the function returns true, the search continues with the successors of this lanelet.
        # Otherwise, the followers will not be visited through this lanelet, but could still be visited through
        # other lanelets.
        if visitInformation.lanelet == target:
            raise TargetFound()
        else:
            return True
    try:
        graph.forEachSuccessor(start, raiseIfDestination)
        return False
    except TargetFound:
        return True


def get_linestring_at_x(x):
    return LineString3d(getId(), [Point3d(getId(), x, i, 0) for i in range(0, 3)])


def get_linestring_at_y(y):
    return LineString3d(getId(), [Point3d(getId(), i, y, 0) for i in range(0, 3)])


def get_a_lanelet(index=0):
    return Lanelet(getId(),
                   get_linestring_at_y(3+index),
                   get_linestring_at_y(0+index))


def part7example_map():
############################################################################
# This part is used to check all the python_api for Layer, Lanelet, RoutingGraph and route, just for a more compact orgnization of these codes
#############################################################################

    # ----------- read map from file ----------
    print(" *** load map from file ***")
    # set node 40736 as origin
    projector = UtmProjector(lanelet2.io.Origin(49.00535119589, 8.41556206437))
    example_map, load_errors = lanelet2.io.loadRobust(example_file, projector)


    # ----------- methods of Layer ----------

    laneletlayer = example_map.laneletLayer
    regulatory_layer = example_map.regulatoryElementLayer

    ## .len
    print(f"Totally, there are {len(laneletlayer)} lanelets in the example_map.")
    ## .exists
    print(laneletlayer.exists(45112))
    ## .iter
    lanelet_layer_list = []
    for each_lanelet in laneletlayer:
        lanelet_layer_list.append(each_lanelet)
    print(f"{type(lanelet_layer_list[0])} \n")
    ## .get and its alternative
    lanelet_45064 = laneletlayer[45064]
    lanelet_45064_get = laneletlayer.get(45064)
    assert lanelet_45064 == lanelet_45064_get
    ## .search
    nearby_lanelet = laneletlayer.search(BoundingBox2d(BasicPoint2d(0,0), BasicPoint2d(2,2)))
    for lanelet in nearby_lanelet:
        print(f"Inside the boundingbox, there are lanelet: {lanelet.id}\n")
    ## .nearest and its alternative
    neighbor = lanelet2.geometry.findNearest(laneletlayer, BasicPoint2d(0,0),1)
    print(f"lanelet found by findNearest: {neighbor}\n")
    neighbor2 = laneletlayer.nearest(BasicPoint2d(0, 0), 1)
    print(f"lanelet found by nearest: {neighbor2}\n")
    ## .findUsages
    way_44058 = example_map.lineStringLayer[44058]
    lanelet_45334 = laneletlayer.findUsages(way_44058)
    print(f"lanelet_45334 found by findUsages: {lanelet_45334}\n")


    # -------- method of lanelet --------

    # .centerline
    centerline = lanelet_45064.centerline
    print(f"The centerline of lanelet_45064 is:  {centerline}\n")
    # .leftBound
    print(f"left bound: {lanelet_45064.leftBound}\n")
    # .rightBound
    print(f"right bound: {lanelet_45064.rightBound}\n")
    ## .regulatoryElements
    for regelem in example_map.laneletLayer[45082].regulatoryElements:
        print(f"regelem: {regelem}\n")
    ## .trafficlights()
    lanelet_45082 = laneletlayer[45082]
    print("lanelet_45082 related traffic_light is: ", lanelet_45082.trafficLights(),"\n")
    ## .rightOfWay()
    print("lanelet_45082 related rightOfWay is: ", lanelet_45082.rightOfWay()[0], "\n")
    ## .invert()
    print(f"inverted lanelet_45082: {lanelet_45082.invert()} \n")


    # --------- method of routingGraph -----------
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(example_map, traffic_rules)


    #lanelet_vehGraph_s_4984315 = laneletlayer[4984315]
    #lanelet_vehGraph_e_7697222576222483732 = laneletlayer[7697222576222483732]
    start_lanelet_45460_graph = laneletlayer[45300]
    dest_lanelet_45290_graph = laneletlayer[45362]

    ## .shortestPath
    for i in graph.shortestPath(start_lanelet_45460_graph,dest_lanelet_45290_graph):
        print(i.id)
    print("shortest path in graph: ", graph.shortestPath(start_lanelet_45460_graph,dest_lanelet_45290_graph))
    ## .following
    lanelet_with_two_follow = laneletlayer[45124]
    for i in graph.following(lanelet_with_two_follow):
        print("following lanelet: ",i.id,"\n")
    ## .followingRelations
    for j in graph.followingRelations(lanelet_with_two_follow):
        print(f"following relations are: ", j.relationType, "\n")
    ## .previous
    lanelet_with_two_previous = laneletlayer[45144]
    for i in graph.previous(lanelet_with_two_previous):
        print("previous lanelet: ", i.id,"\n")
    ## .previousRelations
    for j in graph.previousRelations(lanelet_with_two_previous):
        print(f"previous relation is: ", j.relationType, "\n")
    ## .adjacentLeft
    adja_lanelet = laneletlayer[3766022379599666264]
    print(f"left but not_lane-changable lanelet: {graph.adjacentLeft(adja_lanelet).id} \n")
    ## .reachableSet
    for i in graph.reachableSet(adja_lanelet, 20):
        print(f"reachable lanelet: {i.id}\n")
    ## .routingRelation
    print(f"routing relation is: {graph.routingRelation(laneletlayer[45362],laneletlayer[45364],0)}\n")
    ## .possiblePaths
    print(f"possible path from start allowing lane change: {graph.possiblePaths(laneletlayer[45358],5,0,True)}\n")
    print(f"there are {len(graph.possiblePaths(laneletlayer[45358],30,0,True))} paths")
    for i in graph.possiblePaths(laneletlayer[45358],30,0,True)[0]:
        print("######",i.id,"\n")



    ''' check the two_way lanelet
    # maybe two_way for specific participant. Not the case, otherwise should be one_way:bicycle = no
    print(f"following lanelet of a two-way lanelet for vehicle: {graph.following(laneletlayer[45362])}\n")
    traffic_rules_bicycle = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Bicycle)
    graph_bicycle= lanelet2.routing.RoutingGraph(example_map, traffic_rules_bicycle)
    print(f"following lanelet of a two-way lanelet for bicycle: {graph.following(laneletlayer[45362])}\n")

    # lanelet 45334 is an error or need to modify the implementation of following for two_way lanelets
    print(f"45356 has {len(graph.following(laneletlayer[45356]))} following, and the follwoing lanelet: {graph.following(laneletlayer[45356])[0].id}\n")
    print(f"45356 has {len(graph.following(laneletlayer[45356]))} previous, previous lanelet: {graph.previous(laneletlayer[45356])[0].id}\n")

    print(f"45334 has {len(graph.following(laneletlayer[45334]))} following, and the follwoing lanelet: {graph.following(laneletlayer[45334])[0].id}\n")
    print(f"45334 has {len(graph.following(laneletlayer[45334]))} previous, previous lanelet: {graph.previous(laneletlayer[45334])[0].id}\n")
    '''


    ## ------ method of route -------
    s_lanelet = laneletlayer[4984315]
    d_lanelet = laneletlayer[2925017]
    ## .getRoute
    route = graph.getRoute(s_lanelet,d_lanelet)
    print("route: ",route)
    ## .shortestPath
    s_path = route.shortestPath()
    for i in s_path:
        print("lanelet belonging to the shortest path: ",i.id, "\n")
    ## .fullLane
    for i in route.fullLane(laneletlayer[1967009324258694641]):
        print(f"full lane of s_lanelet: {i.id} \n")
    ## .remainingLane
    for i in route.remainingLane(laneletlayer[s_lanelet.id]):
        print(f"remaining lanelets in the lane: {i.id}\n")
    ## .remainingShortestPath
    for i in route.remainingShortestPath(laneletlayer[236893084089463991]):
        print(f"remain lanelets in the shortest path: {i.id}\n")
    ## .length2d
    print(route.length2d())
    ## .numLanes
    print(route.numLanes())
    ## .size
    print("size: ",route.size())
    ## .fowllowingRelations
    print("following relation: ", route.followingRelations(s_lanelet)[0].lanelet)
    ## .previousRelations
    print("previous relation: ", route.previousRelations(laneletlayer[1490339216733857237])[0].lanelet)
    ## .leftRealtion

    ## .leftRelations

    ## .rightRelation

    ## .rightRelations

    ## .conflictingInRoute

    ## .conflictingInMap
    print("conflicting in map: ",route.conflictingInMap(laneletlayer[6012398680329441872])[0].id)
    ## .allConflictingInMap
    for i in route.allConflictingInMap():
        print("all conflicting lanelets with the lanelets in route: ",i.id)
    '''
    for i in route.remainingLane(start_lanelet_45460_graph):
        print(f"remaining lanelets: {i.id}\n")
    print(route.length2d())
    print(route.numLanes())
    print(route.size())
    '''



if __name__ == '__main__':
    tutorial()
