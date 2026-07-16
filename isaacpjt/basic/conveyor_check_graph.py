#!/usr/bin/env python3
# check_graph.py
# Diagnostic: auto-find all IsaacConveyor nodes in the Sorter ActionGraph,
# print their input connections, and test live values while toggling the switch.

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/basic/Collected_Conveyor_split/conveyor_split.usd"
GRAPH_PATH = "/World/ConveyorTrack/Sorter/ActionGraph"
SWITCH_NODE = GRAPH_PATH + "/binary_switch"

open_stage(usd_path=USD_PATH)
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

print("\n========== ALL NODES IN ACTIONGRAPH ==========")
graph_prim = stage.GetPrimAtPath(GRAPH_PATH)
conveyor_nodes = []
if graph_prim.IsValid():
    for child in graph_prim.GetChildren():
        node_type = ""
        ta = child.GetAttribute("node:type")
        if ta:
            node_type = ta.Get() or ""
        print(f"  {child.GetName():28s} type={node_type}")
        if "Conveyor" in node_type or "conveyor" in child.GetName():
            conveyor_nodes.append(str(child.GetPath()))
else:
    print("  ActionGraph NOT FOUND")

print(f"\n========== FOUND CONVEYOR NODES: {conveyor_nodes} ==========")

for node_path in conveyor_nodes:
    print(f"\n----- {node_path} : INPUTS -----")
    node_prim = stage.GetPrimAtPath(node_path)
    for attr in node_prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith("inputs:"):
            continue
        val = attr.Get()
        conns = attr.GetConnections() if hasattr(attr, "GetConnections") else []
        if conns:
            print(f"  {name:22s} = {val}   <== CONNECTED FROM {[str(c) for c in conns]}")
        else:
            print(f"  {name:22s} = {val}   (literal, no connection)")
    # show conveyorPrim target
    cp = node_prim.GetRelationship("inputs:conveyorPrim")
    if cp:
        print(f"  conveyorPrim target  -> {[str(t) for t in cp.GetTargets()]}")

print("\n========== LIVE TEST: toggle switch, read each conveyor node ==========")
for sw in [False, True]:
    og.Controller.attribute(SWITCH_NODE + ".inputs:value").set(sw)
    for _ in range(5):
        world.step(render=False)
    print(f"  switch = {sw}")
    for node_path in conveyor_nodes:
        try:
            vel = og.Controller.attribute(node_path + ".inputs:velocity").get()
        except Exception as e:
            vel = f"ERR({e})"
        try:
            direction = og.Controller.attribute(node_path + ".inputs:direction").get()
        except Exception as e:
            direction = f"ERR({e})"
        print(f"    {node_path.split('/')[-1]:18s} velocity={vel}  direction={direction}")

print("\n========== DONE ==========")
simulation_app.close()