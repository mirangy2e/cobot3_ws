#!/usr/bin/env python3
# check_graph2.py
# Check whether the ActionGraph is registered/evaluated in standalone,
# and find where binary_switch flows to (which downstream nodes it drives).

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/basic/Collected_Conveyor_split/conveyor_split.usd"
GRAPH_PATH = "/World/ConveyorTrack/Sorter/ActionGraph"

open_stage(usd_path=USD_PATH)
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

print("\n========== GRAPHS REGISTERED IN OMNIGRAPH ==========")
try:
    graphs = og.get_all_graphs()
    for g in graphs:
        print(f"  graph: {g.get_path_to_graph()}")
except Exception as e:
    print(f"  ERR listing graphs: {e}")

print("\n========== TRY GET ACTIONGRAPH AS OG GRAPH ==========")
try:
    g = og.get_graph_by_path(GRAPH_PATH)
    print(f"  get_graph_by_path -> {g}")
    if g is not None:
        print(f"  is_valid: {g.is_valid()}")
        print(f"  pipeline stage: {g.get_pipeline_stage()}")
        print(f"  evaluation mode: {g.get_evaluation_mode()}")
        print("  nodes in graph:")
        for n in g.get_nodes():
            print(f"     {n.get_prim_path()}")
except Exception as e:
    print(f"  ERR: {e}")

print("\n========== write_prim_attribute: what does it write? ==========")
wpa = stage.GetPrimAtPath(GRAPH_PATH + "/write_prim_attribute")
if wpa.IsValid():
    for attr in wpa.GetAttributes():
        nm = attr.GetName()
        if nm.startswith("inputs:") and ("attribute" in nm.lower() or "value" in nm.lower()):
            print(f"  {nm} = {attr.Get()}")
    rel = wpa.GetRelationship("inputs:prims")
    if rel:
        print(f"  writes to prims -> {[str(t) for t in rel.GetTargets()]}")

print("\n========== DONE ==========")
simulation_app.close()