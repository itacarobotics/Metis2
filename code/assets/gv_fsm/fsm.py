import graphviz

RED         = "#ff6666"
LIGHTRED    = "#ffc1c1"
GREEN       = "#66ff66"
LIGHTGREEN  = "#c1ffc1"


f = graphviz.Digraph("fsm_middleware", filename="assets/gv_fsm/_fsm_middleware.dot", format="png")

f.attr("node", shape="ellipse")

f.node("init", style="filled", color=GREEN, fillcolor=LIGHTGREEN)
f.node("consume")
f.node("compute")
f.node("produce")
f.node("fatal", style="filled", color=RED, fillcolor=LIGHTRED)

f.edge("init", "consume", label="")

f.edge("consume", "consume", label="")
f.edge("consume", "compute", label="")
f.edge("consume", "produce", label="")

f.edge("compute", "consume", label="")
f.edge("compute", "produce", label="")
f.edge("compute", "fatal", label="handle_fatal_error")

f.edge("produce", "compute", label="")
f.edge("produce", "consume", label="")
f.edge("produce", "produce", label="")


# Render and display the graph
f.view()