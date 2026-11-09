import gurobipy as gp
from gurobipy import GRB

n = 8
d = 4

# uniform, nodes send 4/7 to each other node
T = [[0 if i == j else d / (n - 1) for j in range(n)] for i in range(n)]

# lopsided, node i sends all to i+1
# T = [[d if j == (i + 1) % n else 0 for j in range(n)] for i in range(n)]

m = gp.Model()


#fraction of T we carry
theta = m.addVar()

# 1 if cable i -> j exists
E = {(i, j): m.addVar(vtype=GRB.BINARY)
        for i in range(n) for j in range(n) if i != j}

# flow from source s on cable i -> j
f = {(s, i, j): m.addVar()
    for s in range(n) for (i, j) in E}


# Rule 1, each node has d cables out and d cables in
for v in range(n):
    m.addConstr(sum(E[v, j] for j in range(n) if j != v) == d)
    m.addConstr(sum(E[i, v] for i in range(n) if i != v) == d)

# Rule 2, a cable carries max 1 unit (0 if it doesnt exist)
for (i, j) in E:
    m.addConstr(sum(f[s, i, j] for s in range(n)) <= E[i, j])

# Rule 3, flow conservation
for s in range(n):          # for each source s
    for v in range(n):      # check every node v
        # flow from source s leaving v (v's outgoing cables)
        out_ = sum(f[s, v, j] for j in range(n) if j != v)
        # flow from source s entering v (v's incoming cables)
        in_ = sum(f[s, i, v] for i in range(n) if i != v)
        if v == s:
            # v is the source, it sends theta * what s wants to send
            m.addConstr(out_ - in_ == theta * sum(T[s]))
        else:
            # v is a destination, keep theta * T[s][v] and forward the rest
            # if T[s][v] = 0, in = out, v is a middle node passing flow thru
            m.addConstr(in_ - out_ == theta * T[s][v])


m.setObjective(theta, GRB.MAXIMIZE) # gurobi plz maximize theta
m.optimize()

print("theta =", round(theta.X, 4))
for i in range(n):
    print(i, "->", [j for j in range(n) if j != i and E[i, j].X > 0.5])
