import cvxpy as cp
import numpy as np

# Problem data.

# Construct the problem.
x = cp.Variable(integer=True)
objective = cp.Minimize(100/x + x)
constraints = [1 <= x, x <= 100]
prob = cp.Problem(objective, constraints)

# The optimal objective value is returned by `prob.solve()`.
result = prob.solve(gp=True)
# The optimal value for x is stored in `x.value`.
print(x.value)
