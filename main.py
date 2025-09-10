import random
from functools import reduce

from ortools.sat.python import cp_model

ndoors = 6
nlabels = 4


def generate_source_model(num_hexagons):
    hexs = [[(i+1) % num_hexagons for j in range(ndoors)] for i in range(num_hexagons)]
    return hexs


def compute_walk(walk, hexs, labels):
    curr = 0
    result = [0]
    for i in range(len(walk)):
        next = hexs[curr][walk[i]]
        result.append(labels[next])
        curr = next
    return result


def sat(combined, nhexagons):
    model = cp_model.CpModel()

    # true if there is a connection from i to j through door k
    conn_vars = [[[model.new_bool_var(f"conn_{i}_{j}_{k}") for k in range(nhexagons)] for j in range(ndoors)] for i in range(nhexagons)]

    # true if hexagon has the label
    label_vars = [[model.new_bool_var(f"label_{hexagon}_{label}") for label in range(nlabels)] for hexagon in range(nhexagons)]

    # all hexagons can only have 1 connection per door
    for fromH in range(nhexagons):
        for door in range(ndoors):
            model.Add(reduce(lambda x, y: x + y, [conn_vars[fromH][door][toH] for toH in range(nhexagons)]) == 1)

    # all hexagons have exactly 6 connections going to them
    for toH in range(nhexagons):
        model.Add(reduce(lambda x, y: x + y, [conn_vars[fromH][door][toH] for door in range(ndoors) for fromH in range(nhexagons)]) == ndoors)

    # first label must be 0
    model.Add(label_vars[0][0] == True)

    # true if there is a connection from i to j through any door
    is_connected_vars = [[model.new_bool_var(f"is_connected_{fromH}_{toH}") for toH in range(nhexagons)] for fromH in range(nhexagons)]

    # all hexagons must be connected
    arcs = []
    for fromH in range(nhexagons):
        for toH in range(nhexagons):
            is_connected_var = is_connected_vars[fromH][toH]
            model.Add(reduce(lambda x, y: x + y, [conn_vars[fromH][door][toH] for door in range(ndoors)]) > 0).only_enforce_if(is_connected_var)
            arcs.append((fromH, toH, is_connected_var))
    model.AddCircuit(arcs)

    # all labels must exist
    for label in range(min(nlabels, nhexagons)):
        model.Add(reduce(lambda x, y: x + y, [label_vars[h][label] for h in range(nhexagons)]) > 0)

    # all hexagons must have exactly 1 label
    for hexagon in range(nhexagons):
        model.Add(reduce(lambda x, y: x + y, [label_vars[hexagon][label] for label in range(nlabels)]) == 1)

    # the path of the walk must be valid according to the connections and labels
    for i in range(len(combined)):
        [fromLabel, door, toLabel] = combined[i]
        path_covered_vars = []
        for (fromH, toH) in [(x, y) for x in range(nhexagons) for y in range(nhexagons)]:
            is_path_covered_var = model.new_bool_var(f"is_path_covered_{i}_{fromH}_{door}_{toH}")
            path_covered_vars.append(is_path_covered_var)
            model.AddBoolAnd([conn_vars[fromH][door][toH], label_vars[fromH][fromLabel], label_vars[toH][toLabel]]).only_enforce_if([is_path_covered_var])
        model.Add(reduce(lambda x, y: x + y, path_covered_vars) > 0)

    solver = cp_model.CpSolver()
    status = solver.solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print("labels:")
        labels_result = [-1 for _ in range(nhexagons)]
        conns = [[-1 for _ in range(ndoors)] for _ in range(nhexagons)]
        for hexagon in range(nhexagons):
            for label in range(nlabels):
                if solver.value(label_vars[hexagon][label]):
                    print(f"{hexagon}label {label}")
                    labels_result[hexagon] = label

        for fromH in range(nhexagons):
            for door in range(ndoors):
                for toH in range(nhexagons):
                    if solver.value(conn_vars[fromH][door][toH]):
                        print(f"{fromH}[{door}]=>{toH}")
                        conns[fromH][door] = toH
        return (conns, labels_result)
    else:
        print("No solution found.")
        return (None, None)


def de_bruijn(k: int, n: int) -> str:
    """
    Generate a De Bruijn sequence for alphabet size k and subsequences of length n.

    Args:
        k (int): Size of the alphabet (e.g., 2 for binary, 10 for digits).
        n (int): Length of subsequences.

    Returns:
        str: A De Bruijn sequence as a string of digits in base k.
    """
    alphabet = range(k)
    a = [0] * (k * n)
    sequence = []

    def db(t: int, p: int):
        if t > n:
            if n % p == 0:
                sequence.extend(a[1:p+1])
        else:
            a[t] = a[t - p]
            db(t + 1, p)
            for j in range(a[t - p] + 1, k):
                a[t] = j
                db(t + 1, t)

    db(1, 1)
    return [alphabet[i] for i in sequence]


def main():
    nhexagons = 4
    print(f"nhexagons: {nhexagons}")
    hexs = generate_source_model(nhexagons)
    # do a random walk
    walk = de_bruijn(ndoors, 5)
    print(f"walk length: {len(walk)}")
    result = compute_walk(walk, hexs, [i % nlabels for i in range(nhexagons)])
    combined = []
    for i in range(len(walk)-1):
        combined.append([result[i], walk[i], result[i+1]])

    (conns, labels_result) = sat(combined, nhexagons)
    if conns is None:
        return
    # print("combined:")
    # print(combined)
    test_result = compute_walk(walk, conns, labels_result)
    # print("result bit walk:")
    # print(test_result)
    # print("original bit walk:")
    # print(result)
    print("passed" if test_result == result else "failed")


main()
