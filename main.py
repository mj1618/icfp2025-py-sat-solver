import random
from functools import reduce

from ortools.sat.python import cp_model

from util import chunks_loop, de_bruijn

ndoors = 6
nlabels = 4


def generate_source_model(num_hexagons):
    hexs = [[(i+1) % num_hexagons for j in range(ndoors)] for i in range(num_hexagons)]
    return hexs


def compute_walk(walk, conns, labels):
    curr = 0
    result = [0]
    for i in range(len(walk)):
        next = conns[curr][walk[i]]
        result.append(labels[next])
        curr = next
    return result


def sat(combined_ls, nhexagons):
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

    for combined in combined_ls:
        # the path of the walk must be valid according to the connections and labels
        for i in range(len(combined)):
            if i > 0:
                [prevFromLabel, prevDoor, _] = combined[i-1]
                [fromLabel, door, toLabel] = combined[i]
                path_covered_vars = []
                for (prevFromH, fromH, toH) in [(x, y, z) for z in range(nhexagons) for x in range(nhexagons) for y in range(nhexagons)]:
                    is_path_covered_var = model.new_bool_var(f"is_path_covered_{i}_{fromH}_{door}_{toH}")
                    path_covered_vars.append(is_path_covered_var)
                    model.AddBoolAnd([
                        conn_vars[prevFromH][prevDoor][fromH],
                        label_vars[prevFromH][prevFromLabel],
                        label_vars[fromH][fromLabel],
                        conn_vars[fromH][door][toH],
                        label_vars[fromH][fromLabel],
                        label_vars[toH][toLabel]]).only_enforce_if([is_path_covered_var])
                model.Add(reduce(lambda x, y: x + y, path_covered_vars) > 0)
            else:
                [fromLabel, door, toLabel] = combined[i]
                path_covered_vars = []
                for (fromH, toH) in [(x, y) for x in range(nhexagons) for y in range(nhexagons)]:
                    is_path_covered_var = model.new_bool_var(f"is_path_covered_{i}_{fromH}_{door}_{toH}")
                    path_covered_vars.append(is_path_covered_var)
                    model.AddBoolAnd([
                        conn_vars[fromH][door][toH],
                        label_vars[fromH][fromLabel],
                        label_vars[toH][toLabel]]).only_enforce_if([is_path_covered_var])
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
                    print(f"R{hexagon} Label: {label}")
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


def zip_walk_result(walk, result):
    return [[result[i], walk[i], result[i+1]] for i in range(len(walk)-1)]


def main():
    nhexagons = 6
    dj_seq_length = 6
    max_walk_length = 6
    max_walks = 100

    print(f"nhexagons: {nhexagons}")
    hexs = generate_source_model(nhexagons)

    # do a random walk
    walks = chunks_loop(de_bruijn(ndoors, dj_seq_length), max_walk_length)[:max_walks]
    print(f"walk length: {len(walks)} x {max_walk_length} = {len(walks) * max_walk_length}")
    print("original labels:")
    print([i % nlabels for i in range(nhexagons)])

    zipped_walk = [zip_walk_result(walk, compute_walk(walk, hexs, [i % nlabels for i in range(nhexagons)])) for walk in walks]
    print("original connections:")
    for i in range(nhexagons):
        for j in range(ndoors):
            print(f"{i}[{j}]=>{hexs[i][j]}")

    (conns, labels_result) = sat(zipped_walk, nhexagons)
    if conns is None:
        return

    test_results = [zip_walk_result(walk, compute_walk(walk, conns, labels_result)) for walk in walks]

    print("combined walk:")
    for (expected, actual) in zip(zipped_walk, test_results):
        for (expected_step, actual_step) in zip(expected, actual):
            if expected_step != actual_step:
                print(f"Invalid result at position {i}")
                print(expected)
                print(actual)
                print(f"expected: {expected_step[0]}[{expected_step[1]}]=>{expected_step[2]}")
                print(f"actual:   {actual_step[0]}[{actual_step[1]}]=>{actual_step[2]}")
                print("failed")
                exit()
                break
            else:
                print(f"{expected_step[0]}[{expected_step[1]}]=>{expected_step[2]}")

    all_passed = True
    for test_result, result in zip(test_results, zipped_walk):
        if test_result != result:
            all_passed = False
            print(f"expected: {result}")
            print(f"actual:   {test_result}")
            break
    print("passed" if all_passed else "failed")


main()
