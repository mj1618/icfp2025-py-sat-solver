import random
from functools import reduce

from ortools.sat.python import cp_model

from util import chunks_loop, de_bruijn

n_doors = 6
n_labels = 4


def any(iterable):
    for element in iterable:
        if element:
            return True
    return False


def incomplete_room_indexes(rooms):
    res = []
    for [room_idx, room] in enumerate(rooms):
        if any([room[door] == -1 for door in range(n_doors)]):
            res.append(room_idx)
    return res


def incomplete_room_door_indexes(room):
    res = []
    for door in (range(n_doors)):
        if room[door] == -1:
            res.append(door)
    return res


def generate_source_model(n_rooms):
    # rooms = [[(i+1) % n_rooms for j in range(n_doors)] for i in range(n_rooms)]

    labels = [i % n_labels for i in range(n_rooms)]
    rooms = [[-1 for j in range(n_doors)] for i in range(n_rooms)]
    for i in range(n_rooms):
        door_1 = random.choice(incomplete_room_door_indexes(rooms[i]))
        rooms[i][door_1] = (i+1) % n_rooms
        door_2 = random.choice(incomplete_room_door_indexes(rooms[(i+1) % n_rooms]))
        rooms[(i+1) % n_rooms][door_2] = i

    while len(incomplete_room_indexes(rooms)) > 0:
        if len(incomplete_room_indexes(rooms)) == 1:
            room_idx = random.choice(incomplete_room_indexes(rooms))
            door = random.choice(incomplete_room_door_indexes(rooms[room_idx]))
            rooms[room_idx][door] = room_idx
        else:
            room_idx = random.choice(incomplete_room_indexes(rooms))
            door = random.choice(incomplete_room_door_indexes(rooms[room_idx]))
            to_room_idx = random.choice(incomplete_room_indexes(rooms))
            to_door = random.choice(incomplete_room_door_indexes(rooms[to_room_idx]))
            rooms[room_idx][door] = to_room_idx
            rooms[to_room_idx][to_door] = room_idx

    if any([rooms[i][j] == -1 for i in range(n_rooms) for j in range(n_doors)]):
        print("problem with model generation")
        print(rooms)
        exit()

    # count tos
    test_tos = [0 for i in range(n_rooms)]
    for i in range(n_rooms):
        for j in range(n_doors):
            if rooms[i][j] != -1:
                test_tos[rooms[i][j]] += 1

    if any([test_tos[i] != n_doors for i in range(n_rooms)]):
        print("problem with model generation")
        print(test_tos)

        exit()

    print(rooms)
    return (rooms, labels)


def compute_walk(walk, conns, labels):
    curr = 0
    result = [0]
    for i in range(len(walk)):
        next = conns[curr][walk[i]]
        result.append(labels[next])
        curr = next
    return result


def sat(combined_ls, n_rooms):
    model = cp_model.CpModel()

    # true if there is a connection from i to j through door k
    conn_vars = [[[model.new_bool_var(f"conn_{i}_{j}_{k}") for k in range(n_rooms)] for j in range(n_doors)] for i in range(n_rooms)]

    # true if room has the label
    label_vars = [[model.new_bool_var(f"label_{room}_{label}") for label in range(n_labels)] for room in range(n_rooms)]

    # all rooms must have exactly 1 connection per door
    for from_room in range(n_rooms):
        for door in range(n_doors):
            model.Add(reduce(lambda x, y: x + y, [conn_vars[from_room][door][to_room] for to_room in range(n_rooms)]) == 1)

    # all rooms have exactly 6 connections going to them
    for to_room in range(n_rooms):
        model.Add(reduce(lambda x, y: x + y, [conn_vars[from_room][door][to_room] for door in range(n_doors) for from_room in range(n_rooms)]) == n_doors)

    # true if there is a connection from i to j through any door
    is_connected_vars = [[model.new_bool_var(f"is_connected_{from_room}_{to_room}") for to_room in range(n_rooms)] for from_room in range(n_rooms)]
    for i in range(n_rooms):
        for j in range(n_rooms):
            for door in range(n_doors):
                model.AddImplication(conn_vars[i][door][j], is_connected_vars[i][j])

    # all rooms must be connected
    for from_room in range(n_rooms):
        for to_room in range(n_rooms):
            for to_room_2 in range(n_rooms):
                is_connected_var = is_connected_vars[from_room][to_room]
                is_connected_var_2 = is_connected_vars[to_room][to_room_2]
                b = model.new_bool_var("")
                model.Add(b == True).only_enforce_if([is_connected_var, is_connected_var_2])
                model.AddImplication(b, is_connected_vars[from_room][to_room_2])
    for from_room in range(n_rooms):
        for to_room in range(n_rooms):
            model.Add(is_connected_vars[from_room][to_room] == True)

    # all labels must exist
    for label in range(min(n_labels, n_rooms)):
        model.Add(reduce(lambda x, y: x + y, [label_vars[r][label] for r in range(n_rooms)]) > 0)

    # first label must be 0
    model.Add(label_vars[0][0] == True)

    # force labels to be same order as rooms
    for i in range(n_rooms):
        model.Add(label_vars[i][i % n_labels] == True)

    # all rooms must have exactly 1 label
    for room in range(n_rooms):
        model.Add(reduce(lambda x, y: x + y, [label_vars[room][label] for label in range(n_labels)]) == 1)

    for combined in combined_ls:

        # the room position var is true if we are at the current room during the walk
        room_position_vars = [[model.new_bool_var(f"room_position_vars_{i}_{j}") for i in range(n_rooms)] for j in range(len(combined)+1)]

        # we always start at room 0
        model.Add(room_position_vars[0][0] == True)

        # the path of the walk must be valid according to the connections and labels
        for i in range(len(combined)):
            [from_label, door, to_label] = combined[i]

            # we can only be at exactly one room at each step
            model.Add(reduce(lambda x, y: x + y, [room_position_vars[i][j] for j in range(n_rooms)]) == 1)

            # bs are used as an intermediate variable for a (X && Y) => Z constraint
            bs = []
            for from_room in range(n_rooms):
                for to_room in range(n_rooms):
                    b = model.new_bool_var("")
                    bs.append(b)

                    # set the implication variable to true if we are at the from_room and there is a connection from the from_room to the to_room
                    model.Add(b == True).only_enforce_if([room_position_vars[i][from_room], conn_vars[from_room][door][to_room]])

                    # we're at "from_room" and the connection goes to "to_room", set the next room position var to true
                    model.AddImplication(b, room_position_vars[i+1][to_room])

                    # since "to_room" is next it must have the to_label
                    model.AddImplication(b, label_vars[to_room][to_label])

            # at least one implication must be true
            model.Add(reduce(lambda x, y: x + y, bs) >= 1)

    solver = cp_model.CpSolver()
    status = solver.solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print("labels:")
        labels_result = [-1 for _ in range(n_rooms)]
        conns = [[-1 for _ in range(n_doors)] for _ in range(n_rooms)]
        for room in range(n_rooms):
            for label in range(n_labels):
                if solver.value(label_vars[room][label]):
                    print(f"R{room} Label: {label}")
                    labels_result[room] = label

        for from_room in range(n_rooms):
            for door in range(n_doors):
                for to_room in range(n_rooms):
                    if solver.value(conn_vars[from_room][door][to_room]):
                        print(f"{from_room}[{door}]=>{to_room}")
                        conns[from_room][door] = to_room
        return (conns, labels_result)
    else:
        print("No solution found.")
        return (None, None)


def zip_walk_result(walk, result):
    return [[result[i], walk[i], result[i+1]] for i in range(len(walk)-1)]


def rotate_walk(walk, i, length):
    index = i % len(walk)
    prefix = walk[:index]
    rotated_walk = walk[index:] + prefix
    return rotated_walk[:length]


def random_rotation(walk, length):
    index = random.randint(0, len(walk)-1)
    return rotate_walk(walk, index, length)


def main():
    n_rooms = 18
    # dj_seq_length = 6
    max_walk_length = 18*n_rooms
    max_walks = 1
    # prefix_length = 256

    print(f"n_rooms: {n_rooms}")
    (rooms, labels) = generate_source_model(n_rooms)

    # do a random walk
    # dj = de_bruijn(n_doors, dj_seq_length)
    # walks = chunks_loop(dj, max_walk_length)[:max_walks]
    # walks = [rotate_walk(dj, round(i / max_walk_length), max_walk_length) for i in range(max_walks)]
    # random_walk = [random.randint(0, n_doors-1) for i in range(max_walk_length)]
    # walks = [list(map(int, list(random_walk))) for i in range(max_walks)]

    # this is a walk that was generated to hit as many vertices as possible
    entropy_walk = "413022551403315200442351124530532105441250342013450431221500235401332245541100524301142035531432234405215311350240015425104334025123304521120534103522445503114230032542135431452051002415012340523321554433021451132041025533014400351220440115324321342250451305502315412031045522433500142534115203442231104350310540221435132441500553020145133425523012412033443004231254411023052214500135410325345320121545042102113352405514315340154304422003355523411201445331322002514054225105033004323315550112134421441352532400511024124025405311304432221235"
    walks = [list(map(int, list(entropy_walk[:max_walk_length]))) for i in range(max_walks)]

    print(f"walk length: {len(walks)} x {max_walk_length} = {len(walks) * max_walk_length}")
    print("original labels:")
    print(labels)

    # prefixes = [rotate_walk(dj, round(i % max_walk_length), max_walk_length) for i in range(max_walks)]
    prefixes = [[] for i in range(max_walks)]
    zipped_walk = [zip_walk_result(prefixes[i]+walk, compute_walk(prefixes[i]+walk, rooms, labels)) for i, walk in enumerate(walks)]

    print("original connections:")
    for i in range(n_rooms):
        for j in range(n_doors):
            print(f"{i}[{j}]=>{rooms[i][j]}")

    (conns, labels_result) = sat([z[len(prefixes[i]):] for i, z in enumerate(zipped_walk)], n_rooms)
    if conns is None:
        return

    test_walks = [random_rotation(walk, max_walk_length) for walk in walks]
    expected_results = [zip_walk_result(walk, compute_walk(walk, rooms, labels)) for walk in test_walks]
    actual_results = [zip_walk_result(walk, compute_walk(walk, conns, labels_result)) for walk in test_walks]

    print("combined walk:")
    for [i, (expected, actual)] in enumerate(zip(expected_results, actual_results)):
        for [j, (expected_step, actual_step)] in enumerate(zip(expected, actual)):
            if expected_step != actual_step:
                print(f"Invalid result at position {i},{j}")
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
    for expected, actual in zip(expected_results, actual_results):
        if expected != actual:
            all_passed = False
            print(f"expected: {expected}")
            print(f"actual:   {actual}")
            break
    print("passed" if all_passed else "failed")


main()
