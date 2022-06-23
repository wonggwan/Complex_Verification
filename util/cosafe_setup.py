import re


def cosafe_obstacles(formula, room_goal_dict):
    # base on room name
    info = [formula[int(i.start()) + 1] for i in re.finditer('e', formula)]
    info = list(set(info))
    avoid_dict = {}
    avoid_room_name = {}
    for i, x in enumerate(info):
        obstacle_info = info[:i] + info[i + 1:]
        for element in obstacle_info:
            if x not in avoid_dict:
                avoid_dict[x] = []
                avoid_room_name[x] = []
            avoid_dict[x].append(room_goal_dict[element])
            avoid_room_name[x].append(element)
    print(avoid_room_name)
    return avoid_dict, avoid_room_name


def update_obstacles_rt(avoid_dict, avoid_room_name, room_goal_dict, cur_suc, room_num):
    if cur_suc != -1:
        avoid_dict[room_num].remove(room_goal_dict[cur_suc])
        avoid_room_name[room_num].remove(cur_suc)
    return avoid_dict, avoid_room_name
