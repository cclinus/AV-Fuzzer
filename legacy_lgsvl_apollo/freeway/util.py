#! /usr/bin/python3


def print_debug(info):
    print(info)
    with open('output.log', 'a') as f:
        print(info, file=f)
        f.close()

