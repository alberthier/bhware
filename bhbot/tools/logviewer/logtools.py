# encoding: utf-8


def get_value(dump, name):
    for key, value in dump:
        if key == name:
            return value
    return None
