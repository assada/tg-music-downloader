# coding=utf-8
from __future__ import unicode_literals

import sys

from slugify import slugify

reload(sys)
sys.setdefaultencoding('utf8')


class Helper:
    def __init__(self):
        pass

    @staticmethod
    def build_menu(buttons,
                   n_cols,
                   header_buttons=None,
                   footer_buttons=None):
        menu = [buttons[i:i + n_cols] for i in range(0, len(buttons), n_cols)]
        if header_buttons:
            menu.insert(0, header_buttons)
        if footer_buttons:
            menu.append(footer_buttons)
        return menu

    @staticmethod
    def list_get(l, idx, default):
        try:
            return l[idx]
        except IndexError:
            return default

    @staticmethod
    def formatting(title):
        if ":" in title:
            title = title.replace(":", " -")
        if "?" in title:
            title = title.replace("?", "")
        if "/" in title:
            title = title.replace("/", "_")
        if "|" in title:
            title = title.replace("|", "_")
        if '"' in title:
            title = title.replace('"', "'")

        return slugify(title)
