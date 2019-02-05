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
