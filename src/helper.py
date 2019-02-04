class Helper:
    def __init__(self):
        pass

    @staticmethod
    def list_get(l, idx, default):
        try:
            return l[idx]
        except IndexError:
            return default
