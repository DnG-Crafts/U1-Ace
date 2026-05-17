from . import ace1
from . import ace2

def load_config(config):
    ace_type = config.getint('ace_type', 1)

    if ace_type == 2:
        return ace2.AceDevice(config)
    elif ace_type == 1:
        return ace1.AceDevice(config)
    else:
        raise config.error("ace_device: unsupported ace_type %d (must be 1 or 2)" % ace_type)
