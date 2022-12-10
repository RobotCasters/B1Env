import os


def getDataPath():
    """
    Get the path where the package is installed. This can be used to search for other file in this repo.

    :return: installation path.
    :rtype: str
    """
    resdir = os.path.join(os.path.dirname(__file__))
    return resdir
