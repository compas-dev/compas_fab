import os

__all__ = ["list_files_in_directory"]


def list_files_in_directory(directory, fullpath=False, extensions=None):
    """This function lists just the files in a directory, not sub-directories.

    Parameters
    ----------
    directory : :obj:`str`
        The directory to search for files.
    fullpath : :obj:`bool`, optional
        Specifies if the returned list of strings is with the full path.
    extensions : :obj:`list` of :obj:`str`, optional
        A list of allowed extensions, e.g. ["jpg", "png"] if you just want to list images.

    Returns
    -------
    :obj:`list` of :obj:`str`
        A list of files as string if files exist, or empty list.
    """
    directory = os.path.abspath(directory)
    files = []
    extensions = extensions or []
    extensions = [".%s" % ext for ext in extensions if ext[0] != "."]
    for item in os.listdir(directory):
        item_fullpath = os.path.join(directory, item)
        if os.path.isfile(item_fullpath):
            if len(extensions):
                found = any([item.endswith(ext) for ext in extensions])
                if not found:
                    continue
            if fullpath:
                files.append(item_fullpath)
            else:
                files.append(item)
    return files


if __name__ == "__main__":

    path = os.path.join(os.path.dirname(__file__), "..", "robots", "ur", "ur10", "model")
    os.listdir(path)
    print(list_files_in_directory(path, fullpath=True, extensions=["obj"]))
