import os

__all__ = [
    'list_files_in_directory'
]


def list_files_in_directory(directory, fullpath=False, extensions=[]):
    """This function lists just the files in a directory, not sub-directories.

    Args:
        directory (str): the directory to search for files.
        fullpath (:obj:`bool`, optional): specifies if the returned list of
            strings is with the full path. Defaults to False.
        extensions (:obj:`list` of :obj:`str`, optional): a list of allowed
            extensions, e.g. ["jpg", "png"] if you just want to list images.
            Defaults to empty list.

    Returns:
        files (:obj:`list` of :obj:`str`): A list of files as string if files
            exist, or empty list.
    """
    directory = os.path.abspath(directory)
    files = []
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

    path = os.path.join(os.path.dirname(__file__), "..",
                        "robots", "ur", "ur10", "model")
    os.listdir(path)
    print(list_files_in_directory(path, fullpath=True, extensions=["obj"]))
