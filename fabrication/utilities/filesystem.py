'''
Created on 06.06.2017

@author: rustr
'''

import os

def list_files_in_directory(dir, fullpath=False, extensions=[]):
    """
    This function lists just the files in a directory, not subdirectories.
    Optional is a list of allowed extensions, e.g. ["jpg", "png"] if you just 
    want to list images.
    
    Returns:
        A list of files as string, optional with its full path. 
    """
    files = []
    extensions = [".%s" % ext for ext in extensions if ext[0] != "."]
    
    for item in os.listdir(dir):
        item_fullpath = os.path.join(dir, item)
        if os.path.isfile(item_fullpath):
            if len(extensions):
                found = reduce(lambda x, y: x or y, [item.endswith(ext) for ext in extensions])
                if not found:
                    continue
            if fullpath:
                files.append(item_fullpath)
            else:
                files.append(item)
    return files


if __name__ == "__main__":
    
    path = r"C:\Users\rustr\workspace\compas_fabrication\fabrication\robots\ur\ur10\model\visual"
    
    print list_files_in_directory(path, fullpath=True, extensions=["obj"])
    

    