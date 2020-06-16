import csv
import json
import pickle

__all__ = [
    'read_csv_to_dictionary',
    'write_data_to_json',
    'read_data_from_json',
    'write_data_to_pickle',
    'read_data_from_pickle'
]


def read_csv_to_dictionary(csvfile, delimiter=';'):
    """Reads a csv file and returns a dictionary with the respective keys
    specified in the first row of the csv file.

    Parameters
    ----------
    csvfile : str
        The path to csv file.
    delimiter : str, optional
        The character used to separate the values. Default ``;``

    Returns
    -------
    dict
    """
    data = []
    with open(csvfile, mode='r') as infile:
        reader = csv.reader(infile, delimiter=delimiter)
        for rows in reader:
            data.append(rows)
        infile.close()
    data = zip(*data)  # transpose data
    data_dict = {}
    for col in data:
        key = col[0]
        values = list(col[1:])
        data_dict.update({key: values})
    return data_dict


def write_data_to_json(data, file):
    """Write data to json file.

    Parameters
    ----------
    data : dict, list, tuple, str, unicode, int, long, float, boolean, None
        The data to write to json file. Data must be JSON serialisable.
    file : str
        The path where to save the data.
    """
    with open(file, 'w') as f:
        json.dump(data, f)


def read_data_from_json(file):
    """Read data from json file.

    Parameters
    ----------
    file : str
        The path to the json file.

    Returns
    -------
    object
        An object containing the deserialised data from the json file.
    """
    with open(file) as f:
        data = json.load(f)
    return data


def write_data_to_pickle(data, file):
    """Write data to pickle file.

    Parameters
    ----------
    data : object
        The data to write to a pickle file.
    file : str
        The path where to save the data.
    """
    with open(file, 'wb') as f:
        pickle.dump(data, f)


def read_data_from_pickle(file):
    """Read data from pickle file.

    Parameters
    ----------
    file : str
        The path to the pickle file.

    Returns
    -------
    object
        An object containing the reconstituted object hierarchy.
    """
    with open(file, 'rb') as f:
        data = pickle.load(f)
    return data
