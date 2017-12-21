import csv
import json

def read_csv_to_dictionary(csvfile, delimiter=';'):
    """Reads a csv file and returns a dictionary with the respective keys
    specified in the first row of the csv file.
    """
    data = []
    with open(csvfile, mode='r') as infile:
        reader = csv.reader(infile, delimiter=delimiter)
        for i, rows in enumerate(reader):
            data.append(rows)
        infile.close()
    data = zip(*data) # transpose data
    data_dict = {}
    for l in data:
        key = l[0]
        values = list(l[1:])
        data_dict.update({key: values})
    return data_dict


def write_data_to_json(data, file):
    """Write data to json file.
    """
    with open(file, 'w') as f:
        json.dump(data, f)

def read_data_from_json(file):
    """Read data from json file.
    """
    with open(file) as f:    
        data = json.load(f)
    return data