import csv

def read_csv_to_dict(csvfile, delimiter=';'):
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