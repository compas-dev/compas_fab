import os
import json
from os import listdir
from os.path import isdir, join
from datetime import datetime

import compas
import compas.geometry as cg
from compas.numerical.ga import ga
from compas.geometry.transformations import matrix_from_frame
from compas.plotters.gaplotter import visualize_evolution

import numpy as np
from scipy import stats

from math import cos
from math import pi


import warnings
warnings.filterwarnings("ignore")   #FutureWarning: Using a non-tuple sequence for multidimensional indexing is deprecated...



# import calibration procedure data
filtered_data = []

calibration_records_folder = r'20190520_01'
main_folder = r'/home/administrator/workspace/fts_calibration/calibration_records/'
data_folder = main_folder + calibration_records_folder + r"/"
if (not isdir(data_folder)):
    raise Exception("ERROR:", data_folder, "does NOT exist!")

data_files = listdir(data_folder)
for f_idx, data_file in enumerate(data_files):

    poses = []
    wrenches = []

    file_path = join(data_folder, data_file)

    try:
        with open(file_path) as json_file: 
            data_entry = json.load(json_file)

            poses = data_entry['poses']
            poses_trimmed_mean = stats.trim_mean(poses, 0.15).tolist()     # Trim 15% at both ends
            #print(poses_trimmed_mean)

            wrench = data_entry['wrench']
            wrench_trimmed_mean = stats.trim_mean(wrench, 0.15).tolist()     # Trim 15% at both ends
            #print(wrench_trimmed_mean)

            if poses and wrench:
                filtered_data.append({
                    'pose' : poses_trimmed_mean,
                    'wrench' : wrench_trimmed_mean
                })
    except:
        continue




# global variables
gravity_magnitude = 9.80665     #[m/s2]
g_vector_W = cg.Vector(0.0, 0.0, -gravity_magnitude)


def standard_deviation_fit_function(X):
    MASS, CoM_X, CoM_Y, CoM_Z = X
    
    biases = []

    for data_entry in filtered_data:

        __pose = data_entry['pose']
        sensor_frame = cg.Frame.from_quaternion(__pose[3:7], point = __pose[0:3])
        __wrench = data_entry['wrench']
        FTS_F_vec = __wrench[0:3]
        FTS_T_vec = __wrench[3:6]


        ### F gravity compensation
        ### F = mass * gravity
       
        # transform gravity to sensor coordinate system
        sensor_frame_transf = cg.Transformation.from_frame_to_frame(sensor_frame, cg.Frame.worldXY())
        g_vector_FTS = g_vector_W.transformed(sensor_frame_transf)
        #print("Gravity vector in FTS coordinate system =", g_vector_FTS)

        force_at_CoM_FTS = g_vector_FTS * MASS
        #print("force_at_CoM (sensor) = ", force_at_CoM_FTS)
        #print("force_at_CoM (sensor) length = ", force_at_CoM_FTS.length)

        F_bias_vec = np.subtract(FTS_F_vec, force_at_CoM_FTS).tolist()
        #print("F_bias_vec = ", F_bias_vec)


        ### T gravity compensation
        m_times_r = np.multiply( MASS, [CoM_X, CoM_Y, CoM_Z] ).tolist()
        torque_at_CoM = cg.basic.cross_vectors(m_times_r, g_vector_FTS)

        T_bias_vec = np.subtract(FTS_T_vec, torque_at_CoM).tolist()
        #print("T_bias_vec =", T_bias_vec)

        biases.append( tuple(F_bias_vec + T_bias_vec) )
    
    # max standard deviation
    standard_deviation = np.std(biases, axis=0).tolist()
    return max(standard_deviation)



def get_step_size(bin_dig, delta):
    binary = ''
    for i in range(bin_dig):
        binary += '1'
    value = 0
    for i in range(bin_dig):
        value = value + 2**i
    print('number of steps = {:.1f}'.format(value) )
    print('step size       = {:.16f}'.format(delta / float(value)) )
    print()


# Genetic Algorithm settings:
fit_function = standard_deviation_fit_function
fit_type = 'min'
num_var = 4
boundaries = [
    (3.0, 20.0),
    (-0.05, 0.05),
    (-0.05, 0.05),
    (0.0, 0.3)
    ]
bin_dig = 40
num_bin_dig  = [bin_dig] * num_var
output_path = os.path.join(main_folder, 'ga_out/')
min_fit = 0.000001

# output folder
if not os.path.exists(output_path):
    os.makedirs(output_path)

# step sizes
for b in boundaries:
    delta = abs(b[1] - b[0])
    print("boundary", b, ":")
    get_step_size(bin_dig, delta)


# Genetic Algorithm function
# https://compas-dev.github.io/main/api/generated/compas.numerical.ga.html?highlight=ga#compas.numerical.ga
ga_ = ga(
    fit_function,
    fit_type,
    num_var,
    boundaries,
    num_gen = 50,
    num_pop = 500,
    num_elite = 30,
    num_bin_dig = num_bin_dig,
    output_path = output_path,
    min_fit = None,
    mutation_probability = 0.03,
    n_cross = 2)


#results
best = ga_.current_pop['scaled'][ga_.best_individual_index]
estimated_mass = best[0]
estimated_CoM = best[1:4]
print("\nEstimated mass =", estimated_mass)
print("Estimated center of mass =", estimated_CoM)

# output results to json file
res = {
    'timestamp': str(datetime.now().strftime("%Y%m%d-%H%M%S")),
    'calibration_records': str(calibration_records_folder),
    'mass (Kg)': float(estimated_mass),
    'CoM (meters)' : {
        'x': estimated_CoM[0],
        'y': estimated_CoM[1],
        'z': estimated_CoM[2],
    }
}
results_json_file = main_folder + calibration_records_folder + r".json"
with open(results_json_file, "w", encoding="utf8") as json_file:
	json.dump(res, json_file, sort_keys=False, indent=2, separators=(',', ': '))


# visualize plotting
visualize_evolution(ga_.output_path)