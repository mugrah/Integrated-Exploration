#!/usr/bin/env python3
import argparse
from optparse import OptionParser
import inspect
import subprocess
import pathlib
import json
import os
import shutil
import glob

ALPHA = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
BETA = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
N_SIZE = range(1, 10, 2)
SIGMA = [0.1, 0.3, 0.5, 0.7, 0.9]

def run_roslaunch(n_robot, world, alpha, beta, gama1, gama2, gama3, n_size, sigma):
    world_file = str(pathlib.Path().absolute().parent) + '/world/' + world
    command = 'roslaunch pioneer3at main.launch'
    command = command + ' world_file:=' + world_file

    for i in range(0,n_robot):
        command = command + ' robot' + str(i) + ':=true'

    command = command + ' alpha:=' + str(alpha)
    command = command + ' beta:=' + str(beta)
    command = command + ' gama1:=' + str(gama1)
    command = command + ' gama2:=' + str(gama2)
    command = command + ' gama3:=' + str(gama3)
    command = command + ' n_size:=' + str(n_size)
    command = command + ' sigma:=' + str(sigma)
    
    process = subprocess.Popen([command], shell=True)
    process.wait()
    return process.returncode

def finish_run(config, n_robot, alpha, beta, n_size, sigma, run):
    source_dir = str(pathlib.Path().absolute().parent) + '/maps/'
    config_dir = str(pathlib.Path().absolute()) + '/files/' + config.split('.')[0] + '/'
    target_dir = config_dir + str(n_robot) + '_' + str(alpha) + '_' + str(beta) + '_' + str(n_size) + '_' + str(sigma) + '_' + str(run) + '/'
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    file_names = os.listdir(source_dir)

    for file_name in file_names:
        shutil.move(os.path.join(source_dir, file_name), target_dir)


def main(config):
    
    with open(str(pathlib.Path().absolute()) + '/config/' + config) as json_file:
        specs = json.load(json_file)
    if specs['alpha'] == -1:
        specs['alpha'] = ALPHA
    if specs['beta'] == -1:
        specs['beta'] = BETA
    if specs['n_size'] == -1:
        specs['n_size'] = N_SIZE
    if specs['sigma'] == -1:
        specs['sigma'] = SIGMA
    for n_robot, world_file in zip(specs['n_robot'], specs['world']):
        for alpha, beta, n_size, sigma in zip(specs['alpha'], specs['beta'],specs['n_size'], specs['sigma']):
            for run in range(0,10):
                config_dir = str(pathlib.Path().absolute()) + '/files/' + config.split('.')[0] + '/'
                target_dir = config_dir + str(n_robot) + '_' + str(alpha) + '_' + str(beta) + '_' + str(n_size) + '_' + str(sigma) + '_' + str(run) + '/'
                
                code = -1
                if not os.path.exists(target_dir):
                    print(target_dir)
                    code = run_roslaunch(n_robot, world_file, alpha, beta, specs['gama1'], specs['gama2'], specs['gama3'], n_size, sigma)
                else:
                    maps = [f for f in glob.glob(target_dir + "/*.png")]
                    if len(maps) <= n_robot:
                        print(target_dir)
                        fl = glob.glob(target_dir + "*")
                        for f in fl:
                            os.remove(f)
                        code = run_roslaunch(n_robot, world_file, alpha, beta, specs['gama1'], specs['gama2'], specs['gama3'], n_size, sigma)
                        
                if code == 0:
                    finish_run(config, n_robot, alpha, beta, n_size, sigma, run)


if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option(
        '-C',
        '--config',
        type=str,
        dest='config',
        default='single_robot_dist_cost',
        help='config file. should be on the config folder.')
    
    (options, args) = parser.parse_args()
    main(
        config=options.config,
    )