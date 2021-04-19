'''
    This script organizes the image files into class folders.
    Examplary dataset ../data/cat_dog_small is used, which is a reduced (100 samples) dataset from Kaggle.
    All image files are in ../data/cat_dog_small/all and their labels (cat or dog) in ../data/cat_dog_small/cat_dog_small_annotations.xlsx
    After executing this script, two things happen:
    1. Class folders are created and files copied into there according to their labels
        ../data/cat_dog_small/
            train/cat/
            train/dog/
    2. Dataset is split in train and test groups.
    For that, a fraction of the images already ordered in class forlders are moved
    from ../data/cat_dog_small/train/ to ../data/cat_dog_small/test/
    
    Note: for reading XLSX files, pandas needs the correct xlrd version; if that's not possible
    modify the code and use CSVs instead :)
    (c) Mikel Sagardia, 2020.
'''

import os
import shutil
import pandas as pd
import random
import math

def reorganize_samples(dataset_dir = '../data/cat_dog_small/all', annotations = '../data/cat_dog_small/cat_dog_small_annotations.xlsx', num_samples = -1):
    '''
        This function re-organizes all sample files in a train/ folder into class folders.
        If num_samples == -1, all samples are taken, else min(num_samples, total number of samples in dataset_dir)
        An XLSX file contains the annotations to correctly locate the files.
        Note that a sample could belong to several classes.
        A suffix is added to the sample files; it denotes the classes they belong to.
    '''
    # List that needs to be modified if the dataset is changed
    class_names = ['Cat', 'Dog'] # modify this is dataset changes
    samples_in_class = [0 for i in class_names]

    # Create train folder
    root_folder_name = dataset_dir + '/'
    train_folder_name = dataset_dir
    # FIXME - maybe it's more correct  to use os.sep and be system agnostic...
    sep = '/'
    train_folder_name = sep.join(dataset_dir.split(sep)[:-1])+sep+'train'
    try:
        os.mkdir(train_folder_name)
    except FileExistsError:
        pass

    # Create folders to contain samples of specific classes
    folder_names = class_names
    for f in range(len(folder_names)):
        try:
            os.mkdir(train_folder_name + sep + folder_names[f])
        except FileExistsError:
            pass

    # Open XLSX with class ids
    df = pd.read_excel(annotations)
    if num_samples == -1 or num_samples > len(df['Filename']):
        num_samples = len(df['Filename'])

    for i in range(num_samples):
        # Extract values from XLSX
        filename = df['Filename'][i]
        # Initialize class values & names; allow for multi-class arrangement
        class_values = [0 for i in class_names]
        for f in range(len(class_values)):
            class_values[f] = df[class_names[f]][i]
        total = sum(class_values)

        # Count and generate suffix for new image filename
        suffix = ''
        if total == 0:
            # Sample doesn't belong to any class - maybe there is a default class?
            pass
        else:
            for f in range(len(class_values)):
                if class_values[f] > 0:
                    samples_in_class[f] = samples_in_class[f] + 1
                    label = '_' + class_names[f]
                    suffix = suffix + label

        # Rename image file and move to folders
        if os.path.isfile(root_folder_name + filename):		
            name_parts = filename.split('.')
            new_filename = '_'.join(name_parts[0:-1]) + suffix + '.' + name_parts[-1]
            if total == 0:
                # Sample doesn't belong to any class - maybe there is a default class?
                pass
            else:
                # Create new file with suffix appended
                shutil.copy(root_folder_name + filename, root_folder_name + new_filename)
                # Copy to class folders
                for f in range(len(class_values)):
                    if class_values[f] > 0:
                        new_folder = folder_names[f]
                        shutil.copy(root_folder_name + new_filename, train_folder_name + sep + new_folder + sep + new_filename)
                # Remove new file from upper folder = mv file new_file
                os.remove(root_folder_name + new_filename)

    # Print stats
    print('Samples in each class: ', samples_in_class)

def split_dataset(trainpath, fraction=0.3):
    '''
        This function creates a 'test' folder which duplicates the directories (without files) in 'trainpath'.
        'fraction' random files are moved from the subdirectories in 'trainpath' to 'test'.
        Example:
        split_dataset('./dataset/train', fraction=0.3)
            dataset/
                train/
                    cats/
                        1-fraction
                    dogs/
                        1-fraction
                test/
                    cats/
                        fraction
                    dogs/
                        fraction
    '''
    # Extract all subdirectories in trainpath
    dirs = []
    for (dirpath, dirnames, filenames) in os.walk(trainpath):
        dirs.extend(dirnames)
        break
    # Create test folder
    # FIXME - maybe it's more correct  to use os.sep and be system agnostic...
    sep = '/'
    testpath = sep.join(trainpath.split(sep)[:-1])+sep+'test'
    try:
        os.mkdir(testpath)
    except FileExistsError:
        pass    
    try:
        os.mkdir(testpath)
    except FileExistsError:
        pass
    # Dulicate test subdirectories from trainpath
    # Select fraction of random files and move them
    for d in dirs:
        try:
            os.mkdir(testpath + sep + d)
        except FileExistsError:
            pass
        samplefiles = []
        for (dirpath, dirnames, filenames) in os.walk(trainpath + sep + d):
            samplefiles.extend(filenames)
            break
        indices = list(range(len(samplefiles)))
        random.shuffle(indices)
        num_selected = math.floor(fraction*len(samplefiles))
        print('Moving {} / {} files from {} to {}'.format(num_selected, len(samplefiles), trainpath + sep + d, testpath + sep + d))
        for i in indices[:num_selected]:
            #print(samplefiles[i])
            shutil.move(trainpath + sep + d + sep + samplefiles[i], testpath + sep + d + sep + samplefiles[i])
    return testpath

if __name__ == "__main__":

    # Move sample files to their class folders in train/
    dataset_dir = '../data/cat_dog_small/all'
    annotations = '../data/cat_dog_small/cat_dog_small_annotations.xlsx'
    num_images = -1
    reorganize_samples(dataset_dir = dataset_dir, annotations = annotations, num_samples = num_images)

    # Create train/ and test/ splits
    fraction = 0.3
    trainpath = '../data/cat_dog_small/train'
    split_dataset(trainpath = trainpath, fraction = fraction)