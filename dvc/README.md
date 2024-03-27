# DVC Guide

This is a small guide on how to use [DVC](https://dvc.org/), focusing on its use as artifact versioning/tracking tool.

Additionally, you can check my notes on the Udacity ML DevOps Nanodegree, which briefly introduces the topic, [mlops_udacity](https://github.com/mxagar/mlops_udacity):

- [Reproducible Model Workflows](https://github.com/mxagar/mlops_udacity/blob/main/02_Reproducible_Pipelines/MLOpsND_ReproduciblePipelines.md)
- **[Deploying a Scalable ML Pipeline in Production](https://github.com/mxagar/mlops_udacity/blob/main/03_Deployment/MLOpsND_Deployment.md)**

Table of contents:

- [DVC Guide](#dvc-guide)
  - [1. Introduction and Setup](#1-introduction-and-setup)
    - [Installation](#installation)
    - [Resemblance to Git](#resemblance-to-git)
  - [2. Tracking with DVC: Local Remote/Storage](#2-tracking-with-dvc-local-remotestorage)
  - [3. Remote Storage](#3-remote-storage)
    - [3.1 GDrive Remote Storage](#31-gdrive-remote-storage)
    - [3.2 AWS S3 Remote Storage](#32-aws-s3-remote-storage)
      - [Create a new S3 Bucket to use as your DVC remote](#create-a-new-s3-bucket-to-use-as-your-dvc-remote)
      - [Create IAM credentials to give DVC access to just this bucket](#create-iam-credentials-to-give-dvc-access-to-just-this-bucket)
      - [Configure the AWS CLI with your IAM credentials](#configure-the-aws-cli-with-your-iam-credentials)
      - [Push your data up to S3 using DVC](#push-your-data-up-to-s3-using-dvc)
  - [Links](#links)

## 1. Introduction and Setup

DVC = Data Version Control. We use it to control the version of:

- data
- models
- etc.

either (1) locally or (2) remotely. Additionally, we can use it to 

- create pipelines 
- and track experiments.

DVC relies on Git to do the version control and it doesn't store the data/models itself, but that is done by external or remote services like S3, HDFS, GDrive, etc.

DVC provides tools to track changes in large/binary files.

Links:

- Documentation: [https://dvc.org/doc/start](https://dvc.org/doc/start).
- Installation: [https://dvc.org/doc/install](https://dvc.org/doc/install).

### Installation

There are several ways of installing DVC; we need an environment with Python 3.8+ to run the latest version, which is necessary for some remote storage functionalities; here a quick recipe with the [conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) environment manager and [pip-tools](https://github.com/jazzband/pip-tools):

```bash
# Set proxy, if required

# Create environment, e.g., with conda, to control Python version
conda create -n dvc python=3.10 pip
conda activate dvc

# Install pip-tools
python -m pip install -U pip-tools

# Generate pinned requirements.txt
pip-compile requirements.in

# Install pinned requirements, as always
python -m pip install -r requirements.txt

# DVC: I put the DVC installation explicitly
pip install -U "dvc[all]" # all: all remote storage interfaces are installed

# AWS CLI: I had to install it manually
pip install -U awscli

# If required, add new dependencies to requirements.in and sync
# i.e., update environment
pip-compile requirements.in
pip-sync requirements.txt
python -m pip install -r requirements.txt

# To delete the conda environment, if required
conda remove --name dvc --all
```

Also, note that there is a VSCode extension for DVC!

### Resemblance to Git

DVC is designed to have a very similar use as git:

```bash
# Initialize project, ./dvc/ folder is created
dvc init

# Add files/folders to tracking
dvc add ...

# Upload download data from the remote store, specified in dvc.yaml
dvc push
dvc pull

# This one is different than git commit
dvc commit
```

[DVC Reference](https://dvc.org/doc/command-reference): Typical workflow:

- `dvc init`
- `dvc remote add ...`
- `dvc add ...`
  - `*.dvc` files of the artifacts are created
  - These files are committed, but originals are pushed to the remote/storage
- Optional: Create a `dvc.yaml` file, if necessary/required:
  - This file contains the processing pipeline + artifact outputs
- `dvc repro`: execute or restore any version of the pipeline
- `dvc push`, `dvc pull`: access remote storage

## 2. Tracking with DVC: Local Remote/Storage

```bash
# 0. Make sure we're on a git repo;
# if not, initialize first git, then dvc
git init
dvc init # --no-scm if no git 
# dvc init generates:
# .dvc/ folder
# .dvcignore: equivalent to .gitignore

# 1. Create a local remote folder
mkdir ./data/remote
dvc remote add -d localremote ./data/remote
# list available remotes
dvc remote list
# In addition to local remote folders, we can use
# real remote storage: S3, GDrive, etc.
# Check how config file changed
cat .dvc/config

# 2. Track files
dvc add data/sample.csv
# We get a prompt saying that
# we should add .gitignore
# and sample.csv.dvc to git
# NOTE: sample.csv is added to .gitignore for us!
git add data/sample.csv.dvc .gitignore

# 2. Commit changes to git
git commit -m "initial commit of dataset using dvc"
# If our git repo is connected to a remote origin
# we can always do git push/pull

# 3. Send data to local remote/storage
# The remote storage folder gets a copy of the dvc-added files
# In this example, we can check data/remote/...
# NOTE: the files in the remote storage have a hased name
# but the same content!
dvc push

# 4. Retrieve data from local remote/storage
# In case we don't have the original artifact, it is downloaded
dvc pull

# 5. Change a dataset and track changes
vim data/sample.csv # now, change something
dvc add data/sample.csv
git add data/sample.csv.dvc
git commit -m "changes..."
dvc push

# 6. Manage remotes
# Change/add properties to a remote
dvc remote modify
# Rename a remote
dvc remote rename
# Change a defalut remote
dvc remote default # we get the name of the current default remote
dvc remote default <name> # new default remote
```

The `sample.csv.dvc` has content of the following form:

```yaml
outs:
- md5: 82c893581e57f7e84418cc44e9c0a3d0
  size: 3856
  path: sample.csv
```

## 3. Remote Storage

The true potential of DVC is unlocked when we use remote storage; then, we can simply `git clone` any repository anywhere and push/pull remote datasets/models from GDrive, S3, or similar. Therefore, we can develop on one machine a deploy on another one without any issues, because the datasets and models are remotely stored.

Example:

```bash
dvc remote add s3remote s3://a/remote/bucket
```

Note: We can have multiple remote stores!

Links:

- [Data and Model Access](https://dvc.org/doc/start/data-management/data-and-model-access)
- [Supported Storage Types](https://dvc.org/doc/command-reference/remote/add#supported-storage-types)

### 3.1 GDrive Remote Storage

```python
# To work with remote GDrive folders, we need the unique identifier,
# ie., the last long token in the URL after the last slash '/'
# Unique identifier: <UNIQUEID>
# Additionally, dvc-gdrive must be installed (see above)
dvc remote add driveremote gdrive://<UNIQUEID>
dvc remote modify driveremote gdrive_acknowledge_abuse true
# Check how config file changes
cat .dvc/config

# Push to the gdrive remote
# Since the local remote is the default,
# we need to specify the new drive remote
dvc push --remote driveremote
# We open the prompted URL and log in to Google
# or are redirected to a log in window.
# If a verification code is given on the web
# we paste it back on the terminal
# Now, in GDrive, we should see version files

# If we do dvc push,
# it pushes to the default remote,
# which is usually the local remote! 
dvc push

# We can change the default remote
# to be a cloud storage
dvc remote list # we get the list of all remotes: localremote, driveremote
dvc remote default # we get the name of the current default remote
dvc remote default driveremote # new default remote
```

### 3.2 AWS S3 Remote Storage

Source: [Set up an S3 DVC Remote](https://ritza.co/articles/dvc-s3-set-up-s3-as-dvc-remote/).

The process is very simple, equivalent to the local remote/storage; however, we need to create an S3 bucket and an IAM user which hass access to it.

Steps (assuming `awscli` and `dvc-s3` are installed):

> - Create a new S3 Bucket to use as your DVC remote
> - Create IAM credentials to give DVC access to just this bucket
> - Configure the AWS CLI with your IAM credentials
> - Push your data up to S3 using DVC

#### Create a new S3 Bucket to use as your DVC remote

    Sign in with root user to AWS
    Search S3
    Create bucket
        Choose Region
        Choose Name: <bucket-name>
            The bucket name must be unique globally across all AWS accounts.
        Leave default
        Create bucket

#### Create IAM credentials to give DVC access to just this bucket

    Sign in with root user to AWS
    Search IAM
    Users > Create user
        User name: dvc-user
        Next
        Attach policies directly
            We see a list of policies
            If we click on the + sign of any of them (e.g., one relatd to S3)
            we see the JSON definition
            We are going to create JSON manually
        Create Policy
            Policy ditor: JSON
                Copy and paste the JSON below
                after changing the <bucket-name>
                to the one we selected
            Next
            Policy name: AllowFullAccessMyTestDVCStore (or something similar)
            Create
    
            Now, we'll choose the policy we have created for the user
                AllowFullAccessMyTestDVCStore

        If the user creation was stopped, we restart
        and then choose the policy we created
    After the user has been created, we need security credentials
    IAM: Users: dvc-user
        Security credentials
        Access keys: Create access keys
            Download/copy and save credentials
            For instance, in .env
                AWS_ACCESS_KEY_ID=xxx
                AWS_SECRET_ACCESS_KEY=yyy

`AllowFullAccessMyTestDVCStore`:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": ["s3:ListBucket"],
      "Resource": "arn:aws:s3:::<bucket-name>"
    },
    {
      "Effect": "Allow",
      "Action": ["s3:PutObject", "s3:GetObject"],
      "Resource": "arn:aws:s3:::<bucket-name>/*"
    }
  ]
}
```

#### Configure the AWS CLI with your IAM credentials

We need to log in to AWS with the IAM user; to that end, we run `aws configure`, which asks for the key and the secret and stores them in `~/.aws/credentials`:

```bash
aws configure
# AWS Access Key ID [None]: xxx
# AWS Secret Access Key [None]: yyy
# Default region name [None]: (enter)
# Default output format [None]: (enter)

# Later on, we can extend ~/.aws/credentials
# with further credentials and then select the
# profile we'd like to log in with
aws configure --profile profile_name
```

#### Push your data up to S3 using DVC

```bash
# When we are logged into AWS
# we add the remote storage S3 bucket
# Note that we set the S3 bucket as default!
dvc remote add --default datastore s3://<bucket-name>

# List available remotes
dvc remote list

# Now, the workflow is identical to the localremote: add & commit
dvc add data/sample.csv
git add data/sample.csv.dvc .gitignore
git commit -m "initial commit of dataset using dvc"

# Upload to AWS S3
dvc push
# If we check in AWS S3, we should see a new file 
# (with hashed name, but same content)

# Download from AWS
dvc pull
```



## Links

- [Tutorial: Data and Model Versioning](https://dvc.org/doc/use-cases/versioning-data-and-models/tutorial)
- [Continuous Integration and Deployment for Machine Learning](https://dvc.org/doc/use-cases/ci-cd-for-machine-learning)
- [Fast and Secure Data Caching Hub](https://dvc.org/doc/use-cases/fast-data-caching-hub)
- [Machine Learning Experiment Tracking](https://dvc.org/doc/use-cases/experiment-tracking)
- [Machine Learning Model Registry](https://dvc.org/doc/use-cases/model-registry)
- [Data Registry](https://dvc.org/doc/use-cases/data-registry)
- [Set up an S3 DVC Remote](https://ritza.co/articles/dvc-s3-set-up-s3-as-dvc-remote/)
