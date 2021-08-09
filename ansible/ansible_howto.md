# Ansible Guide

This file contains the notes made during the Udemy Course

"Dive into Ansible - From Beginner to Expert in Ansible"

by James Spurin.

Overview of contents:
1. Introduction
2. Setup

## 1. Introduction

Configuration management, software provisioning and application deployment toolset.
Created by Micheal de Haan in 2012, acquired by Red Hat in 2015.
Maintained by Red Hat and the Open Source conmmunity.
It is a toolset comprising many components:
- Ansible modules
- Ansible executable
- Ansible playbook
- Ansible inventories

### Modules
We have modules for everything; if a module doe snot exist, we can create it:
- Cloud computing
- Networking
- Virzualization
- Containers
- ...

### Ansible CLI
A central components is the `ansible` executable: CLI for using Ansible; we can also access the modules.


### Ansible playbook
Ansible playbook: human-readable deplyment and orchestration language.

### Ansible inventories
Ansible inventories: collections of targets, which can be dynamic and can relate to many components, such as hosts, network switches, etc.

## 2. Setup

Steps:
1. we need to install docker (desktop) and docker-compose.
2. we set our ansible lab with `docker-compose`.
3. we configure SSH connectivity between hosts.

### Installing Ansible Lab

Clone the repositories:
- [course code](https://github.com/spurin/diveintoansible)
- [ansible lab](https://github.com/spurin/diveintoansible-lab)

```bash
cd ~/git_repositories
git clone git@github.com:spurin/diveintoansible.git
git clone git@github.com:spurin/diveintoansible-lab.git
# So we have two folders: diveintoansible/ and diveintoansible-lab/
```

The instructor recommends to to clone them to `~/`, but I did it on `~/git_repositories`.

Modify the `.../diveintoansible-lab/.env` file to contain the correct paths in `CONFIG` and `ANSIBLE_HOME` variables:

```bash
# Shared config volume
CONFIG=/Users/mxagar/git_repositories/diveintoansible-lab/config

# Shared home directories
ANSIBLE_HOME=/Users/mxagar/git_repositories/diveintoansible-lab/ansible_home
```

Run the lab:

```bash
cd git_repositories/diveintoansible-lab
# We have our docker-compose.yaml in here
docker-compose up
# Open browser on http://localhost:1000/
# Course lab opens
# Keep the terminal
# To shut down the lab, Ctrl+C and rm
docker-compose rm
```

