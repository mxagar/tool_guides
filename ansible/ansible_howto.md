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
1. We need to install docker (desktop) and docker-compose.
2. We set our Ansible Lab with `docker-compose`. Ansible Lab is an instructor's setup for learning Ansible using several hosts run using docker.
3. We configure SSH connectivity between hosts.

### Installing Ansible Lab (Instructor's lab)

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

Modify the `.../diveintoansible-lab/.env` file to contain the correct paths in `CONFIG` and `ANSIBLE_HOME` variables, in my case:
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
# Periodically, pull changes
docker-compose pull
```

On the browser, we can log in to the Ansible Terminal from the menu with username `ansible`, password `password`.

The instructors Ansible Lab lanches these nodes:
- One Ansible Terminal: `ubuntu-c`
- Three Ubuntu hosts: `ubuntu1`, `ubuntu2`, `ubuntu3`
- Three CentOS hosts: `centos1`, `centos2`, `centos3`

The Ansible Terminal is used to practice with Ansible and we should clone the course material in `~`

```bash
# In a Terminal on Mac/Host
cd git_repositories/diveintoansible-lab
docker-compose up
# Open browser at localhost:1000
# Open Ansible Terminal: ubuntu-c
cd ~
git clone https://github.com/spurin/diveintoansible.git
```

### Installing Ansible Library (not covered in the course)

I tried this while following 
[Installing Ansible on macOS](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html#installing-ansible-on-macos).
However, as I understand, it makes moe sense to follow the instructor's setup to learn how to use Ansible, since it creates a cluster of nodes to work on.


```bash
sudo launchctl limit maxfiles unlimited
conda env list # select env, for me, the one I took: 3d
source activate 3d
python -m pip install --user ansible
python -m pip install --user paramiko
```

### Passwordless SSH connections between hosts/nodes

Our goal is to connect automatically and securely to our desired hosts without entering any user passwords. For that:

1. We need to get the **fingerprints** of the nodes/hosts we're going to use to sstablish a secure connection.
2. Then, we are going to generate a **public-private key-pair** and copy the public to the hosts we connect to.

The first connection to a host/node creates a fingeprint, which is saved in `~/.ssh/known_hosts`.
With the fingerprint, we can stablish a secure connection.
For each host, we have two fingerprints: on related to the hostname + one related to the IP.
We always use username `ansible`, password `password` during the course; in the real setting, we'd use different ones.
If we remove `~/.ssh/known_hosts`, we need to repeat the process again.

```bash
# Open Ansible Lab on the browser: localhost:1000
# Open Ansible Terminal: ubuntu-c
# SSH connect to ubuntu1
ssh ubuntu1 # password, yes
# Exit and go back to ubuntu-c
exit
# Check fingerprints: for each host 2: related to hostname, related to IP
cat ~/.ssh/known_hosts 
```

We now create the key-pair and copy the public to our nodes/hosts:

```bash
# On Ansible host: ubuntu-c
# Generated public & private keys
ssh-keygen # accept all defaults
# Check keys are there
ls ~/.ssh/ # private: id_rsa, public: id_rsa.pub
# Check the content of our public key file
cat ~/.ssh/id_rsa.pub
# Now, we can copy the content of id_rsa.pub to the file
# ~/.ssh/authorized_keys @ the hosts and change the permissions
# or we can use the following command, which does all for us:
ssh-copy-id ansible@ubuntu1
# Now, we can log in to the host without re-enetring password!
# We log in to our host and check everything worked
ssh ubuntu1
cat ~/.ssh/authorized_keys
exit
```

We could do that fo all the hosts, but we can automate it:

```bash
sudo apt update # password
sudo apt install sshpass # a tool we are going to use
echo password > password.txt
touch copy_ids.sh
vi copy_ids.sh # See content below
chmod a+x copy_ids.sh
./copy_ids.sh # we get some notification that each pw was copies
rm password.txt
```

`copy_ids.sh`:
```bash
for user in ansible root
do
	for os in ubuntu centos
	do
		for instance in 1 2 3
		do
			# For automatically accepting unkown host fingerprints
			# StrictHostKeyChecking=no
			sshpass -f password.txt ssh-copy-id -o StrictHostKeyChecking=no ${user}@${os}${instance}
		done
	done
done
```

We can use the `ping` module from Ansible to test the system of hosts/nodes:

```bash
# On Ansible host, ubuntu-c
ansible -i,ubuntu1,ubuntu2,ubuntu3,centos1,centos2,centos3 all -m ping
# we get JSON output of correct connections!
```

## Section 3: Ansible Architecture and Design

In this section we
- see how the Ansible **Configuration** is done
- how **Inventories** are defined (hosts/nodes)
- how **Modules** are used (commands to act on nodes: copy, setup, etc.)

We need to clone the course material on the Ansible Terminal host, as explained in Section 1.
The course material is organized by section topics and each topic has revisions (`01`, `02`, ...) that build one on the previous:
- Ansible Architecture and Design
- Ansible Playbooks, Intriduction
- Ansible Playbooks, Deep Dive
- Structuring Ansible Playbooks
- Using Ansible with Cloud Services and Containers
- Creating Modules and Plugins

### 3.1 Configuration

The Ansible configuration file can be placed in different locations.
Each location has a priority, if a file in location `n` is missing, `n+1` is used.
Locations listed in decending priorities:
1. `ANSIBLE_CONFIG`
2. `.../.ansible.cfg`
3. `~/.ansible.cfg`
4. `/etc/ansible/ansible.cfg`


```bash
# Help: see all options
ansible --help
# @ ubuntu-c, ansible host
# Priority 4 config file: system
ansible --version # no configuration file, we create one
su - # password
mkdir /etc/ansible
touch /etc/ansible/ansible.cfg
exit # exit sudo
ansible --version # now, we have a configuration file
# Priority 3 config file: home
cd ~
touch .ansible.cfg
ansible --version # now, we have another configuration file, with higher prio
# Priority 2 config file: local folder
touch ./ansible.cfg
ansible --version
# Priority 1: environment variable
mkdir ~/test/
touch ~/test/ansible_config.cfg
export ANSIBLE_CONFIG=/home/ansible/test/ansible_config.cfg
ansible --version
# Remove config files if not necessary
sudo rm /etc/ansible/ansible.cfg
rm ~/.ansible.cfg
rm ~/ansible.cfg
rm ~/test/ansible_config.cfg
```

### 3.2 Ansible Inventories


```bash
# In a Terminal on Mac/Host
cd git_repositories/diveintoansible-lab
docker-compose up
# Open browser at localhost:1000
# Open Ansible Terminal: ubuntu-c
# Check ping works
ping centos1
# After cloning (see above)
#
# Ansible Architecture and Design - Inventories - 01
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/01
ls # ansible.cfg hosts
# Both files are in INI format (see below)
# ansible.cfg maps: inventory = hosts
cat ansible.cfg
# hosts shows which all hosts to work with are to ansible
cat hosts # [all] centos1
# If we remove known_hosts, we'll need the fingerprint to ping
rm -f ~/.ssh/known_hosts
ansible all -m ping # we can accept fingerprint or for not to
# Unix tip: define env variable before commend
# for var to be effective on command only
# For disabling fingerprint requirement: ANSIBLE_HOST_KEY_CHECKING=False
ANSIBLE_HOST_KEY_CHECKING=False ansible all -m ping
#
# Ansible Architecture and Design - Inventories - 02
# Now, we have the conf value 'host_key_checking = False' @ ansible.cfg
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/02
cat ansible.cfg # host_key_checking = False
ansible all -m ping # now it works without asking fingerprints even without known_hosts
#
# Ansible Architecture and Design - Inventories - 03
# Now, we have all hosts in groups [ubuntu] and [centos]
# By default a group [all] is created which contains all inventories/hosts
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/03
ansible all -m ping
# We can now target our groups, not only the default [all]
ansible ubuntu -m ping # [ubuntu]
ansible centos -m ping # [centos]
ansible '*' -m ping # all groups
# Further commands and options
ansible all -m ping -o # condensed view
ansible ubuntu --list-hosts # lists hosts in [ubuntu] group
ansible all --list-hosts # lists all hosts
# we can also use regular expressions
ansible *1 --list-hosts # lists all hosts ubuntu1 centos1
#
# Ansible Architecture and Design - Inventories - 04
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/04
# Now, centos* hosts have ansible_user=root
cat hosts
id # ansible user is shown
# We display it fo all hosts
ansible all -m command -a 'id' -o # we see that centos* has root user
#
# Ansible Architecture and Design - Inventories - 05
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/05
# Now, ubuntu hosts have 'ansible_become=true ansible_become_pass=password'
# That way, we log in with user ansible but grant premission to escalate to sudo
cat hosts
# The command module is the default
# so if we ommit '-m', that is what is used
ansible all -a 'id' -o # same as with -m command
#
# Ansible Architecture and Design - Inventories - 06 & 07
# By default, all hosts are running sshd on port 22
# We can change that in the docker-compose.yaml file
# Ctrl+C the docker terminal on Mac/Host
vim docker-compose.yaml
# Swap image and port lines of host centos1 to the ones with 2222
docker-compose up
# We open the Ansible Terminal ubuntu-c
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/05
ansible all -m ping -o # pinging to centos1 fails, because Ansible expects default ssh port 22
# Now, we go to revision 06
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/06
# centos1 has variable ansible_port=2222
cat hosts
ansible all -m ping -o # now it works
# We check now revision 07
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/07
# another way of specifying the ssh port is with hostname:port
cat hosts # centos1:2222
ansible all -m ping -o # it works again
#
# Ansible Architecture and Design - Inventories - 08
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/08
# We add ubuntu-c to the host inventory, but allow connection without ssh
cat hosts # [control] ubuntu-c ansible_connection=local
ansible all -m ping -o
#
# Ansible Architecture and Design - Inventories - 09
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/09
# Ranges are used in host names for common names
cat hosts # ubuntu[1:3] ...
ansible all -m ping -o
ansible all --list-hosts
#
# Ansible Architecture and Design - Inventories - 10
# Now, we add new sections in which we define group variables
# That way, variables can be comfortably defined for each group
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/10
cat hosts # ... [centos:vars] ansible_user=root ...
#
# Ansible Architecture and Design - Inventories - 11
# We can define parent-children groups
# We define a linux parent group
# [linux:children]
# centos
# ubuntu
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/11
cat hosts # ... [linux:children] centos ubuntu ...
ansible linux -m ping -o # we ping the linux group
#
# Ansible Architecture and Design - Inventories - 12 & 13
# We can define variables for [all], but specific definitions have priority
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/12
# Example in 12/hosts:
# [all:vars]
# ansible_port=1234
# It won't work, because 1234 is not a valid port for ssh unless defined in docker...
ansible all -m ping -o
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/13
# Example in 13/hosts:
# [linux:vars]
# ansible_port=1234
# It won't work, because 1234 is not a valid port for ssh unless defined in docker...
ansible all -m ping -o
#
# Ansible Architecture and Design - Inventories - 14
# We can use the YAML format for the definition of the inventories
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Inventories/14
cat ansible.cfg # hosts.yaml
cat hosts.yaml
ansible all -m ping -o # works perfectly
#
# Ansible Architecture and Design - Inventories - 15
# We can use the JSON format for the definition of the inventories too
# We can use th efollowing python one-liner for converting from YAML to JSON
python3 -c 'import sys, yaml, json; json.dump(yaml.load(sys.stdin, Loader=yaml.FullLoader), sys.stdout, indent=4)' < hosts.yaml > hosts.json
cat hosts.json
cat ansible.cfg # inventory = hosts.json
ansible all -m ping -o # works perfectly
#
# Ansible Architecture and Design - Inventories - 16
# We can specify the inventory file in the call!
ansible all -i hosts --list-hosts
ansible all -i hosts.yaml --list-hosts
ansible all -i hosts.json --list-hosts
# We can also override variables with -e specifying the group we're acting on
ansible linux -m ping -e 'ansible_port=22'
```

The final `hosts` file from revision `Inventories/16`:
```INI
[control]
ubuntu-c ansible_connection=local

[centos]
centos1 ansible_port=2222
centos[2:3]

[centos:vars]
ansible_user=root

[ubuntu]
ubuntu[1:3]

[ubuntu:vars]
ansible_become=true
ansible_become_pass=password

[linux:children]
centos
ubuntu
```

Note: 
[INI file format](https://en.wikipedia.org/wiki/INI_file)
```INI
# comment, not always supported
; comment
[mysection1]
key1=value1
key2=value2
# comment, not always supported
[mysection2]
key3=value3
key4=value4
```

The final `hosts.yaml` file from revision `Inventories/16`:
```yaml
---
control:
  hosts:
    ubuntu-c:
      ansible_connection: local
centos:
  hosts:
    centos1:
      ansible_port: 2222
    centos2:
    centos3:
  vars:
    ansible_user: root
ubuntu:
  hosts:
    ubuntu1:
    ubuntu2:
    ubuntu3:
  vars:
    ansible_become: true
    ansible_become_pass: password
linux:
  children:
    centos:
    ubuntu:
...
```


The final `hosts.json` file from revision `Inventories/16`:
```json
{
    "control": {
        "hosts": {
            "ubuntu-c": {
                "ansible_connection": "local"
            }
        }
    }, 
    "ubuntu": {
        "hosts": {
            "ubuntu1": null, 
            "ubuntu2": null, 
            "ubuntu3": null
        }, 
        "vars": {
            "ansible_become": true, 
            "ansible_become_pass": "password"
        }
    }, 
    "centos": {
        "hosts": {
            "centos3": null, 
            "centos2": null, 
            "centos1": {
                "ansible_port": 2222
            }
        }, 
        "vars": {
            "ansible_user": "root"
        }
    }, 
    "linux": {
        "children": {
            "centos": null, 
            "ubuntu": null
        }
    }
}
```

### 3.3 Ansible Modules

Ansible modules can be used to configure our hosts.
There are code/builtin modules and we can also create our own modules.
Modules can be called through `ansible` and they produce a JSON output, which is color-coded:
- Red: failure
- Yellow: success, with changes
- Green: success, no changes

Note that the operations in Ansible are **idempotent**: the result of performing an operation once is the same as operating it repeteadly.

We start our hosts and go to the Modules folder of the course:
```bash
# In a Terminal on Mac/Host
cd git_repositories/diveintoansible-lab
docker-compose up
# Open browser at localhost:1000
# Open Ansible Terminal: ubuntu-c
cd /home/ansible/diveintoansible/Ansible Architecture and Design/Modules
# Get module help, code location, etc.
ansible-doc file
ansible-doc setup
ansible-doc fetch
```

#### Setup Module

- Core module, inside `builtin` plugin
- Automatically executed with playbooks to gather information as variables
- It can be executed also with `ansible`

```bash
ansible centos1 -m setup # we get a los of information printed as a JSON
```

#### File Module

- Core module, inside `builtin` plugin
- Removes or sets attributes of files, symlinks, directories
- Many other modules support the same options as `file`; eg: `copy`
- Windows targets: use `win_file`

```bash
# We create/touch a file in a path using the file module
# This is equivalent to dooing it manually with touch
# but we do it on all nodes/hosts!
ansible all -m file -a 'path=/tmp/test state=touch'
ls -la /tmp/test # notice timestamp
# Changing permisions
# User - Group - Other -> first, second, third number
# RWX: 421 -> R+W+X are added, 0 if permision not given, 4/2/1 if R/W/X given
# Examples:
# rwx------ = 700
# r-x--x--x = 411
ansible all -m file -a 'path=/tmp/test state=file mode=600' # output yellow
ls -la /tmp/test # notice permisions
ansible all -m file -a 'path=/tmp/test state=file mode=600' # now, output green, because no change
```

#### Copy Module

- Used to copy files from ansible terminal host to the other hosts

```bash
touch /tmp/x
# We copy the x file from ubuntu-c to all targets/hosts
# Since Ansible is idempotent, copying x from ubuntu-c to itself works (only green output)
ansible all -m copy -a 'src=/tmp/x dest=/tmp/x'
# Copy from host to host
ansible all -m copy -a 'remote_src=yes src=/tmp/x dest=/tmp/y'
```

#### Command Module

- Very useful!
- Core module, inside `builtin` plugin
- It is the default module: if we do not specify any module, `command` is used
- We use it to pass the arguments that we desire
- It is not processed through the shell, thus, env variables and pipe-like operators won't work
- Windows targets: used `win_command`

```bash
# Recall: if no module specified, command is used
# Get hotname of all hosts
ansible all -a 'hostname' -o
# We can touch/create files in all hosts through command
# variable 'creates' is for idempotence: command executed only if it does not exist
# Note that we could touch a file with the file module, too
ansible all -a 'touch /tmp/test_command_module creates=/tmp/test_command_module' -o # yellow
ansible all -a 'touch /tmp/test_command_module creates=/tmp/test_command_module' -o # green: idempotence
# We can remove them too
# option 'removes' is for idempotence: command executed only if file exists
ansible all -a 'rm /tmp/test_command_module removes=/tmp/test_command_module' -o
```

#### Fetch Module

- Reverse of `copy`: files are fetched from remote machines and stored locally in a file tree in which each host/node has a folder

```bash
# We create a file on all hosts and chage its mode/perimision
ansible all -m file -a 'path=/tmp/test_modules.txt state=touch mode=600' -o
# We copy/fetch from remote hosts the files we created
ansible all -m fetch -a 'src=/tmp/test_modules.txt dest=/tmp' -o
cd /tmp
ls # centos1 centos2 centos3 ubuntu1 ubuntu2 ubuntu3
ls /tmp/centos1/tmp # test_modules.txt
```

## Section 4: Ansible Playbooks

Ansible Playbooks are scripts that contain an advanced usage of the `ansible` CLI.
Those playbooks or scripts are written in YAML or JSON, but YAML is the usual way to go.
The scripts are executed with the tool `ansible-playbook`.

### 4.1 YAML

YAML is a data serialization language, very human-readable.
YAML = yet another markup language.
YAML is a series of key-value pairs, that is, dictionaries.
Every subsequent key-value lines are considered part of the same dictionary.
Dictionaries can be nested and can contain the basic types as well as lists of basic types: string, int, bool, real.
Note that we can convert a YAML file into python structures; this section is basically about that conversion and correspondence.
There is a one-liner that converts a YAML to python in the examples:

```bash
python3 -c 'import yaml,pprint;pprint.pprint(yaml.load(open("test.yaml").read(), Loader=yaml.FullLoader))'
```

The examples to understand YAML are in the folder `/home/ansible/diveintoansible/Ansible Playbooks, Introduction/YAML`, as shown below; however, the different concepts introduced in them have been collected in the YAML file after this `bash` block:
```bash
# Host/Mac
cd ~/git_repositories/diveintoansible-lab
# We have our docker-compose.yaml in here
docker-compose up
# Open browser on http://localhost:1000/
# Ansible Terminal: ubuntu-c
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/YAML
#
# Ansible Playbooks, Introduction - YAML - 01 ... 19
# Each revision showcases a concept
# All concepts are summarized in the sample YAML sample below
# Note that all revisions contain a YAML file as well a bash script
# which converts the YAML into python code/structures
cd YAML/02
ls # show_yaml_python.sh  test.yaml
cat show_yaml_python.sh # one-liner that converts YAML into python and displays it as text!
# python3 -c 'import yaml,pprint;pprint.pprint(yaml.load(open("test.yaml").read(), Loader=yaml.FullLoader))'
# The YAML in here contains a dictionary like that
# example_key_1: this is a string
# example_key_2: this is another string
cat test.yaml
# We can convert it to python text
./show_yaml_python.sh test.yaml
# {'example_key_1': 'this is a string', 'example_key_2': 'this is another string'}
```

YAML sample lines:
```yaml
# Every YAML file should start with three dashes
---

# subsequent key-value pairs are considered to belong to the same dictionary
# strings can have no quotes, single or double quotes
# but if we want escape characters, double quotes are needed
no_quotes: this is a string example
double_quotes: "this is a string example"
single_quotes: 'this is a string example'
escape: "this is a string with a line break\n"

# if we don't assign a value to a key, it is defined as None
# {'my_key': None}
my_key:

# pipe | inserts \n in the python conversion for every new line
example_key_1: |
	this is the first line
	this is the second line
	this is the third line

# > connects multiple lines into a single and addes \n at the end
# >- does the same, but does not add \n
example_key_2: >-
	this is a first string
	this is a second string
	this is a third string

# integers are automatically interpreted
# but if we want them to be strings, we need to double-quote them
example_integer: 1
example_integer_string: "1"

# booleans are possible in many ways
is_false_01: false
is_false_02: False
is_false_03: FALSE
is_false_04: no
is_false_05: No
is_false_06: NO
is_false_07: off
is_false_08: Off
is_false_09: OFF
is_true_01: true
is_true_02: True
is_true_03: TRUE
is_true_04: yes
is_true_05: Yes
is_true_06: YES
is_true_07: on
is_true_08: On
is_true_09: ON

# item lists are converted into lists
# ['item 1', 'item 2', 'item 3', 'item 4', 'item 5'] 
- item 1
- item 2
- item 3
- item 4
- item 5

# we can also describe the dictionaries in a compact way
# example_key_a: example_value_a
# example_key_b: example_value_b
{example_key_a: example_value_a, example_key_b: example_value_b}

# we can also describe the lists in a compact way
[element_1, element_2]

# (typically) 2 whitespace inentation nests elements
# here we would have a dictionary of dictionaries
example_key_3:
  sub_example_key_3: sub_example_value_3

example_key_4:
  sub_example_key_4: sub_example_value_4

# a dictionary of lists
example_1:
  - item 1
  - item 2
  - item 3

example_2:
  - item 1
  - item 2
  - item 3

# Every YAML file should end with three dots
...
```

### 4.2 Ansible Playbooks: Sections

We have some common sections in a playbook YAML file:
- Hosts/Targets: where our play will run and options it will run with
- Vars: variables that will apply to the play, on all target systems
- Tasks: the list of tasks that will be executed within the play, this section can also be used for pre and post tasks
- Handlers: the list of handlers that are executed as a notify key from a task; they are often used for debugging.
- Roles: list of roles to be imported into the play


```bash
# Host/Mac
cd ~/git_repositories/diveintoansible-lab
# We have our docker-compose.yaml in here
docker-compose up
# Open browser on http://localhost:1000/
# Ansible Terminal: ubuntu-c
cd /home/ansible/diveintoansible/Ansible Playbooks, Breakdown of Sections/01
cat motd_playbook.yaml # see content below
```

Typical YAML structure:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list 
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with

  # Vars: variables that will apply to the play, on all target systems

  # Tasks: the list of tasks that will be executed within the play, this section
  #       can also be used for pre and post tasks

  # Handlers: the list of handlers that are executed as a notify key from a task

  # Roles: list of roles to be imported into the play

# Three dots indicate the end of a YAML document
...
```

The instructor explains how to create a playbook for modifying the 'message of the day' or 'MOTD' that appears as greeting when connecting to Linux systems.

```bash
# Ansible Terminal: ubuntu-c
cd /home/ansible/diveintoansible/Ansible Playbooks, Breakdown of Sections/02
cat motd_playbook.yaml # see content below
# The YAML file defines a task in which
# a MOTD file (centos_motd) is copied to the location where the system MOTD should be: /etc/motd
#
# We execute the playbook as follows
# Color doing is the same as before: yellow, green, red
# Note that in addition to the task we define,
# an extra task of collecting facts is carried out
ansible-playbook motd_playbook.yaml
# If we open a centos terminal from localhos:1000 in the browser, we'll see our MOTD:
# Welcome to CentOS Linux - Ansible Rocks
# Note that by default 'facts are gathered' when executing the playbook:
# setup module is run and checks are performed
```

YAML file `motd_playbook.yaml` for changing the message of the day when logging in to a CentOS node:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list 
# of plays, with each play being a dictionary
-
 
  # Hosts/Targets: where our play will run and options it will run with
  hosts: centos
  user: root
  gather_facts: False # default is True; set True if we want to run the setup module every time for every node

  # Vars: variables that will apply to the play, on all target systems
  motd: "Welcome to CentOS Linux - Ansible Rocks"

  # Tasks: the list of tasks that will be executed within the playbook
  tasks:
    - name: Configure a MOTD (message of the day)
      copy:
        src: centos_motd
        # Instead of suing the file centos_motd
        # we can also define the content key with its string:
        # content: Welcome to CentOS Linux - Ansible Rocks
        # or we can create a var above and use it with this (ginger2) notation:
        # content: "{{ motd }}"
        dest: /etc/motd
      # notify is a handler, not necessary, but outputs a message
      notify: MOTD changed

  # Handlers: the list of handlers that are executed as a notify key from a task
  # handlers are often used for debugging
  handlers:
    - name: MOTD changed
      debug:
        msg:: The MTD was changed

  # Roles: list of roles to be imported into the play

# Three dots indicate the end of a YAML document
...
```

In the file above, tree ways of changing the massage of the day are shown: 
- from a file: `src: centos_motd
- as a string defined in `content`: `content: Welcome to CentOS Linux - Ansible Rocks`
- as a variable used in `content`: `content: "{{ motd }}"`

If we use variables, these can be changed from the command line:

```bash
ansible-playbook motd_playbook.yaml -e 'motd="Testing the motd playbook\n"'
```

We can add conditionals to the task execution based on the information or facts available, for instance, when performing the default initial setup.
That is achieved adding the `when` directive to the tasks.
When need to remove `gather_facts: False` and first check all available facts gathered with the `setup` module.
In the following example, we define a different MOTD depending on the host type: `ubuntu` or `centos`:

```bash
# We check all facts/properties available with the setup module
# We scroll down with ENTER until finding sth we could use for 
# "ansible_distribution": "CentOS"
ansible all -i centos2, -m setup | more
# We check that fact is different for ubuntu
ansible all -i centos2,ubuntu2, -m setup | grep ansible_distribution
# Now, in the playbook YAML, we add the when directive
# See example here
cd /home/ansible/diveintoansible/Ansible Playbooks, Breakdown of Sections/07
cat motd_playbook.yaml
ansible-playbook motd_playbook.yaml
# If we log in to the ubuntu/centos hosts, we check it works
```

`07/motd_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---
 
# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-
 
  # Hosts: where our play will run and options it will run with
  hosts: linux
 
  # Vars: variables that will apply to the play, on all target systems
  vars:
    motd_centos: "Welcome to CentOS Linux - Ansible Rocks\n"
    motd_ubuntu: "Welcome to Ubuntu Linux - Ansible Rocks\n"
 
  # Tasks: the list of tasks that will be executed within the playbook
  tasks:
    - name: Configure a MOTD (message of the day)
      copy:
        content: "{{ motd_centos }}"
        dest: /etc/motd
      notify: MOTD changed
      when: ansible_distribution == "CentOS"

    - name: Configure a MOTD (message of the day)
      copy:
        content: "{{ motd_ubuntu }}"
        dest: /etc/motd
      notify: MOTD changed
      when: ansible_distribution == "Ubuntu"
 
  # Handlers: the list of handlers that are executed as a notify key from a task
  handlers:
    - name: MOTD changed
      debug:
        msg: The MOTD was changed
 
  # Roles: list of roles to be imported into the play
 
# Three dots indicate the end of a YAML document
...
```

More information:
[Ansible Playbook Keywords](https://docs.ansible.com/ansible/latest/reference_appendices/playbooks_keywords.html).

### 4.3 Ansible Playbooks: Variables

