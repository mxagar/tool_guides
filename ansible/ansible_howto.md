# Ansible Guide

This file contains the notes made during the Udemy Course

"Dive into Ansible - From Beginner to Expert in Ansible"

by James Spurin.

Notes made by Mikel Sagardia, 2021.
No warranties.

Overview of analyzed and tested contents:

1. Introduction
2. Setup
3. Ansible CLI & Architecture
  - Configuration
  - Inventories
  - Modules
4. Ansible Playbooks, Introduction
  - YAML
  - Sections
  - Variables
  - Facts
  - Templating with Jinja2
7. Ansible with Docker
9. My personal summary / notes

Overview of sections I just watched, not tested; for those sections, I just write a summary of what the instructor explains:

5. Ansible Playbooks, Deep Dive
  - Modules
  - Dynamic Inventories
  - Register and When
  - Looping
  - Asynchronous, Serial, Parallel
  - Task Delegation
  - Magic Variables
  - Blocks
  - Vault
6. Structuring Ansible Playbooks
  - Include & Import
  - Tags
  - Roles
7. Using Ansible with Cloud Services and Containers
8. Creating Modules and Plugins


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

A reference link to all keywords in playbooks:
[Ansible Playbook Keywords](https://docs.ansible.com/ansible/latest/reference_appendices/playbooks_keywords.html).

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

### 4.3 Ansible Playbooks: Variables

Within the playbook, we can define and use variables of different types: lists, dictionaries, strings.

The folder that showcases the variable capabilities in the playbooks is:

`/home/ansible/diveintoansible/Ansible Playbooks, Introduction/Ansible Playbooks, Variables`

There is a script which runs all revisions in that folder, showing all posibilities:

```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Ansible Playbooks, Variables
# Show examples one after the other
./show_examples.sh
```
Summary of capabilities, odered by revisions:
- `01`: We can define a variable `example_key: example value` in the `vars` section and use it later with `"{{ example_key }}"` 
- `02`: We can define a dictionary `dict` as variable and use them as `"{{ dict }}"` or `"{{ dict.dict_key }}"` or `"{{ dict['dict_key'] }}"`
- `03`: We can also define inline dictionaries: `{inline_dict_key: This is an inline dictionary value}`
- `04`: We can define a list (`named_list`) with items (`-`) and access them with `"{{ named_list }}"`, `"{{ named_list.0 }}"` and `"{{ named_list[0] }}"
- `05`: Same as above but with th einline notation for lists: `[ item1, item2, item3, item4 ]`
- `06`: We can create `external_vars.yaml` and define variables in there; then, we include the file in the `vars` section under the variable `var_files`, and we have access to the external variables!
- `07`: We can ask the user the value of a variable with `var_promt`
- `08`: Same as before, but we add the flag `private: True` under `var_promt` so that the typed text is not visualized (eg., for passwords)
- `09`: There is always a `hostrvars` dictionary created which we can use to access host setup information with `gather_facts: True`. Example: `"{{ hostvars[ansible_hostname].ansible_port }}"` or `"{{ hostvars[ansible_hostname]['ansible_port'] }}"`
- `10`: Same as before.
- `11`: Same as before, but we set a default value if the variable we're trying to access in `hostvars` does not exist: `"{{ hostvars[ansible_hostname].ansible_port | default('22') }}"`
- `12`, `13`, `14`, `15`: We can also directly access values within `hostvars`, foe example: `"{{ ansible_user }}"`
- `16`, `17`: We can pass variables to the CLI `ansible-playbook` with `-e` using different formats: INI, YAML and JSON (as variable or file)
  ```bash
  ansible-playbook variables_playbook.yaml -e extra_vars_key="extra vars value"
  ansible-playbook variables_playbook.yaml -e {"extra_vars_key": "extra vars value"}
  ansible-playbook variables_playbook.yaml -e {extra_vars_key: extra vars value}
  ansible-playbook variables_playbook.yaml -e @extra_vars_file.json
  ```

### 4.4 Ansible Playbooks: Facts

The `setup` module is automatically run by every playbook execution and `facts` or system/node properties are gathered, unless set set `gather_facts: False`.
Facts are returned in JSON format.

We can filter facts, create custom ones, etc.

Documentation of the facts available in the `setup` module: [Ansible Setup Module](https://docs.ansible.com/ansible/latest/collections/ansible/builtin/setup_module.html).

Note that we get a `ansible_facts` dictionary when running the setup module.
All the keys of that dictionary are added to the variable space directly, we don't need to access them through `ansible_facts`.

```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Ansible Playbooks, Facts
cd 01
# We can gather subsets
ansible centos1 -m setup -a 'gather_subset=network' | more
# We can filter facts, also with wildcards
ansible centos1 -m setup -a 'filter=ansible_mem*'
```

#### Custom Facts

We can define **custom facts** to be returned in JSON or INI format; examples:

Example in JSON `getdate1.fact`:
```bash
#!/bin/bash
echo {\""date\"" : \""`date`""}
```

Example in INI `getdate2.fact`:
```bash
#!/bin/bash
echo [date]
echoh date=`date`
```

```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Ansible Playbooks, Facts
cd 02/templates/
ls # getdate1.fact getdate2.fact
# First, we need to create the standard file for the facts
sudo mkdir -p /etc/ansible/facts.d
sudo cp * /etc/ansible/facts.d/
# We run the setup module and gather facts to check that our custom ones work
# These will be in the dictionary ansible_local
ansible ubuntu-c -m setup | more
# We can filter the dictionary of custom facts, which is ansible_local
ansible ubuntu-c -m setup -a 'filter=ansible_local'
# We can run ansible-playbook limiting the execution to the desired hosts too
cd ../../03
ansible-playbook facts_playbook.yaml -l ubuntu-c
```

Notes:
- We can use any default facts in the YAML: `"{{ ansible_default_ipv4.address }}"`
- We can use the custom facts in the YAML: `"{{ ansible_local.getdate1.date }}"`, `"{{ hostvars[ansible_hostname].ansible_local.getdate1.date }}"`
- Revision `06` shows how to define facts in a directory without root priviledges.

### 4.5 Ansible Playbooks: Templating with Jinja2

Jinja2 is a web template engine for python.
It is used, among others, by Flask and Ansible.
We can basically write code scripts that perform operations such as if/else, loops, etc.
Jinja2 is a topic on its own, and a very extensive one.
The more we know Jinja2, the better we can work with Ansible.
In Ansible, Jinja2 can be used in YAML playbook files and in the configuration files.

Very basic Jinja2 example:
```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Templating with Jinja2
cd 03
cat jinja2_playbook.yaml # see below
ansible-playbook jinja2_playbook.yaml
```

The YAML file above, with a Jinja2 script consisting of an if-statement:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: all

  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Ansible Jinja2 if elif else
      debug:
        msg: >
             --== Ansible Jinja2 if elif else statement ==--

             {% if ansible_hostname == "ubuntu-c" -%}
                This is ubuntu-c
             {% elif ansible_hostname == "centos1" -%}
                This is centos1 with it's modified SSH Port
             {% else -%}
                This is good old {{ ansible_hostname }}
             {% endif %}

# Three dots indicate the end of a YAML document
...
```

Some additional notes:
- As we see, the script happens in between `{% -%}`, and comments appear within `--== ==--`
- Revision `04`: we can use `is defined`: `{% if example_variable is defined -%}`
- Revision `06`: for loops can be used as follows
  ```jinja2
  {% for entry in ansible_interfaces -%}
    Interface entry {{ loop.index }} = {{ entry }}
  {% endfor %}
  ```
- Revision `07`: ranges can be used as follows:
  ```jinja2
  {% for entry in range(1, 11) -%}
    {{ entry }}
  {% endfor %}
  ```

#### Filters

We have access to many filtering capabilities: min, max, unique, difference, random, split

[Playbook filters](https://docs.ansible.com/ansible/latest/user_guide/playbooks_filters.html)

```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Templating with Jinja2/
cd 10/
cat cat jinja2_playbook.yaml
#
#
```

```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: all

  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Ansible Jinja2 filters
      debug:
        msg: >
             ---=== Ansible Jinja2 filters ===---

             --== min [1, 2, 3, 4, 5] ==--

             {{ [1, 2, 3, 4, 5] | min }}

             --== max [1, 2, 3, 4, 5] ==--

             {{ [1, 2, 3, 4, 5] | max }}

             --== unique [1, 1, 2, 2, 3, 3, 4, 4, 5, 5] ==--

             {{ [1, 1, 2, 2, 3, 3, 4, 4, 5, 5] | unique }}

             --== difference [1, 2, 3, 4, 5] vs [2, 3, 4] ==--

             {{ [1, 2, 3, 4, 5] | difference([2, 3, 4]) }}

             --== random ['rod', 'jane', 'freddy'] ==--

             {{ ['rod', 'jane', 'freddy'] | random }}

             --== urlsplit hostname ==--

             {{ "http://docs.ansible.com/ansible/latest/playbooks_filters.html" | urlsplit('hostname') }}

# Three dots indicate the end of a YAML document
...
```

#### Templates as external files

Usually, templating is done by including external Jinja2 files in the `template` dictionary variable.
In the next exammple, all the revisions in the templaing folder are executed together using an external Jinja2 script.

```bash
cd /home/ansible/diveintoansible/Ansible Playbooks, Introduction/Templating with Jinja2/11
cat jinja2_playbook.yaml # see below
cat template.j2 # see below
ansible-playbook jinja2_playbook.yaml -l ubuntu-c
```

`jinja2_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: all

  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Jinja2 template
      template:
        src: template.j2
        dest: "/tmp/{{ ansible_hostname }}_template.out"
        trim_blocks: true
        mode: 0644

# Three dots indicate the end of a YAML document
...
```

`template.j2`:
```jinja2
--== Ansible Jinja2 if statement ==--

{# If the hostname is ubuntu-c, include a message -#}
{% if ansible_hostname == "ubuntu-c" -%}
      This is ubuntu-c
{% endif %}

--== Ansible Jinja2 if elif statement ==--

{% if ansible_hostname == "ubuntu-c" -%}
   This is ubuntu-c
{% elif ansible_hostname == "centos1" -%}
   This is centos1 with it's modified SSH Port
{% endif %}

--== Ansible Jinja2 if elif else statement ==--

{% if ansible_hostname == "ubuntu-c" -%}
   This is ubuntu-c
{% elif ansible_hostname == "centos1" -%}
   This is centos1 with it's modified SSH Port
{% else -%}
   This is good old {{ ansible_hostname }}
{% endif %}

--== Ansible Jinja2 if variable is defined ( where variable is not defined ) ==--

{% if example_variable is defined -%}
   example_variable is defined
{% else -%}
   example_variable is not defined
{% endif %}

--== Ansible Jinja2 if varible is defined ( where variable is defined ) ==--

{% set example_variable = 'defined' -%}
{% if example_variable is defined -%}
   example_variable is defined
{% else -%}
   example_variable is not defined
{% endif %}

--== Ansible Jinja2 for statement ==--

{% for entry in ansible_all_ipv4_addresses -%}
   IP Address entry {{ loop.index }} = {{ entry }}
{% endfor %}

--== Ansible Jinja2 for range

{% for entry in range(1, 11) -%}
   {{ entry }}
{% endfor %}

--== Ansible Jinja2 for range, reversed (simulate while greater 5) ==--

{% for entry in range(10, 0, -1) -%}
   {% if entry == 5 -%}
      {% break %}
   {% endif -%}
   {{ entry }}
{% endfor %}

--== Ansible Jinja2 for range, reversed (continue if odd) ==--

{% for entry in range(10, 0, -1) -%}
   {% if entry is odd -%}
      {% continue %}
   {% endif -%}
   {{ entry }}
{% endfor %}

---=== Ansible Jinja2 filters ===---

--== min [1, 2, 3, 4, 5] ==--

{{ [1, 2, 3, 4, 5] | min }}

--== max [1, 2, 3, 4, 5] ==--

{{ [1, 2, 3, 4, 5] | max }}

--== unique [1, 1, 2, 2, 3, 3, 4, 4, 5, 5] ==--

{{ [1, 1, 2, 2, 3, 3, 4, 4, 5, 5] | unique }}

--== difference [1, 2, 3, 4, 5] vs [2, 3, 4, 5, 6] ==--

{{ [1, 2, 3, 4, 5] | difference([2, 3, 4]) }}

--== random ['rod', 'jane', 'freddy'] ==--

{{ ['rod', 'jane', 'freddy'] | random }}

--== urlsplit hostname ==--

{{ "http://docs.ansible.com/ansible/latest/playbooks_filters.html" | urlsplit('hostname') }}

```

## Section 5: Ansible Playbooks, Deep Dive

I fast-forwarded this section and took notes on the capabilities of Ansible.
Look at the course for concrete coding examples, available at:


`/home/ansible/diveintoansible/Ansible Playbooks, Deep Dive/`


### Ansible Playbook Modules

We have 100s of modules available:

- `set_fact`: we can set pur facts or re-write available ones; often it is used with conditionals, like `when: ansible_distribution == 'Ubuntu'`
- `pause`: playbook execution is paused for a period or until something is introduced after prompting
- `wait_for`: for instance, wait for a port being used (that implicitly means a service is running)
- `assemble`: configurations can bet broken into segments or files, which are later assembled
- `add_host`: dynamically add hosts in a play
- `group_by`: we can create groups of hosts depending on key facts we specify
- `fetch`: capture files from a remote host; it is anologous to `copy`

### Dynamic Inventories

Inventories are basically the hosts.
We can create a script that returns a JSON with the hosts we would like, depending on the arguments we pass to it.
That is the idea of dynamic inventories: we pass to ansible a script that returns the hosts we would like depending on the arguments it receives.

The instructor has a `inventory.py` python script that is used, called through `ansible`:

```bash
ansible all -i inventory.py --list-hosts
```

###  Register and When

We can catch/register the output generated while executing a playbook.

### Looping

We can loop within a playbook with several commands:
`with_items`, `with_dict`, `with_subelements`, `with_together`, `with_sequence`, `with_random_choice`, `until`.

We usually pass a list:
```YAML
with_items:
  - CentOS
  - Ubuntu
```

### Asynchronous, Serial, Parallel

Running tasks asynchronously means running them in parallel, in different shells.
That is achieved with the commands `async` and `poll`, subordinated in a `task`.
That way, it is possible that the launched tasks are still running when the playbook has finished executing.

### Task Delegation

We can assign specific targets/hosts to selected tasks.
That is achieved with `delegate_to` in the `task`.

### Magic Variables

Variables that are automatically made available when the playbook is executed.
These can be used when we want.

There is an ansible script which dumps all magic variables to a file:
`/home/ansible/diveintoansible/Ansible Playbooks, Deep Dive/Magic Variables/01`

```bash
cat dump_vars_playbook.yaml
ansible-playbook dump_vars_playbook.yaml
# a file with available (magic) variables is dumped for each host
cd captures_variables/
ls # centos1  centos2  centos3  ubuntu-c  ubuntu1  ubuntu2  ubuntu3
vim centos1
```

Some magic variables we should have a look at:
- `hostvars`
- `groups`
- `group_names`
- `inventory_hostname`
- `inventory_dir`

### Blocks

We can group tasks into blocks.
For that, we subordinate `block` to the `tasks` section and nest our tasks inside `block`.
Blocks can be used for debugging.

### Vault

We can encrypt/decrypt variables and files that contain sensible information, for example passwords.
The CLI tool `ansible-vault` is used to encrypt a string, with several options such as `encrypt` and `decrypt`, `view`, etc.
That encrypted string can be used as a variable saved in a YAML file;
it will have a concrete format which tells ansible it is encrypted.

## Section 6: Structuring Ansible Playbooks

The examples of this section can be found in

`/home/ansible/diveintoansible/Structuring Ansible Playbooks`

### Includes and Imports

We can use the following instructions in a playbook:
- `include_tasks` and `import_tasks`, followed by a YAML file: both integrate the tasks defined in the YAML file. Note that `include_` is dynamic, ie., the external tasks are loaded during playbook execution, whereas `import_` is static: the external tasks are loaded and integrated before the playbook execution.
- `import_playbook`: an external YAML with the playbook definition is imported (static).

### Tags

Tags can be added to tasks with the key `tags:`.
Then, when we execute a playbook with `ansible-playbook`, we can filter to run the tasks with the tags we'd like; for example, we might want to define tasks for deployment:

`my_playbook.yaml`:

```yaml
...
- name: My task
  ...
  tags:
    - deploy-app
...
```

Then, with `ansible-playbook`:

```bash
ansible-playbook my_playbook.yaml --tags "deploy-app"
```

It is also possible to skip tasks with a tag using `--skip-tags`.

Note that all tasks are assigned by default to the `all` tag.

Tags can be inherited by importer/included tasks/playbooks.

### Roles

Larger projects become easier with roles.
A role extend the concept of tags further to create maximum customization.
A role is basically encoded in a folder structure and it contains a whole definition for a given use case with files; for example:

```bash
example-role/
  README.md
  defaults/
    main.yaml
  files
  handlers/
    main.yaml
  meta/
    main.yaml
  tasks/
    main.yaml
  templates
  tests/
    inventory
    test.yaml
  vasr/
    main.yaml
```

We can create the structure manually or with the CLI tool `ansible-galaxy`.

## Section 7: Using Ansible with Cloud Services and Containers

### 7.1 AWS with Ansible

AWS and Docker interaction are introduced; examples can be found in

`/home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/`

For the usage of Ansible with AWS, a (free) account needs to be set up:
- We log in with a free tier
- We create a key-secret pair
- We `export` the key & the secret to the environment where ansible is run.
- We install boto: `sudo pip3 install boto boto3`

Following the examples, we interact with AWS: we create instances (linux nodes), launch tasks, etc.

After the setup, the configuration files for connecting to AWS need to be downloaded and set up locally: configuration (cfg), inventories, etc.

After setting up everything, we can control/configure our AWS instances using Ansible! All changes are reflected on the AWS web interfaces.

### 7.2 Docker with Ansible

The examples of this section can be found in

`/home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible`

With Ansible, we can do the following things in docker, each contained in the specified example folders:
- Configuration
- Pull images: `01`
- Build containers: `02`
- Build customized images & containers: `03`
- Connect to running containers: `05`
- Terminate and remove docker resources: `06`

Setting up docker:
```bash
# Host/Mac
cd ~/git_repositories/diveintoansible-lab
# We have our docker-compose.yaml in here
docker-compose up
# Open browser on http://localhost:1000/
# Ansible Terminal: ubuntu-c
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible
# In the lab, we have a dedicated docker container running on ubuntu-c
# Check that it is running; Ctrl+C for stopping
ping docker
cd 01/
# To communicate with docker we need more tools
# that are installed with the following script
cat install_docker.sh
# sudo apt update
# sudo apt install -y docker.io
# pip3 install docker
# We install docker with it; pw: password
bash -x install_docker.sh
# We need to set where docker host is
# That is not necessary if docker runs locally
# but in our case we need to do it
# because we're running docker in a container
cat envdocker
# export DOCKER_HOST=tcp://docker:2375
source envdocker
# Now, we can run docker commands
docker ps
```

First example: **Pulling images**
```bash
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible/01
# The first playbook shows how to pull images using our remote docker
# If we used a local docker, no remote host would be required in the YAML
# Note that one of the images is very large (>1GB)
cat docker_playbook.yaml # see below
ansible-playbook docker_playbook.yaml
# After pulling the images, we can test them with the commands in this file
cat examples.txt
# docker run --rm -it wernight/funbox cmatrix # Matrix screen shown
# docker run --rm -it wernight/funbox nyancat
# docker run --rm -it wernight/funbox asciiquarium
```

`docker_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: ubuntu-c
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Pull images
      docker_image:
        docker_host: tcp://docker:2375
        name: "{{ item }}"
        source: pull
      with_items:
        - centos
        - ubuntu
        - redis
        - nginx
        # n.b. large image, >1GB
        - wernight/funbox

# Three dots indicate the end of a YAML document
...
```

Second example: We **pull** the nginx image and **build a container** with it.
```bash
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible/02
cat docker_playbook.yaml # see below
ansible-playbook docker_playbook.yaml
# We check we have now a running container
docker ps -a
```

`docker_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: ubuntu-c
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Pull images
      docker_image:
        docker_host: tcp://docker:2375
        name: "{{ item }}"
        source: pull
      with_items:
        - nginx

    - name: Create an nginx container
      docker_container:
        docker_host: tcp://docker:2375
        name: containerwebserver
        image: nginx
        ports:
          - 80:80
        container_default_behavior: no_defaults

# Three dots indicate the end of a YAML document
...
```

Third example: A **Dockerfile is created** based on a **pulled image** and its **container is built**:
```bash
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible/03
cat docker_playbook.yaml # see below
ansible-playbook docker_playbook.yaml
# We check the images: our new image should be there
docker image ls
```

`docker_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: ubuntu-c
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Pull images
      docker_image:
        docker_host: tcp://docker:2375
        name: "{{ item }}"
        source: pull
      with_items:
        - nginx

    - name: Create a customised Dockerfile
      copy:
        dest: /shared/Dockerfile
        mode: 0644
        content: |
          FROM nginx

    - name: Build a customised image
      docker_image:
        docker_host: tcp://docker:2375
        name: nginxcustomised:latest
        source: build
        build:
          path: /shared
          pull: yes
        state: present
        force_source: yes

    - name: Create an nginxcustomised container
      docker_container:
        docker_host: tcp://docker:2375
        name: containerwebserver
        image: nginxcustomised:latest
        ports:
          - 80:80
        container_default_behavior: no_defaults
        recreate: yes

# Three dots indicate the end of a YAML document
...
```

Fourth example: we **connect to docker containers with Ansible as if they were targets (hosts/nodes)**:
```bash
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible/05
# Check YAML
# We need a python image/container which is running continuously,
# thus we run the command: sleep infinity
cat docker_playbook.yaml # see below
# Ansible connection is enabled using python containers as hosts 
# Note that 3 python containers are stated in a sequence,
# these are also reflected in the YAML file
cat hosts # see below
# We run the YAML file
# Two playbooks are in it: (1) python image/container setup (2) ping to python containers as a task
ansible-playbook docker_playbook.yaml
# We check the images: our new images should be there
docker image ls
# Check new running containers
docker ps -a
```

`hosts`:
```bash
[control]
ubuntu-c

[centos]
centos[1:3]

[ubuntu]
ubuntu[1:3]

[linux:children]
centos
ubuntu

[containers]
python[1:3] ansible_connection=docker ansible_python_interpreter=/usr/bin/python3
```

`docker_playbook.yaml`:
```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: ubuntu-c
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Pull python image
      docker_image:
        docker_host: tcp://docker:2375
        name: python:3.8.5
        source: pull

    - name: Create 3 python containers
      docker_container:
        docker_host: tcp://docker:2375
        name: "python{{ item }}"
        image: python:3.8.5
        container_default_behavior: no_defaults
        command: sleep infinity
      with_sequence: 1-3
-

  # Hosts: where our play will run and options it will run with
  hosts: containers
  gather_facts: False
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:
    - name: Ping containers
      ping:

# Three dots indicate the end of a YAML document
...
```

Fifth example: **remove/clean up docker stuff**: images, containers and files
```bash
cd /home/ansible/diveintoansible/Using Ansible with Cloud Services and Containers/Docker with Ansible/06
cat docker_playbook.yaml # see below
ansible-playbook docker_playbook.yaml
# We check that the images were removed
docker image ls
# We check that the containers were deleted
docker container ls -a
```

```yaml
---
# YAML documents begin with the document separator ---

# The minus in YAML this indicates a list item.  The playbook contains a list
# of plays, with each play being a dictionary
-

  # Hosts: where our play will run and options it will run with
  hosts: ubuntu-c
  
  # Tasks: the list of tasks that will be executed within the play, this section
  # can also be used for pre and post tasks
  tasks:

    - name: Remove old containers
      docker_container:
        docker_host: tcp://docker:2375
        name: "{{ item }}"
        state: absent
        container_default_behavior: no_defaults
      with_items:
        - containerwebserver
        - python1
        - python2
        - python3

    - name: Remove images
      docker_image:
        docker_host: tcp://docker:2375
        name: "{{ item }}"
        state: absent
      with_items:
        - nginx
        - nginxcustomised
        - python:3.8.5

    - name: Remove files
      file:
        path: "{{ item }}"
        state: absent
      with_items:
        - /shared/Dockerfile
        - /shared/index.html

# Three dots indicate the end of a YAML document
...

```

## Section 8: Creating Modules and Plugins

### Creating Modules

We can download teh Ansible source code (clone from github) and create our own modules based on the official modules.
There is a `hacking/test-module` that can be used as a blueprint.
A module can be written in any language (python, bash), but it must return a JSON output.
There is also a template and documentation online.
If we follow the guidelines we also get the documentation of our custom modules when running `ansible-doc`.

### Creating Plugins

As far as I understand, while modules are operations run in tasks, plugins are functionalities that affect any module or operation.
Ansible has many plugins, for example: `with_items`.
We can also create our own ones, for example: `with_sorted_items` (this one is created in the course).


## Section 9: My personal summary / notes

TBD