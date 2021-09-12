# Vagrant: A Brief Guide

I made this guide following the tutorial the Youtube channel by [makigas.es](https://www.makigas.es):

[Vagrant - Tutorial by makigas.es](https://www.youtube.com/watch?v=Ud7cmVCNACE&list=PLTd5ehIj0goPCodyeh2ThX37Ceh-2torY)

Overview:
1. 

Mikel Sagardia, 2021. No warranties.

## 1. What is Vagrant

Vagrant is a tool to define the creation of virtual machines in a declarative manner: we define in a configuration file the VMs with the characteristics we want and we run it; since it is declarative, we do not need to perform all the process of the creation, but we just define the end state of the VM we'd like and Vagrant manages to create them.

Typical configurations we can define:
- CPU cores
- OS
- RAM
- Virtual networks
- Exposed ports
- Install programs
- Execute Ansible playbooks for provisioning

Since it declarative, we can very easily upload the configuration file to a repository. That way, the VM/machine characteristics can be made constant between teams working on the same repository.

Vagrant does not provide VMs, but it uses VM providers. It supports **VirtualBox** and **Docker** as VM providers, among others.

## 2. Installing Vagrant

Official website: [Vagrant](https://www.vagrantup.com/)

MacOS:

```bash
brew install vagrant
```

Windows: download binary.
Linux: some `apt-get` commands, look up on the web.

To execute it, type in the Terminal

```bash
vagrant # menu appears
vagrant version # version displayed: 2.2.18
```

## 3. Vagrant Boxes

Vagrant has online in their servers so called **boxes**, which are initialized images of operative systems (with users, etc.). We can download those templates and use them to create our own.

Under [Vagrant: Find Boxes](https://app.vagrantup.com/boxes/search) we can find the boxes that are available; official ones are usually created by vendors (eg., Ubuntu). We can manually select the provider: **virtualbox**, **docker**, etc.

We can manually download/clone them from the web interface of via command line.

```bash
# list all 
vagrant box list
# look on the URL above the available VMs
# add the selected one: ubuntu/focal64 (Official Ubuntu 20.04 LTS Focal Fossa)
# watch out: boxes might have a large size
vagrant box add ubuntu/focal64
# remove a box
vagrant box list
vagrant box remove ubuntu/focal64
```

Note that we can use also VMs we have created as template, and it is possible to download VM boxes from many other places, such as Microsoft!

## 4. Start and Stop Virtual Machines (VMs)

### Configure and Start

For starting a VM, we need to have installed VirtualBox/Docker and define a configuration file which is passed to Vagrant.

```bash
# go to our desired working difectory
cd git_repositories/templates/vagrant
# we create an empty configuration file: Vagrantfile
# the file is written in Ruby, but we don't need to know it
# because it is very simple and we just uncomment & modify stuff in it
vagrant init
# edit Vagrantfile; see below
vim Vagrantfile
# after editting
# we can upload the file to our repo
# without needing to upload the box!
git add Vagrant file
git commit -m "first version of our VM"
# start VM: Vagrantfile in folder loaded and VM created
vagrant up
# if we open VirtualBox, we're going to see our VM running there
# everything has been configuraed automatically:
# user, ports, SSH keys, etc.
```

Part modified in `Vagrantfile`:
```ruby
# ...
# set our box
config.vm.box = "ubuntu/focal64"
# ...
```

[Vagrant documentation](https://www.vagrantup.com/docs) collects all things we can configure. The most common ones are commented in the `Vagrantfile`:
- Forwarded port mapping between host and VM
- Create a private network
- Create a public network
- Share a folder between host and VM
- Modify VM properties for VirtualBox: GUI yes/no, RAM size
- Perform provisioning with Shell scripts of Ansible

### SSH

During the start up of the VM, we can see its IP address; we could `ssh` to it, but usually `vagrant ssh` is used instead, which eases some things (for instance, no user & pw are needed).

```bash
# ssh to our VM: vagrant@ubuntu-focal
vagrant ssh
# we can even do sudo without pw
sudo su # root@ubuntu-focal
# exit ssh shell
exit
exit
```

### Stop

We can stop the VM with `vagrant halt` and start it again with `vagrant up`.

```bash
# stop VM
vagrant halt
# start VM again
vagrant up
# a folder named ./.vagrant/ will be created
# which contains information used during runtime: 
#
# we can destroy a VM
# and it won't appear in VirtualBox
vagrant destroy
```

## 5. Provisioning with Shell Scripts

Usually, we create the VM as a server for a purpose; for that purpose, we need to install, configure and use the required tools.
That is what is achieved by the **provisioning** step.
Provisioninig can be done with specific tools, like Ansible, or with shell scripts.

The sample `Vagrant` file already has the recipy for inserting our provisioning shell scripts:

```ruby
# ...
config.vm.provision "shell", inline: <<-SHELL
  # Now, we write all the shell commands we require line by line
  apt-get update
  # Scripts cannot be interactive,
  # thus, it is important to add -y for install commands:
  # as if we press Y for all questions;
  # we must avoid/circumvent any command that requires user interaction
  apt-get install -y apache2
  apt-get install -y nginx
SHELL
# ...
```

If we run our `Vagrant` file now, we're going to get a VM with `apache2` and `nginx`.
Notes:
- The shell language used must match the one of the host computer: Mac: `bash`/`zsh`, Windows: `Powershell`, Linux: `bash`.
- The provisioning script is run once, on creation; if we remove a packaged installed during provisioning and restart the VM, it won't have the package anymore, unless we explicitly add the option `--provision`

```bash
# Remove anything running
vagrant destroy
# Start VM
# Note that we're going to see green lines: that's apt-get working
vagrant up
# We connect to the VM
vagrant ssh
# Check that nginx is running
curl http://localhost
# Provisioning scripts are run on ly once
# If we uninstall a package installed during provisioning
# it won't be available if we restart the VM
# unless we do `vagrant up --provision` or `vagrant provision`
sudo apt purge nginx
exit
vagrant halt
vagrant up
```

Instea of writing the complete provisioning commands in the `Vagrant` file,
we can also import provisioning shell scripts.

```ruby
config.vm.provision "shell", path: 'php_setup.sh'
```

`php_setup.h`:
```bash
#!/bin/sh

apt-get update
apt-get install -y php5-fmp
```

## 6. Provisioning with Ansible Playbooks

Ansible is a modern provisioning tool, more powerful than shell scripts, because it is declarative.
With shell scripts we need to specifically define all steps for installation and configuration (it is imperative); with Ansible, we just define the end-state and the tool managaes to bring up a VM with those characteristics (declarative).

See for more information on Ansible:
`git_repositories/templates/ansible/ansible_howto.md`

In order to use Ansible playbooks, we need to change the provisioning configuration in the `Vagrant` file from `"shell"` to `"ansible"`:

```ruby
#...
config.vm.provision "ansible" do |ansible|
  ansible.playbook = "my_playbook.yml"
end
#...
```

`my_playbook.yml`:
```YAML
---
- hosts: all
  # run as sudo
  become: true
  tasks:
    - name: Install git
      apt: name=git state=installed update_cache=true
    - name: Install tools for the web app
      apt: name={{ item }} state=installed
      with_items:
        - postgresql
        - php5-fmp
    - name: Create an user for my app
      user: name=app comment="user of the app"
...
```

Note that we can configure more stuff in the `|ansible|` block, look at the documentation.
