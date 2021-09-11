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

Part modifief in `Vagrantfile`:
```ruby
# set our box
config.vm.box = "ubutu/focal64"
```

